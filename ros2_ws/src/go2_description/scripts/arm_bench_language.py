#!/usr/bin/env python3
"""Chinese natural-language command shell for the arm bench.

Rules:
    REPL keeps reading commands until the user says "结束了".
    Every motion command shows the parsed plan and asks for confirmation.

Examples:
    抓红球
    去抓红球,放到黄盒子上
    把蓝方块放到绿圆柱旁边
    抓红球放到黄盒子上,然后抓蓝方块放到绿圆柱旁边
    桌上有什么
"""
import re
import signal
import subprocess
import sys


from arm_bench_visual_grasp import OBJECTS

LABEL_CN = {
    'red_ball': '红球',
    'blue_cube': '蓝方块',
    'green_cylinder': '绿圆柱',
    'yellow_box': '黄盒子',
}
ALIASES = [
    ('红色的球', 'red_ball'), ('红色球', 'red_ball'), ('红球', 'red_ball'),
    ('小球', 'red_ball'), ('球', 'red_ball'),
    ('蓝色的方块', 'blue_cube'), ('蓝色方块', 'blue_cube'),
    ('蓝色立方体', 'blue_cube'), ('蓝方块', 'blue_cube'), ('蓝块', 'blue_cube'),
    ('绿色的圆柱', 'green_cylinder'), ('绿色圆柱', 'green_cylinder'),
    ('绿圆柱', 'green_cylinder'), ('圆柱', 'green_cylinder'),
    ('黄色的盒子', 'yellow_box'), ('黄色盒子', 'yellow_box'),
    ('黄盒子', 'yellow_box'), ('黄盒', 'yellow_box'),
    ('黄色方块', 'yellow_box'),
]
PLACE_KEYS = ('放到', '放在', '摆到', '搁到', '拿到', '移到', '移动到')
PICK_VERBS = ('抓', '拿', '取', '夹')
YES_WORDS = ('y', 'yes', '是', '确认', '执行', '可以', '好', '行', 'ok')
EXIT_WORDS = ('结束了', '结束', '退出', '再见', '拜拜', 'quit', 'exit', 'q')
STOP_WORDS = ('停止', '停', '暂停', '取消')
QUERY_WORDS = ('有什么物体', '桌上有什么', '查询物体', '列出物体',
               '看看有什么', '有什么东西', '检测一下', '有什么', '有哪些')
HELP_WORDS = ('帮助', 'help', '怎么用', '?', '？')

FULL2HALF = {
    '，': ',', '。': '.', '！': '!', '？': '?', '；': ';', '：': ':',
    '（': '(', '）': ')', '【': '[', '】': ']', '“': '"', '”': '"',
    '‘': "'", '’': "'", '、': ',',
}

HELP = """
arm_bench 自然语言控制
  抓取类:
    抓红球
    去抓红球,放到黄盒子上
    把蓝方块放到绿圆柱旁边
    抓红球放到黄盒子上,然后抓蓝方块放到绿圆柱旁边
  查询/控制:
    桌上有什么 | 查询物体          (会环视扫描台面)
    帮助
    停止              (空闲时)
    Ctrl+C            中止当前执行
    结束了 / 退出 / q  (结束程序)
  物体: 红球 / 蓝方块 / 绿圆柱 / 黄盒子
  说明:
    - 多步指令用“然后 / 再 / 接着 / ；”连接
    - 动作执行前会显示解析结果并请求确认,默认不执行
    - “放到...上”是堆叠(仅支持方块/黄盒子),“放到...旁边”自动找 3cm 空位
"""


class CommandError(Exception):
    pass


def _normalize(text):
    text = text.strip().lower().translate(str.maketrans(FULL2HALF))
    return re.sub(r'\s+', '', text)


def _mentions(text):
    found = []
    aliases = sorted(ALIASES, key=lambda item: -len(item[0]))
    i = 0
    while i < len(text):
        for alias, label in aliases:
            if text.startswith(alias, i):
                found.append((i, i + len(alias), label))
                i += len(alias)
                break
        else:
            i += 1
    return found


def _place_mode(after, mention_end):
    tail = after[mention_end:]
    if any(w in tail for w in ('旁边', '旁', '边上', '侧', '附近', '左边', '右边')):
        return 'beside'
    if any(w in tail for w in ('上', '顶', '里')):
        return 'on_top'
    return 'on_top'


def _parse_clause(text):
    place_pos = -1
    place_key = None
    for key in PLACE_KEYS:
        pos = text.find(key)
        if pos >= 0 and (place_pos < 0 or pos < place_pos):
            place_pos, place_key = pos, key
    if place_pos < 0:
        mentioned = _mentions(text)
        if not mentioned:
            raise CommandError('没有识别到物体。可用: 红球、蓝方块、绿圆柱、黄盒子')
        if len(mentioned) > 1:
            raise CommandError('一次只能抓一个物体')
        return {'action': 'pick', 'pick': mentioned[0][2]}

    before = text[:place_pos]
    after = text[place_pos + len(place_key):]
    picks = _mentions(before)
    places = _mentions(after)
    if not picks:
        if places:
            return {'action': 'place', 'place': places[0][2],
                    'mode': _place_mode(after, places[0][1])}
        raise CommandError('没说抓什么、也没说放哪里')
    if not places:
        raise CommandError('没说放到哪里')
    if len(picks) > 1:
        raise CommandError('一次只能抓一个物体')
    if len(places) > 1:
        raise CommandError('一次只能放到一个物体上/旁边')
    return {
        'action': 'pick_place',
        'pick': picks[0][2],
        'place': places[0][2],
        'mode': _place_mode(after, places[0][1]),
    }


def _parse_parts(parts):
    actions = []
    for part in parts:
        part = part.strip(' ,')
        if not part:
            continue
        action = _parse_clause(part)
        if action['action'] == 'place':
            if actions and actions[-1]['action'] == 'pick':
                prev = actions.pop()
                actions.append({
                    'action': 'pick_place', 'pick': prev['pick'],
                    'place': action['place'], 'mode': action['mode'],
                })
            else:
                raise CommandError('放置指令前没有抓取动作')
        else:
            actions.append(action)
    if not actions:
        raise CommandError('没听懂')
    return actions


def parse_command(line):
    text = _normalize(line)
    if not text:
        raise CommandError('空指令')
    if text in EXIT_WORDS:
        return 'exit', None
    if text in HELP_WORDS:
        return 'help', None
    if text in STOP_WORDS:
        return 'stop', None
    if any(word in text for word in QUERY_WORDS):
        return 'query', None
    if any(color in text for color in '橙紫黑粉棕灰青白') and _mentions(text):
        raise CommandError('只认识红/蓝/绿/黄四种颜色')

    parts = []
    for chunk in re.split(r'(?:然后|接着|；|;)', text):
        segments = re.split(
            r'再(?=(?:抓|拿|取|把|夹|帮|请|去))', chunk)
        parts.extend(segment for segment in segments if segment)

    try:
        actions = _parse_parts(parts)
    except CommandError as exc:
        # A comma can separate two complete actions; a comma inside one
        # pick-and-place clause should not split it.
        comma_parts = [p for p in text.split(',') if p]
        if len(comma_parts) > 1:
            try:
                comma_actions = _parse_parts(comma_parts)
                if all(a['action'] == 'pick_place' for a in comma_actions):
                    actions = comma_actions
                else:
                    raise exc
            except CommandError:
                raise exc
        else:
            raise

    for idx, action in enumerate(actions):
        if action['action'] == 'pick' and idx != len(actions) - 1:
            raise CommandError('多步指令里每一步都要放到某个位置')
    return 'actions', actions


def validate_actions(actions):
    for action in actions:
        if action['pick'] not in OBJECTS:
            return '未知物体 %s' % action['pick']
        if action['action'] == 'pick_place':
            if action['place'] not in OBJECTS:
                return '未知物体 %s' % action['place']
            if action['pick'] == action['place']:
                return '抓取和放置目标不能是同一个物体'
            if action['mode'] == 'on_top' and \
                    OBJECTS[action['place']]['shape'] != 'box':
                return ('%s 顶面太小,不适合堆叠;请说“放到%s旁边”'
                        % (LABEL_CN[action['place']],
                           LABEL_CN[action['place']]))
    return None


def describe(actions):
    lines = []
    for idx, action in enumerate(actions, 1):
        if action['action'] == 'pick':
            lines.append('%d. 抓取 %s(%s)' %
                         (idx, LABEL_CN[action['pick']], action['pick']))
        else:
            mode = '上方(堆叠)' if action['mode'] == 'on_top' \
                else '旁边(3cm 空位)'
            lines.append('%d. 抓取 %s(%s) → 放到 %s(%s) %s' %
                         (idx, LABEL_CN[action['pick']], action['pick'],
                          LABEL_CN[action['place']], action['place'], mode))
    return '\n'.join(lines)


class Shell:

    def run_action(self, action):
        params = ['--ros-args', '-p', 'target_class:=%s' % action['pick']]
        if action['action'] == 'pick_place':
            params += [
                '-p', 'place_class:=%s' % action['place'],
                '-p', 'place_mode:=%s' % action['mode'],
                '-p', 'place_offset:=0.03',
            ]
        cmd = ['ros2', 'run', 'go2_description',
               'arm_bench_visual_grasp.py'] + params
        print('  执行: %s' % ' '.join(cmd))
        return self._run(cmd)

    def run_scan(self):
        cmd = ['ros2', 'run', 'go2_description',
               'arm_bench_visual_grasp.py', '--ros-args',
               '-p', 'scan_only:=true']
        print('  执行: %s' % ' '.join(cmd))
        return self._run(cmd)

    @staticmethod
    def _run(cmd):
        proc = subprocess.Popen(cmd)
        try:
            return proc.wait()
        except KeyboardInterrupt:
            proc.send_signal(signal.SIGINT)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
            return 130


def main():
    shell = Shell()
    print(HELP)
    try:
        while True:
            try:
                line = input('指令> ')
            except EOFError:
                break
            except KeyboardInterrupt:
                print('\n已退出。')
                break
            try:
                kind, payload = parse_command(line)
            except CommandError as exc:
                print('没听懂: %s' % exc)
                print('示例: 去抓红球,放到黄盒子上')
                continue
            if kind == 'exit':
                print('再见。')
                break
            if kind == 'help':
                print(HELP)
                continue
            if kind == 'query':
                shell.run_scan()
                continue
            if kind == 'stop':
                print('当前没有正在执行的任务。')
                continue

            error = validate_actions(payload)
            if error:
                print('不能执行: %s' % error)
                continue
            print('解析结果:')
            print(describe(payload))
            for action in payload:
                if (action['action'] == 'pick_place' and
                        action['mode'] == 'on_top' and
                        OBJECTS[action['pick']]['shape'] == 'sphere'):
                    print('注意: %s 是球形,叠放到平面上可能会滚落;'
                          '建议用方块或改说“放到...旁边”'
                          % LABEL_CN[action['pick']])
            answer = input('确认执行? [y/N] ').strip().lower()
            if answer not in YES_WORDS:
                print('已取消。')
                continue

            success = True
            for idx, action in enumerate(payload, 1):
                print('\n[%d/%d] %s' %
                      (idx, len(payload), describe([action])))
                code = shell.run_action(action)
                if code == 130:
                    print('已中止当前任务。')
                    success = False
                    break
                if code != 0:
                    print('第 %d 步失败 (exit %d),已停止。' % (idx, code))
                    success = False
                    break
            if success:
                print('\n完成。')
    finally:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())

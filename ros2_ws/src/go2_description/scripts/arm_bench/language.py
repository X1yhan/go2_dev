"""Chinese command parsing for the arm bench.  Stdlib only, no ROS.

Layer L5.  Parsing is pure text work, so it can be tested without a
simulator:

    python3 -c "from arm_bench.language import parse_command; \\
                print(parse_command('抓红球放到黄盒子上'))"

The REPL and the subprocess calls that actually move the arm stay in
`arm_bench_language.py`.
"""
import re

from arm_bench.objects import LABEL_CN, OBJECTS

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

BESIDE_HINTS = ('旁边', '旁', '边上', '侧', '附近', '左边', '右边')
ON_TOP_HINTS = ('上', '顶', '里')
OTHER_COLOURS = '橙紫黑粉棕灰青白'


class CommandError(Exception):
    pass


def _normalize(text):
    text = text.strip().lower().translate(str.maketrans(FULL2HALF))
    return re.sub(r'\s+', '', text)


def _mentions(text):
    """(start, end, label) for every object mention, longest alias first."""
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
    if any(word in tail for word in BESIDE_HINTS):
        return 'beside'
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
                previous = actions.pop()
                actions.append({
                    'action': 'pick_place', 'pick': previous['pick'],
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
    """Text -> ('exit'|'help'|'stop'|'query'|'actions', payload)."""
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
    if any(colour in text for colour in OTHER_COLOURS) and _mentions(text):
        raise CommandError('只认识红/蓝/绿/黄四种颜色')

    parts = []
    for chunk in re.split(r'(?:然后|接着|；|;)', text):
        segments = re.split(r'再(?=(?:抓|拿|取|把|夹|帮|请|去))', chunk)
        parts.extend(segment for segment in segments if segment)

    try:
        actions = _parse_parts(parts)
    except CommandError as exc:
        # A comma can separate two complete actions; a comma inside one
        # pick-and-place clause should not split it.
        comma_parts = [part for part in text.split(',') if part]
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

    for index, action in enumerate(actions):
        if action['action'] == 'pick' and index != len(actions) - 1:
            raise CommandError('多步指令里每一步都要放到某个位置')
    return 'actions', actions


def validate_actions(actions):
    """Return a human readable reason, or None when the plan is runnable."""
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
    for index, action in enumerate(actions, 1):
        if action['action'] == 'pick':
            lines.append('%d. 抓取 %s(%s)' %
                         (index, LABEL_CN[action['pick']], action['pick']))
        else:
            mode = '上方(堆叠)' if action['mode'] == 'on_top' \
                else '旁边(3cm 空位)'
            lines.append('%d. 抓取 %s(%s) → 放到 %s(%s) %s' %
                         (index, LABEL_CN[action['pick']], action['pick'],
                          LABEL_CN[action['place']], action['place'], mode))
    return '\n'.join(lines)


def warns_sphere_on_top(actions):
    """True if stacking a sphere was requested (it may roll off)."""
    return any(action['action'] == 'pick_place' and
               action['mode'] == 'on_top' and
               OBJECTS[action['pick']]['shape'] == 'sphere'
               for action in actions)

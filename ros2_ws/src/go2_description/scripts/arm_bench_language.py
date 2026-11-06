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

The parsing itself lives in `arm_bench/language.py` and imports nothing
from ROS, so the language layer no longer drags MoveIt and OpenCV into
this process.  What stays here is the shell: printing, confirmation, and
starting one child process per action.
"""
import os
import signal
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from arm_bench.language import (  # noqa: E402
    HELP, YES_WORDS, CommandError, describe, parse_command, validate_actions,
    warns_sphere_on_top)
from arm_bench.objects import LABEL_CN  # noqa: E402


class Shell:

    def run_action(self, action):
        params = ['--ros-args', '-p', 'target_class:=%s' % action['pick']]
        if action['action'] == 'pick_place':
            params += [
                '-p', 'place_class:=%s' % action['place'],
                '-p', 'place_mode:=%s' % action['mode'],
                '-p', 'place_offset:=0.05',
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
            if warns_sphere_on_top(payload):
                sphere = next(action['pick'] for action in payload
                              if warns_sphere_on_top([action]))
                print('注意: %s 是球形,叠放到平面上可能会滚落;'
                      '建议用方块或改说“放到...旁边”' % LABEL_CN[sphere])
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

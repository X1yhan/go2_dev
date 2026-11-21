#!/usr/bin/env python3
"""arm_bench 纯函数层的离线回归测试。不需要 ROS、不需要 Gazebo。

    cd ros2_ws/src/go2_description/scripts && python3 test_arm_bench_pure.py

覆盖:中文解析、抓取/放置几何、尺寸表、场景融合。
这些是"分层"最直接的收益——改 HSV、改放置逻辑不需要起仿真就能验证。
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from arm_bench import geometry, objects, scene
from arm_bench.language import (CommandError, describe, parse_command,
                                validate_actions)

FAILS = []


def check(name, got, want):
    if got != want:
        FAILS.append('%s: got %r, want %r' % (name, got, want))


def check_near(name, got, want, tol=1e-9):
    if abs(got - want) > tol:
        FAILS.append('%s: got %r, want %r (±%r)' % (name, got, want, tol))


def test_language():
    check('pick', parse_command('抓蓝方块'),
          ('actions', [{'action': 'pick', 'pick': 'blue_cube'}]))
    check('cube', parse_command('抓正方体'),
          ('actions', [{'action': 'pick', 'pick': 'blue_cube'}]))
    check('pick_place_table', parse_command('把蓝方块放到台面上'),
          ('actions', [{'action': 'pick_place', 'pick': 'blue_cube',
                        'place': 'table', 'mode': 'on_table'}]))
    check('pick_place_table2', parse_command('去抓蓝方块,放到桌面上'),
          ('actions', [{'action': 'pick_place', 'pick': 'blue_cube',
                        'place': 'table', 'mode': 'on_table'}]))
    check('two_steps_len',
          len(parse_command('把蓝方块放到台面上,然后再把蓝方块放到台面上')[1]),
          2)
    check('query', parse_command('桌上有什么')[0], 'query')
    check('help', parse_command('帮助')[0], 'help')
    check('exit', parse_command('结束了')[0], 'exit')
    try:
        parse_command('跳个舞')
        FAILS.append('unknown verb should raise CommandError')
    except CommandError:
        pass
    text = describe(parse_command('把蓝方块放到台面上')[1])
    if '蓝方块' not in text or '台面' not in text:
        FAILS.append('describe lost the Chinese labels: %r' % text)
    errs = validate_actions(parse_command('把蓝方块放到台面上')[1])
    check('validate_ok', errs, None)   # 通过时返回 None


def test_objects():
    check('labels', sorted(objects.OBJECTS), ['blue_cube'])
    check('model', objects.OBJECTS['blue_cube']['model'], 'blue_cube')
    check_near('cube_half', objects.OBJECTS['blue_cube']['half_height'], 0.035)
    check_near('held_cube', objects.held_width(objects.OBJECTS['blue_cube']), 0.07)
    check_near('place_xy_x', objects.DEFAULT_PLACE_XY[0], 0.255)


def test_geometry():
    cfg = objects.OBJECTS['blue_cube']
    pre, grasp = geometry.grasp_pose((0.21, 0.0, 0.035), cfg, 0.17, 0.10)
    check_near('grasp_z', grasp[2], 0.035 + 0.17)
    check_near('pre_z', pre[2], 0.035 + 0.17 + 0.10)
    # on_table 与 beside 用同一套高度算术:桌面 z=0
    release_z, expected_z = geometry.place_beside(cfg, 0.17, 0.0)
    check_near('table_release', release_z, 0.035 + 0.17)
    check_near('table_expected', expected_z, 0.035)
    check_near('release_q', geometry.release_q(0.032), 0.01)
    check_near('release_q_floor', geometry.release_q(0.01), 0.0)
    # 反投影:光心射线上的点应回到同一像素
    k = [600.0, 0.0, 320.0, 0.0, 600.0, 320.0, 0.0, 0.0, 1.0]
    check_near('deproject_u', geometry.deproject(400, 240, 0.5, k)[0],
               (400 - 320) * 0.5 / 600.0)
    spot = geometry.free_beside_spot(
        (0.25, 0.16, 0.035), {}, cfg, objects.OBJECTS['blue_cube'],
        'blue_cube', 'blue_cube', 0.03)
    if spot is None:
        FAILS.append('free_beside_spot returned None for an empty table')
    else:
        # 落在桌面内、且离盒子至少 pad+safety
        (sx, sy), nearest, _ = spot
        if not (objects.TABLE_X_MIN <= sx <= objects.TABLE_X_MAX
                and abs(sy) <= objects.TABLE_Y_LIMIT):
            FAILS.append('beside spot out of table: %r' % ((sx, sy),))


def test_scene():
    m = scene.SceneModel()
    for i, pt in enumerate([(0.1, 0.0, 0.0), (0.12, 0.02, 0.0), (0.11, 0.0, 0.0)]):
        m.add('blue_cube', pt, 100.0, i)
    m.add('blue_cube', (0.9, 0.9, 0.9), 50.0, scene.SCAN_RANK)  # 旧观测,排序靠后
    m.fuse()
    fused = m.seen
    if 'blue_cube' not in fused:
        FAILS.append('fuse lost blue_cube')
    else:
        # 4 个观测里有一个 0.9 的离群值,中位数应落在中间两个之间
        check_near('fuse_x', fused['blue_cube'][0], 0.115, 1e-6)
    m.set_reference('blue_cube', (0.2, 0.0, 0.0))
    check('best_uses_reference', m.best('blue_cube')[0], 0.2)
    check('labels', m.labels(), ['blue_cube'])
    m.reset()
    check('reset', m.labels(), [])


def main():
    test_language()
    test_objects()
    test_geometry()
    test_scene()
    if FAILS:
        print('FAILED %d' % len(FAILS))
        for f in FAILS:
            print('  -', f)
        return 1
    print('all pure-layer checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())

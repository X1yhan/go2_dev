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
    check('pick', parse_command('抓红球'),
          ('actions', [{'action': 'pick', 'pick': 'red_ball'}]))
    check('pick_place', parse_command('去抓红球,放到黄盒子上'),
          ('actions', [{'action': 'pick_place', 'pick': 'red_ball',
                        'place': 'yellow_box', 'mode': 'on_top'}]))
    check('beside', parse_command('把蓝方块放到绿圆柱旁边'),
          ('actions', [{'action': 'pick_place', 'pick': 'blue_cube',
                        'place': 'green_cylinder', 'mode': 'beside'}]))
    two = parse_command('抓红球放到黄盒子上,然后抓蓝方块放到绿圆柱旁边')[1]
    check('two_steps', len(two), 2)
    check('query', parse_command('桌上有什么')[0], 'query')
    check('help', parse_command('帮助')[0], 'help')
    check('exit', parse_command('结束了')[0], 'exit')
    try:
        parse_command('跳个舞')
        FAILS.append('unknown verb should raise CommandError')
    except CommandError:
        pass
    text = describe(parse_command('抓红球,放到黄盒子上')[1])
    if '红球' not in text or '黄盒子' not in text:
        FAILS.append('describe lost the Chinese labels: %r' % text)
    errs = validate_actions(parse_command('抓红球,放到黄盒子上')[1])
    check('validate_ok', errs, None)   # 通过时返回 None


def test_objects():
    check('labels', sorted(objects.OBJECTS), sorted(
        ['red_ball', 'blue_cube', 'green_cylinder', 'yellow_box']))
    check('model', objects.OBJECTS['red_ball']['model'], 'target_ball')
    check_near('ball_r', objects.OBJECTS['red_ball']['radius'], 0.035)
    check_near('held_sphere', objects.held_width(objects.OBJECTS['red_ball']), 0.07)
    check_near('held_box', objects.held_width(objects.OBJECTS['yellow_box']), 0.10)


def test_geometry():
    cfg = objects.OBJECTS['red_ball']
    pre, grasp = geometry.grasp_pose((0.2, 0.0, 0.035), cfg, 0.17, 0.10)
    check_near('grasp_z', grasp[2], 0.035 + 0.17)
    check_near('pre_z', pre[2], 0.035 + 0.17 + 0.10)
    release_z, expected_z = geometry.place_on_top(
        (0.25, 0.16, 0.035), cfg, objects.OBJECTS['yellow_box'], 0.17, 0.0)
    check_near('on_top_release', release_z, 0.035 + 0.035 + 0.035 + 0.17)
    # expected = 盒顶(0.07) + 球半高(0.035)
    check_near('on_top_expected', expected_z, 0.105)
    beside_z, beside_expected = geometry.place_beside(cfg, 0.17, 0.0)
    check_near('beside_release', beside_z, 0.035 + 0.17)
    check_near('beside_expected', beside_expected, 0.035)
    check_near('release_q', geometry.release_q(0.032), 0.01)
    check_near('release_q_floor', geometry.release_q(0.01), 0.0)
    # 反投影:光心射线上的点应回到同一像素
    k = [600.0, 0.0, 320.0, 0.0, 600.0, 320.0, 0.0, 0.0, 1.0]
    check_near('deproject_u', geometry.deproject(400, 240, 0.5, k)[0],
               (400 - 320) * 0.5 / 600.0)
    spot = geometry.free_beside_spot(
        (0.25, 0.16, 0.035), {}, cfg, objects.OBJECTS['yellow_box'],
        'yellow_box', 'red_ball', 0.03)
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
        m.add('red_ball', pt, 100.0, i)
    m.add('red_ball', (0.9, 0.9, 0.9), 50.0, scene.SCAN_RANK)  # 旧观测,排序靠后
    m.fuse()
    fused = m.seen
    if 'red_ball' not in fused:
        FAILS.append('fuse lost red_ball')
    else:
        # 4 个观测里有一个 0.9 的离群值,中位数应落在中间两个之间
        check_near('fuse_x', fused['red_ball'][0], 0.115, 1e-6)
    m.set_reference('red_ball', (0.2, 0.0, 0.0))
    check('best_uses_reference', m.best('red_ball')[0], 0.2)
    check('labels', m.labels(), ['red_ball'])
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

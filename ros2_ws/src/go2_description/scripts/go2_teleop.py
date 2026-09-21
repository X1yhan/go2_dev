#!/usr/bin/env python3
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist

HELP = """
go2_teleop 键盘遥控 (发布 /cmd_vel,需要步态节点在工作,如 walk:=true)
  w / s   前进 / 后退        a / d   左转 / 右转
  q / e   左转 / 右转(同 a/d) k 或空格 停止
  + / -   加速 / 减速        x 或 Ctrl+C 退出
  注:横向平移(linear.y)对角小跑暂不支持
"""

KEY_AXES = {
    'w': (1.0, 0.0, 0.0), 's': (-1.0, 0.0, 0.0),
    'a': (0.0, 0.0, 1.0), 'd': (0.0, 0.0, -1.0),
    'q': (0.0, 0.0, 1.0), 'e': (0.0, 0.0, -1.0),
}
MAX_SPEED = 0.6
MAX_TURN = 1.0


def main():
    rclpy.init()
    node = rclpy.create_node('go2_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    speed, turn = 0.3, 0.8
    vx = vy = wz = 0.0

    if not sys.stdin.isatty():
        print('go2_teleop 需要交互式终端(直接运行,不要重定向输入)')
        return
    fd = sys.stdin.fileno()
    settings = termios.tcgetattr(fd)
    print(HELP)
    try:
        tty.setcbreak(fd)
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                if key in ('x', '\x03'):
                    break
                if key == '+':
                    speed = min(speed * 1.2, MAX_SPEED)
                    turn = min(turn * 1.2, MAX_TURN)
                elif key == '-':
                    speed = max(speed / 1.2, 0.05)
                    turn = max(turn / 1.2, 0.1)
                else:
                    axes = KEY_AXES.get(key)
                    if axes is not None:
                        vx, vy, wz = (axes[0] * speed, axes[1] * speed,
                                      axes[2] * turn)
                    elif key in ('k', ' '):
                        vx = vy = wz = 0.0
                    else:
                        continue
                print('\r速度 %.2f m/s,转向 %.2f rad/s -> vx=%.2f vy=%.2f wz=%.2f   '
                      % (speed, turn, vx, vy, wz), end='', flush=True)

            msg = Twist()
            msg.linear.x = vx
            msg.linear.y = vy
            msg.angular.z = wz
            pub.publish(msg)
    finally:
        pub.publish(Twist())
        termios.tcsetattr(fd, termios.TCSADRAIN, settings)
        print('\n已停止,退出。')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

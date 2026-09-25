#!/usr/bin/env python3
"""把 RM75 URDF 合并到 Go2 URDF 背上(固定挂载).

- 在 Go2 `base`(躯干)上定义载荷挂点 `load_link`(躯干顶面)
- 机械臂 base_link 通过 `arm_mount_joint` 固定在 load_link 上
- package:// 网格路径重写成 file://(rm_description 在 third_party 里,未安装)

用法:
    build_arm_model.py --go2-urdf ... --arm-urdf ... --out /tmp/go2_rm75.urdf
    build_arm_model.py ...             # 不写 --out 则打印到 stdout
"""
import argparse
import os
import sys
import tempfile
import xml.etree.ElementTree as ET

DEFAULT_PLATE_XYZ = '0 0 0.065'
DEFAULT_CAMERA_XYZ = '0 0.06 -0.02'
DEFAULT_CAMERA_RPY = '0 -1.5708 0'


def _add_fixed(root, joint_name, parent, child, xyz, rpy, add_link=False):
    if add_link:
        ET.SubElement(root, 'link', {'name': child})
    joint = ET.SubElement(root, 'joint',
                          {'name': joint_name, 'type': 'fixed'})
    ET.SubElement(joint, 'parent', {'link': parent})
    ET.SubElement(joint, 'child', {'link': child})
    ET.SubElement(joint, 'origin', {'xyz': xyz, 'rpy': rpy})


def _gripper_children(urdf_path, share, package):
    root = ET.parse(urdf_path).getroot()
    drop_links = {'world', 'gripper_root_link'}
    drop_joints = {'world_fixed', 'ee_fixed_joint'}
    drop_tags = {'transmission', 'gazebo'}
    children = []
    for child in list(root):
        if child.tag in drop_tags:
            continue
        if child.tag == 'link' and child.get('name') in drop_links:
            continue
        if child.tag == 'joint' and child.get('name') in drop_joints:
            continue
        children.append(child)
    return children


def _realsense_children(model, parent, share):
    import xacro
    xacro_file = os.path.join(share, 'urdf', '_%s.urdf.xacro' % model)
    content = (
        '<?xml version="1.0"?>\n'
        '<robot name="camera_attach" '
        'xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <xacro:include filename="%s"/>\n'
        '  <xacro:sensor_%s parent="%s" name="camera" '
        'use_nominal_extrinsics="true" use_mesh="true">\n'
        '    <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        '  </xacro:sensor_%s>\n'
        '</robot>\n' % (xacro_file, model, parent, model))
    with tempfile.NamedTemporaryFile('w', suffix='.urdf.xacro',
                                     delete=False) as handle:
        handle.write(content)
        path = handle.name
    try:
        document = xacro.process_file(path)
    finally:
        os.unlink(path)
    return ET.fromstring(document.toxml())


def _rewrite_meshes(root, package, share):
    prefix = 'package://%s/' % package
    for mesh in root.iter('mesh'):
        filename = mesh.get('filename', '')
        if filename.startswith(prefix):
            mesh.set('filename', 'file://' + share + '/' +
                     filename[len(prefix):])


def build(go2_urdf, arm_urdf, go2_share, arm_share,
          plate_xyz=DEFAULT_PLATE_XYZ, arm_xyz='0 0 0', arm_rpy='0 0 0',
          tool_xyz='0 0 0', tool_rpy='0 0 0',
          camera='none', camera_xyz=DEFAULT_CAMERA_XYZ,
          camera_rpy=DEFAULT_CAMERA_RPY, realsense_share='',
          gripper='none', gripper_xyz='0 0 0',
          gripper_rpy='0 -1.5708 0', gripper_share=''):
    go2 = ET.parse(go2_urdf).getroot()
    arm = ET.parse(arm_urdf).getroot()

    _add_fixed(go2, 'load_joint', 'base', 'load_link', plate_xyz, '0 0 0',
               add_link=True)

    for child in list(arm):
        go2.append(child)

    _add_fixed(go2, 'arm_mount_joint', 'load_link', 'base_link',
               arm_xyz, arm_rpy)

    # flange frame reserved for the gripper
    _add_fixed(go2, 'tool0_joint', 'Link7', 'tool0', tool_xyz, tool_rpy,
               add_link=True)

    # camera on a side bracket, flange stays free
    if camera != 'none':
        if not realsense_share:
            raise RuntimeError('camera=%s needs --realsense-share' % camera)
        _add_fixed(go2, 'camera_mount_joint', 'Link7', 'camera_mount_link',
                   camera_xyz, camera_rpy, add_link=True)
        camera_root = _realsense_children(camera, 'camera_mount_link',
                                          realsense_share)
        for child in list(camera_root):
            go2.append(child)

    # two-finger gripper on the flange
    if gripper != 'none':
        if not gripper_share:
            raise RuntimeError('gripper=%s needs --gripper-share' % gripper)
        package = 'dh_robotics_%s_description' % gripper
        urdf_path = os.path.join(gripper_share, 'urdf',
                                 'dh_robotics_%s.urdf' % gripper)
        _add_fixed(go2, 'tool0_gripper_joint', 'tool0', 'ee_link',
                   gripper_xyz, gripper_rpy)
        for child in _gripper_children(urdf_path, gripper_share, package):
            go2.append(child)

    _rewrite_meshes(go2, 'go2_description', go2_share)
    _rewrite_meshes(go2, 'rm_description', arm_share)
    if camera != 'none':
        _rewrite_meshes(go2, 'realsense2_description', realsense_share)
    if gripper != 'none':
        _rewrite_meshes(go2, 'dh_robotics_%s_description' % gripper,
                        gripper_share)
    return ('<?xml version="1.0" encoding="utf-8"?>\n'
            + ET.tostring(go2, encoding='unicode') + '\n')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--go2-urdf', required=True)
    parser.add_argument('--arm-urdf', required=True)
    parser.add_argument('--go2-share', default='')
    parser.add_argument('--arm-share', default='')
    parser.add_argument('--plate-xyz', default=DEFAULT_PLATE_XYZ)
    parser.add_argument('--arm-xyz', default='0 0 0')
    parser.add_argument('--arm-rpy', default='0 0 0')
    parser.add_argument('--tool-xyz', default='0 0 0')
    parser.add_argument('--tool-rpy', default='0 0 0')
    parser.add_argument('--camera', default='none',
                        help='none | d435i | d435 | d415 ...')
    parser.add_argument('--camera-xyz', default=DEFAULT_CAMERA_XYZ)
    parser.add_argument('--camera-rpy', default=DEFAULT_CAMERA_RPY)
    parser.add_argument('--realsense-share', default='')
    parser.add_argument('--gripper', default='none',
                        help='none | ag95 | ag145')
    parser.add_argument('--gripper-xyz', default='0 0 0')
    parser.add_argument('--gripper-rpy', default='0 -1.5708 0')
    parser.add_argument('--gripper-share', default='')
    parser.add_argument('--out')
    args = parser.parse_args()

    urdf = build(args.go2_urdf, args.arm_urdf, args.go2_share,
                 args.arm_share, args.plate_xyz, args.arm_xyz, args.arm_rpy,
                 args.tool_xyz, args.tool_rpy, args.camera, args.camera_xyz,
                 args.camera_rpy, args.realsense_share, args.gripper,
                 args.gripper_xyz, args.gripper_rpy, args.gripper_share)
    if args.out:
        with open(args.out, 'w') as handle:
            handle.write(urdf)
    else:
        sys.stdout.write(urdf)


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
urdf_tree.py — AMR-MTT URDF → frames.gv + frames.pdf
======================================================
Generates a Graphviz .gv file and PDF from the robot URDF,
matching the output format of `ros2 run tf2_tools view_frames`.

Usage — full tree:
  ros2 run amr_mtt_tf2_tools urdf_tree \\
      --ros-args -p xacro_file:=<abs_path_to_xacro> \\
                 -p xacro_args:="camera_enabled:=true two_d_lidar_enabled:=true"

Usage — filtered tree (comma-separated keywords, case-insensitive substring match):
  ros2 run amr_mtt_tf2_tools urdf_tree \\
      --ros-args -p xacro_file:=<abs_path_to_xacro> \\
                 -p include_only:="base,imu,camera,odom"

  A joint is kept when its parent OR child name contains any keyword.
  Keyword examples:
    base      → base_footprint, base_link, chassis_link
    ur5       → all UR5 arm links
    camera    → kinect_camera, kinect_camera_optical, rear_camera*
    wheel     → all wheel links
    imu       → imu_frame
    lidar     → L_F_Lidar_Link, R_B_Lidar_Link
    gripper   → robotiq_85_* links

Usage — read from running robot_state_publisher (no xacro_file needed):
  ros2 run amr_mtt_tf2_tools urdf_tree

Output files (written to current working directory):
  frames_<timestamp>.gv          (full tree)
  frames_<timestamp>_<tag>.gv    (filtered, tag = keywords joined by '_')
"""

import subprocess
import time
import xml.etree.ElementTree as ET
from datetime import datetime

import rclpy
from rclpy.node import Node


# ── URDF parser ───────────────────────────────────────────────────────────────

def parse_urdf(urdf_xml: str):
    """Return list of joint dicts: name, type, parent, child, xyz, rpy."""
    root = ET.fromstring(urdf_xml)
    joints = []
    for el in root.findall('joint'):
        parent_el = el.find('parent')
        child_el  = el.find('child')
        origin_el = el.find('origin')
        if parent_el is None or child_el is None:
            continue
        xyz = origin_el.attrib.get('xyz', '0 0 0') if origin_el is not None else '0 0 0'
        rpy = origin_el.attrib.get('rpy', '0 0 0') if origin_el is not None else '0 0 0'
        joints.append({
            'name':   el.attrib.get('name', '?'),
            'type':   el.attrib.get('type', '?'),
            'parent': parent_el.attrib.get('link', '?'),
            'child':  child_el.attrib.get('link', '?'),
            'xyz':    xyz,
            'rpy':    rpy,
        })
    return joints


# ── Filter ───────────────────────────────────────────────────────────────────

def filter_joints(joints: list, include_only: str) -> list:
    """Keep joints where parent OR child contains any keyword (case-insensitive).

    include_only: comma-separated keywords, e.g. "base,imu,camera"
    Returns the original list if include_only is empty.
    """
    if not include_only.strip():
        return joints

    keywords = [k.strip().lower() for k in include_only.split(',') if k.strip()]

    def matches(name: str) -> bool:
        n = name.lower()
        return any(kw in n for kw in keywords)

    return [j for j in joints if matches(j['parent']) or matches(j['child'])]


# ── Graphviz generator ────────────────────────────────────────────────────────

def build_gv(joints: list, timestamp: float) -> str:
    """Build a Graphviz DOT string in view_frames format."""
    lines = ['digraph G {']

    for j in joints:
        label = (
            f' Joint: {j["name"]}\\n'
            f'Type: {j["type"]}\\n'
            f'xyz: {j["xyz"]}\\n'
            f'rpy: {j["rpy"]}\\n'
        )
        lines.append(f'"{j["parent"]}" -> "{j["child"]}"[label="{label}"];')

    # Legend subgraph — same style as view_frames
    dt_str = datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')
    # Find the root (first parent that is never a child)
    children = {j['child'] for j in joints}
    roots = [j['parent'] for j in joints if j['parent'] not in children]
    root_node = roots[0] if roots else (joints[0]['parent'] if joints else 'root')

    lines.append('edge [style=invis];')
    lines.append(' subgraph cluster_legend { style=bold; color=black; label ="view_frames Result";')
    lines.append(f'"Recorded at time: {timestamp}"[ shape=plaintext ] ;')
    lines.append(f'}}->"{root_node}";')
    lines.append('}')

    return '\n'.join(lines)


# ── File writer ───────────────────────────────────────────────────────────────

def write_files(gv_content: str, timestamp: float,
                tag: str = '') -> tuple[str, str]:
    """Write .gv and .pdf files, return (gv_path, pdf_path).

    tag: optional suffix added to filename, e.g. 'base_imu_camera'
    """
    dt_str   = datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d_%H.%M.%S')
    suffix   = f'_{tag}' if tag else ''
    gv_path  = f'frames_{dt_str}{suffix}.gv'
    pdf_path = f'frames_{dt_str}{suffix}.pdf'

    with open(gv_path, 'w') as f:
        f.write(gv_content)

    result = subprocess.run(
        ['dot', '-Tpdf', gv_path, '-o', pdf_path],
        capture_output=True, text=True
    )
    if result.returncode != 0:
        raise RuntimeError(f'dot failed: {result.stderr}')

    return gv_path, pdf_path


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class UrdfTreeNode(Node):

    def __init__(self):
        super().__init__('urdf_tree')
        self.declare_parameter('xacro_file', '')
        self.declare_parameter('xacro_args', '')
        self.declare_parameter('include_only', '')

    def run(self):
        xacro_file   = self.get_parameter('xacro_file').get_parameter_value().string_value
        xacro_args   = self.get_parameter('xacro_args').get_parameter_value().string_value
        include_only = self.get_parameter('include_only').get_parameter_value().string_value

        if xacro_file:
            urdf_xml = self._run_xacro(xacro_file, xacro_args)
        else:
            urdf_xml = self._get_from_param()

        if not urdf_xml:
            return

        timestamp  = time.time()
        all_joints = parse_urdf(urdf_xml)
        joints     = filter_joints(all_joints, include_only)

        # Build filename tag from keywords, e.g. "base,imu,camera" → "base_imu_camera"
        tag = '_'.join(k.strip() for k in include_only.split(',') if k.strip()) \
              if include_only.strip() else ''

        if include_only:
            self.get_logger().info(
                f'Filter: "{include_only}" → {len(joints)}/{len(all_joints)} joints kept')

        gv_str = build_gv(joints, timestamp)

        try:
            gv_path, pdf_path = write_files(gv_str, timestamp, tag)
            self.get_logger().info('view_frames Result')
            self.get_logger().info(f'Wrote {gv_path}')
            self.get_logger().info(f'Wrote {pdf_path}')
            print(f'\nview_frames Result')
            print(f'  {len(joints)} joints (filtered from {len(all_joints)} total)')
            print(f'  Wrote: {gv_path}')
            print(f'  Wrote: {pdf_path}')
        except RuntimeError as e:
            self.get_logger().error(str(e))
            self.get_logger().warn(
                '`dot` not found — saved .gv only. '
                'Install with: sudo apt install graphviz')
            dt_str  = datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d_%H.%M.%S')
            suffix  = f'_{tag}' if tag else ''
            gv_path = f'frames_{dt_str}{suffix}.gv'
            with open(gv_path, 'w') as f:
                f.write(gv_str)
            print(f'\n  Wrote: {gv_path}  (PDF skipped — install graphviz)')

    # ── private helpers ──────────────────────────────────────────────────────

    def _run_xacro(self, xacro_file: str, extra_args: str) -> str | None:
        cmd = ['xacro', xacro_file] + extra_args.split()
        self.get_logger().info(f'Running: {" ".join(cmd)}')
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode != 0:
            self.get_logger().error(f'xacro failed:\n{result.stderr}')
            return None
        return result.stdout

    def _get_from_param(self) -> str | None:
        """Read robot_description via the /robot_state_publisher parameter service."""
        from rcl_interfaces.srv import GetParameters

        self.get_logger().info(
            'No xacro_file given — querying robot_description from '
            '/robot_state_publisher ...')

        client = self.create_client(
            GetParameters, '/robot_state_publisher/get_parameters')

        if not client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(
                '/robot_state_publisher is not running. '
                'Start it first, or pass xacro_file:=<path>.')
            return None

        req = GetParameters.Request()
        req.names = ['robot_description']
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is None:
            self.get_logger().error('GetParameters call failed.')
            return None

        value = future.result().values[0]
        if value.type != 4 or not value.string_value:  # 4 = PARAMETER_STRING
            self.get_logger().error(
                'robot_description parameter is empty or wrong type.')
            return None

        self.get_logger().info('Got robot_description from parameter server.')
        return value.string_value


# ── entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = UrdfTreeNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

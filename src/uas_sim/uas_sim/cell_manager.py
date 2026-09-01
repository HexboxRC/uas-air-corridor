"""Resize air cells at runtime via a ROS 2 service.

Gazebo cannot resize a spawned model's geometry, so a resize = remove the
model + respawn it at the same pose with new dimensions. Drive this from
rqt's Service Caller for a GUI, or call it from code later for automatic
(dynamic) resizing.

Place at:  uas_sim/uas_sim/cell_manager.py
"""

import subprocess

import rclpy
from rclpy.node import Node

from uas_interfaces.srv import ResizeCell


WORLD = 'test_world'

# Cell state from test_world.sdf: pose (x y z roll pitch yaw), size, rgba
CELLS = {
    'air_cell_EW_up':   {'pose': '40 2.5 25 0 1.57079 0',
                         'size': [50.0, 5.0, 5.0],  'rgba': '0.8 0 0 0.6'},
    'air_cell_EW':      {'pose': '0 2.5 52.5 3.14159 1.57079 3.14159',
                         'size': [5.0, 5.0, 85.0],  'rgba': '0.8 0 0 0.6'},
    'air_cell_EW_down': {'pose': '-40 2.5 25 3.14159 1.57079 3.14159',
                         'size': [50.0, 5.0, 5.0],  'rgba': '0.8 0 0 0.6'},
    'air_cell_WE_up':   {'pose': '40 -2.5 25 3.14159 1.57079 3.14159',
                         'size': [50.0, 5.0, 5.0],  'rgba': '0 0 0.7 0.6'},
    'air_cell_WE':      {'pose': '0 -2.5 52.5 3.14159 1.57079 3.14159',
                         'size': [5.0, 5.0, 85.0],  'rgba': '0 0 0.7 0.6'},
    'air_cell_WE_down': {'pose': '-40 -2.5 25 0 1.57079 0',
                         'size': [50.0, 5.0, 5.0],  'rgba': '0 0 0.7 0.6'},
}


def cell_sdf(name, size, rgba):
    sx, sy, sz = size
    return (
        f"<?xml version='1.0'?><sdf version='1.9'>"
        f"<model name='{name}'><static>true</static><link name='cell'>"
        f"<visual name='vol'><geometry><box><size>{sx} {sy} {sz}</size></box>"
        f"</geometry><material><ambient>{rgba}</ambient>"
        f"<diffuse>{rgba}</diffuse></material></visual>"
        f"</link></model></sdf>"
    )


def gz_call(service, reqtype, req, timeout=3000):
    cmd = ['gz', 'service', '-s', service,
           '--reqtype', reqtype, '--reptype', 'gz.msgs.Boolean',
           '--timeout', str(timeout), '--req', req]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
    return out.returncode == 0, (out.stdout + out.stderr).strip()


class CellManager(Node):
    def __init__(self):
        super().__init__('cell_manager')
        self.cells = {k: dict(v) for k, v in CELLS.items()}
        self.srv = self.create_service(
            ResizeCell, 'resize_cell', self.on_resize)
        self.get_logger().info(
            f'ready - cells: {", ".join(self.cells)}')

    def on_resize(self, req, resp):
        name = req.cell_name
        if name not in self.cells:
            resp.success = False
            resp.message = f'unknown cell "{name}"'
            self.get_logger().warn(resp.message)
            return resp

        cell = self.cells[name]
        new_size = [req.size_x, req.size_y, req.size_z]

        ok, msg = gz_call(
            f'/world/{WORLD}/remove', 'gz.msgs.Entity',
            f'name: "{name}" type: MODEL')
        if not ok:
            self.get_logger().warn(f'remove failed: {msg}')

        sdf = cell_sdf(name, new_size, cell['rgba']).replace('"', '\\"')
        x, y, z, r, p, yw = cell['pose'].split()
        req_str = (
            f'sdf: "{sdf}" '
            f'pose: {{position: {{x: {x} y: {y} z: {z}}}}} '
            f'name: "{name}" allow_renaming: false'
        )
        ok, msg = gz_call(
            f'/world/{WORLD}/create', 'gz.msgs.EntityFactory', req_str)

        if ok:
            cell['size'] = new_size
            resp.success = True
            resp.message = f'{name} -> {new_size}'
            self.get_logger().info(resp.message)
        else:
            resp.success = False
            resp.message = f'create failed: {msg}'
            self.get_logger().error(resp.message)
        return resp


def main(args=None):
    rclpy.init(args=args)
    node = CellManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

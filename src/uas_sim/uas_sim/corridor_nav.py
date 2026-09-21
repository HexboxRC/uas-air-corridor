"""Namespace-aware PX4 corridor navigation for Gazebo SITL.

Assumes each EKF local origin is at its configured spawn point, with NED
axes aligned with the world ENU axes. EKF origin resets are not handled.
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from px4_msgs.msg import VehicleCommand, VehicleCommandAck
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus

CMD_ARM = 400
CMD_SET_MODE = 176
MODE_OFFBOARD = 6.0
ARMED = getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2)
OFFBOARD = getattr(VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)
ACCEPT_R = 15.0
WARMUP = 20
DATA_TIMEOUT = 3.0


def enu_to_ned(east, north, up):
    return (north, east, -up)


class CorridorNav(Node):
    def __init__(self):
        super().__init__('corridor_nav')
        defaults = {
            'ns': 'px4_0',
            'target_system': 1,
            'route_file': '',
            'start_index': 0,
            'spawn_x': 0.0,
            'spawn_y': 0.0,
            'separation_m': 60.0,
            'peers': [''],
            'peer_spawns_json': '{}',
            'status_topic': 'vehicle_status_v4',
            'position_topic': 'vehicle_local_position_v1',
            'ack_topic': 'vehicle_command_ack',
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        g = lambda key: self.get_parameter(key).value
        self.ns = g('ns').strip('/')
        self.target_system = int(g('target_system'))
        self.start_index = int(g('start_index'))
        self.sx = float(g('spawn_x'))
        self.sy = float(g('spawn_y'))
        self.separation = float(g('separation_m'))
        self.peers = [p.strip('/') for p in g('peers')
                      if p.strip('/') and p.strip('/') != self.ns]
        self.peer_spawn = json.loads(g('peer_spawns_json'))
        missing = [p for p in self.peers if p not in self.peer_spawn]
        if missing:
            raise ValueError(f'Missing peer spawn coordinates: {missing}')

        with open(g('route_file'), encoding='utf-8') as handle:
            data = json.load(handle)
        lane0 = sorted(
            (c for c in data['cells'] if c['lane'] == 0),
            key=lambda c: c['index'])
        if not 0 <= self.start_index < len(lane0):
            raise ValueError('start_index is outside the lane-0 route.')
        self.cells = lane0[self.start_index:]
        self.route = [enu_to_ned(c['x'] - self.sx, c['y'] - self.sy, c['z'])
                      for c in self.cells]
        self.yaws = []
        for i, point in enumerate(self.route):
            if i + 1 < len(self.route):
                nxt = self.route[i + 1]
                yaw = math.atan2(nxt[1] - point[1], nxt[0] - point[0])
            else:
                yaw = self.yaws[-1] if self.yaws else 0.0
            self.yaws.append(yaw)

        self.pos = None
        self.position_valid = False
        self.status_received = False
        self.last_pos = self.last_status = 0.0
        self.armed = self.offboard = False
        self.idx = self.counter = self.stream_count = 0
        self.climbed = False
        self.hold_target = None
        self.holding = False
        self.peer_world = {}
        self.peer_seen = {}

        # Volatile readers can receive live data from volatile or transient
        # local writers. Best effort matches PX4 sensor/status publications.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        pre = f'/{self.ns}' if self.ns else ''
        self.ctrl = self.create_publisher(
            OffboardControlMode, f'{pre}/fmu/in/offboard_control_mode', qos)
        self.sp = self.create_publisher(
            TrajectorySetpoint, f'{pre}/fmu/in/trajectory_setpoint', qos)
        self.cmd = self.create_publisher(
            VehicleCommand, f'{pre}/fmu/in/vehicle_command', qos)
        self.create_subscription(
            VehicleLocalPosition, f"{pre}/fmu/out/{g('position_topic')}",
            self.on_pos, qos)
        self.create_subscription(
            VehicleStatus, f"{pre}/fmu/out/{g('status_topic')}",
            self.on_status, qos)
        self.create_subscription(
            VehicleCommandAck, f"{pre}/fmu/out/{g('ack_topic')}",
            self.on_ack, qos)
        for peer in self.peers:
            self.create_subscription(
                VehicleLocalPosition,
                f"/{peer}/fmu/out/{g('position_topic')}",
                lambda message, key=peer: self.on_peer(key, message), qos)
        self.get_logger().info(
            f'{self.ns}: {len(self.route)} cells; '
            f'configured system_id={self.target_system}; waiting for PX4 status')
        self.create_timer(0.1, self.loop)

    def on_pos(self, m):
        self.pos = (float(m.x), float(m.y), float(m.z))
        self.position_valid = (
            m.xy_valid and m.z_valid and all(math.isfinite(v) for v in self.pos))
        self.last_pos = time.monotonic()

    def on_status(self, m):
        first = not self.status_received
        self.status_received = True
        self.last_status = time.monotonic()
        self.armed = m.arming_state == ARMED
        self.offboard = m.nav_state == OFFBOARD
        self.target_system = int(m.system_id)
        if first:
            self.get_logger().info(
                f'{self.ns}: status received, system_id={m.system_id}, '
                f'armed={self.armed}, offboard={self.offboard}')

    def on_ack(self, m):
        if m.command not in (CMD_ARM, CMD_SET_MODE):
            return
        results = {0: 'ACCEPTED', 1: 'TEMPORARILY_REJECTED', 2: 'DENIED',
                   3: 'UNSUPPORTED', 4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED'}
        self.get_logger().info(
            f'{self.ns}: command={m.command}, '
            f'result={results.get(m.result, str(m.result))}')

    def on_peer(self, key, m):
        if not (m.xy_valid and m.z_valid):
            return
        if not all(math.isfinite(v) for v in (m.x, m.y, m.z)):
            return
        sx, sy = self.peer_spawn[key]
        self.peer_world[key] = (m.y + sx, m.x + sy, -m.z)
        self.peer_seen[key] = time.monotonic()

    def world_enu(self):
        north, east, down = self.pos
        return (east + self.sx, north + self.sy, -down)

    def too_close(self):
        # This is a simple same-lane spacing rule, not collision avoidance.
        now = time.monotonic()
        if any(now - self.peer_seen.get(p, 0.0) > DATA_TIMEOUT
               for p in self.peers):
            return True
        me = self.world_enu()
        yaw = self.yaws[self.idx]
        east, north = math.sin(yaw), math.cos(yaw)
        for peer in self.peers:
            point = self.peer_world[peer]
            dx, dy = point[0] - me[0], point[1] - me[1]
            if (math.hypot(dx, dy) < self.separation
                    and east * dx + north * dy > 0.0):
                return True
        return False

    def loop(self):
        try:
            self.tick()
        except Exception as error:
            self.stream_count = 0
            self.get_logger().error(f'{self.ns} loop: {error!r}')

    def tick(self):
        self.counter += 1
        now = time.monotonic()
        ready = (self.status_received and self.position_valid
                 and now - self.last_status < DATA_TIMEOUT
                 and now - self.last_pos < DATA_TIMEOUT)
        if not ready:
            self.stream_count = 0
            if self.counter % 30 == 0:
                self.get_logger().warning(
                    f'{self.ns}: waiting for fresh status and valid position; '
                    f'status_received={self.status_received}, '
                    f'position_valid={self.position_valid}. '
                    'Check PX4 startup, topic names, and px4_msgs compatibility.')
            # If telemetry disappears in flight, stop the Offboard stream
            # so PX4 can execute its configured Offboard-loss failsafe.
            return

        yaw = self.yaws[self.idx]
        if not self.climbed:
            target = (0.0, 0.0, self.route[0][2])
            if (self.armed and self.offboard
                    and abs(self.pos[2] - target[2]) < 5.0):
                self.climbed = True
                self.get_logger().info(f'{self.ns}: corridor altitude reached')
        else:
            hold = self.too_close()
            if hold != self.holding:
                self.holding = hold
                self.hold_target = self.pos if hold else None
                self.get_logger().info(
                    f'{self.ns}: {"HOLD (spacing or missing peer data)" if hold else "resume"}')
            if hold:
                target = self.hold_target
            else:
                if (self.armed and self.offboard
                        and math.dist(self.pos, self.route[self.idx]) < ACCEPT_R
                        and self.idx < len(self.route) - 1):
                    self.idx += 1
                target = self.route[self.idx]
                yaw = self.yaws[self.idx]

        self.heartbeat()
        self.setpoint(target, yaw)
        self.stream_count += 1
        if self.stream_count >= WARMUP and self.counter % 10 == 0:
            if not self.offboard:
                self.send(CMD_SET_MODE, 1.0, MODE_OFFBOARD)
                self.get_logger().info(f'{self.ns}: requesting OFFBOARD')
            elif not self.armed:
                self.send(CMD_ARM, 1.0)
                self.get_logger().info(f'{self.ns}: requesting ARM')
        if self.counter % 30 == 0:
            self.get_logger().info(
                f'{self.ns}: armed={self.armed}, offboard={self.offboard}, '
                f'cell={self.idx}/{len(self.route) - 1}, hold={self.holding}')

    def heartbeat(self):
        m = OffboardControlMode()
        m.position = True
        m.velocity = m.acceleration = m.attitude = m.body_rate = False
        m.timestamp = self.us()
        self.ctrl.publish(m)

    def setpoint(self, ned, yaw):
        m = TrajectorySetpoint()
        m.position = [float(v) for v in ned]
        m.velocity = [math.nan] * 3
        m.acceleration = [math.nan] * 3
        m.jerk = [math.nan] * 3
        m.yaw = float(yaw)
        m.yawspeed = math.nan
        m.timestamp = self.us()
        self.sp.publish(m)

    def send(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = int(command)
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = self.target_system
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = self.us()
        self.cmd.publish(m)

    def us(self):
        return self.get_clock().now().nanoseconds // 1000


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CorridorNav()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

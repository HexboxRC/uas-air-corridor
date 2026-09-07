"""Namespace-aware corridor navigation for multiple PX4 drones.

Each drone reads the shared route.json, flies its assigned stretch of the
corridor, and holds position if it closes on the drone ahead. The separation
distance is a parameter - session 4 makes it a function of wind.

Place at: uas_sim/uas_sim/corridor_nav.py

Params:
  ns            namespace, e.g. px4_0 (must match PX4_UXRCE_DDS_NS)
  route_file    path to route.json from corridor_gen.py
  start_index   which cell this drone starts at (staggers the fleet)
  spawn_x/y     this drone's spawn point in world ENU (its EKF origin)
  separation_m  minimum gap to the drone ahead
  peers         other namespaces to watch for separation
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)

CMD_ARM = 400
CMD_SET_MODE = 176
MODE_OFFBOARD = 6.0

ARMED = getattr(VehicleStatus, 'ARMING_STATE_ARMED', 2)
OFFBOARD = getattr(VehicleStatus, 'NAVIGATION_STATE_OFFBOARD', 14)

ACCEPT_R = 15.0
WARMUP = 10
CLIMB_ALT = 120.0


def enu_to_ned(e, n, u):
    return (n, e, -u)


class CorridorNav(Node):
    def __init__(self):
        super().__init__('corridor_nav')

        self.declare_parameter('ns', 'px4_0')
        self.declare_parameter('route_file', '')
        self.declare_parameter('start_index', 0)
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('separation_m', 60.0)
        self.declare_parameter('peers', [''])

        g = self.get_parameter
        self.ns = g('ns').value
        self.start_index = int(g('start_index').value)
        self.sx = float(g('spawn_x').value)
        self.sy = float(g('spawn_y').value)
        self.separation = float(g('separation_m').value)
        peers = [p for p in g('peers').value if p and p != self.ns]

        with open(g('route_file').value) as f:
            data = json.load(f)
        lane0 = [c for c in data['cells'] if c['lane'] == 0]
        lane0.sort(key=lambda c: c['index'])
        self.cells = lane0[self.start_index:]

        # route relative to THIS drone's EKF origin (its spawn point)
        self.route = [enu_to_ned(c['x'] - self.sx, c['y'] - self.sy, c['z'])
                      for c in self.cells]
        self.yaws = [math.atan2(*reversed(
            (self.route[min(i+1, len(self.route)-1)][0] - self.route[i][0],
             self.route[min(i+1, len(self.route)-1)][1] - self.route[i][1])))
            if i + 1 < len(self.route) else 0.0
            for i in range(len(self.route))]
        for i in range(len(self.yaws) - 1, 0, -1):
            if self.yaws[i] == 0.0:
                self.yaws[i] = self.yaws[i-1]

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)

        pre = f'/{self.ns}' if self.ns else ''
        self.ctrl = self.create_publisher(
            OffboardControlMode, f'{pre}/fmu/in/offboard_control_mode', qos)
        self.sp = self.create_publisher(
            TrajectorySetpoint, f'{pre}/fmu/in/trajectory_setpoint', qos)
        self.cmd = self.create_publisher(
            VehicleCommand, f'{pre}/fmu/in/vehicle_command', qos)
        self.create_subscription(
            VehicleLocalPosition, f'{pre}/fmu/out/vehicle_local_position_v1',
            self.on_pos, qos)
        self.create_subscription(
            VehicleStatus, f'{pre}/fmu/out/vehicle_status_v4',
            self.on_status, qos)

        # peer positions, for separation
        self.peer_world = {}
        self.peer_spawn = {}
        for p in peers:
            self.create_subscription(
                VehicleLocalPosition, f'/{p}/fmu/out/vehicle_local_position_v1',
                lambda m, k=p: self.on_peer(k, m), qos)

        self.pos = None
        self.idx = 0
        self.armed = self.offboard = False
        self.counter = 0
        self.holding = False

        self.get_logger().info(
            f'{self.ns}: {len(self.route)} cells from index {self.start_index}, '
            f'separation {self.separation:.0f} m, peers={peers}')
        self.create_timer(0.1, self.loop)

    # --- callbacks -------------------------------------------------
    def on_pos(self, m):
        self.pos = (m.x, m.y, m.z)

    def on_status(self, m):
        self.armed = (m.arming_state == ARMED)
        self.offboard = (m.nav_state == OFFBOARD)

    def on_peer(self, key, m):
        # peer local NED -> world ENU (needs peer spawn; approximated by its
        # own origin, so compare in world frame)
        self.peer_world[key] = (m.y, m.x, -m.z)

    # --- helpers ---------------------------------------------------
    def world_enu(self):
        n, e, d = self.pos
        return (e + self.sx, n + self.sy, -d)

    def too_close(self):
        """True if a peer is ahead and within the separation distance."""
        if self.pos is None or not self.peer_world:
            return False
        me = self.world_enu()
        for k, p in self.peer_world.items():
            dx, dy = p[0] - me[0], p[1] - me[1]
            dist = math.hypot(dx, dy)
            if dist < self.separation:
                # only yield to a peer that is AHEAD along the corridor
                tgt = self.cells[min(self.idx, len(self.cells)-1)]
                ax, ay = tgt['x'] - me[0], tgt['y'] - me[1]
                if ax * dx + ay * dy > 0:
                    return True
        return False

    # --- main loop -------------------------------------------------
    def loop(self):
        try:
            self.tick()
        except Exception as e:
            self.get_logger().error(f'{self.ns} loop: {e!r}')

    def tick(self):
        self.heartbeat()

        target = self.route[self.idx]
        # climb to corridor altitude before translating
        if self.counter < 120 and self.pos is not None and -self.pos[2] < CLIMB_ALT * 0.8:
            target = (0.0, 0.0, -CLIMB_ALT)

        self.setpoint(target, self.yaws[self.idx])

        if not (self.armed and self.offboard):
            if self.counter >= WARMUP and self.counter % 10 == 0:
                self.send(CMD_SET_MODE, 1.0, MODE_OFFBOARD)
                self.send(CMD_ARM, 1.0)
                self.get_logger().info(
                    f'{self.ns} arming... armed={self.armed} off={self.offboard}')
        elif self.pos is not None:
            hold = self.too_close()
            if hold != self.holding:
                self.holding = hold
                self.get_logger().info(
                    f'{self.ns} {"HOLD - separation" if hold else "resume"}')

            tx, ty, tz = self.route[self.idx]
            dist = math.dist(self.pos, (tx, ty, tz))
            if self.counter % 30 == 0:
                self.get_logger().info(
                    f'{self.ns} cell {self.idx}/{len(self.route)-1} '
                    f'dist={dist:.0f} hold={hold}')
            if not hold and dist < ACCEPT_R and self.idx < len(self.route) - 1:
                self.idx += 1

        self.counter += 1

    # --- publishing ------------------------------------------------
    def heartbeat(self):
        m = OffboardControlMode()
        m.position = True
        m.velocity = m.acceleration = m.attitude = m.body_rate = False
        m.timestamp = self.us()
        self.ctrl.publish(m)

    def setpoint(self, ned, yaw):
        m = TrajectorySetpoint()
        m.position = [float(ned[0]), float(ned[1]), float(ned[2])]
        m.yaw = float(yaw)
        m.timestamp = self.us()
        self.sp.publish(m)

    def send(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = int(command)
        m.param1, m.param2 = float(p1), float(p2)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = self.us()
        self.cmd.publish(m)

    def us(self):
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    n = CorridorNav()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
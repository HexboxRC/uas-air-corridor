"""Offboard navigation through the test_world air-bridge.

Walks the EW-lane arch route: up the east riser, across the top span,
down the west riser.

Place at:  uas_sim/uas_sim/offboard_nav.py
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)

from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition, VehicleStatus)


# EW-lane arch route (ENU: east, north, up) - matches test_world.sdf.
CELL_CENTERS_ENU = [
    (0.0,    2.5, 10.0),
    (40.0,   2.5, 10.0),
    (40.0,   2.5, 52.0),
    (0.0,    2.5, 52.0),
    (-40.0,  2.5, 52.0),
    (-40.0,  2.5, 10.0),
    (-40.0,  2.5,  7.0),
]

ACCEPT_RADIUS = 5.0
WARMUP_TICKS = 10
CELL_TIMEOUT_TICKS = 400      # 40 s at 10 Hz, then advance anyway


def enu_to_ned(e, n, u):
    return (n, e, -u)


def ned_yaw_towards(cur, nxt):
    return math.atan2(nxt[1] - cur[1], nxt[0] - cur[0])


class OffboardNav(Node):
    def __init__(self):
        super().__init__('offboard_nav')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )

        self.ctrl_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # NOTE: versioned topic names on this PX4 build
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.on_pos, qos)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self.on_status, qos)

        self.route = [enu_to_ned(*c) for c in CELL_CENTERS_ENU]
        self.yaws = []
        for i in range(len(self.route)):
            if i + 1 < len(self.route):
                self.yaws.append(ned_yaw_towards(self.route[i], self.route[i + 1]))
            else:
                self.yaws.append(self.yaws[-1] if self.yaws else 0.0)

        self.idx = 0
        self.pos = None
        self.armed = False
        self.offboard = False
        self.status_seen = False
        self.counter = 0
        self.ticks_on_cell = 0

        self.create_timer(0.1, self.loop)   # 10 Hz

    def on_pos(self, msg):
        self.pos = (msg.x, msg.y, msg.z)

    def on_status(self, msg):
        self.status_seen = True
        self.armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.offboard = (msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD)

    def loop(self):
        try:
            self.tick()
        except Exception as e:
            self.get_logger().error(f'loop error (continuing): {e!r}')

    def tick(self):
        self.publish_heartbeat()
        self.publish_setpoint(self.route[self.idx], self.yaws[self.idx])

        ready = self.armed and self.offboard and self.pos is not None

        if not ready:
            if self.counter >= WARMUP_TICKS and self.counter % 10 == 0:
                self.set_offboard()
                self.arm()
                self.get_logger().info(
                    f'requesting... status_seen={self.status_seen} '
                    f'armed={self.armed} offboard={self.offboard} '
                    f'pos={"yes" if self.pos else "NONE"}')
        else:
            tx, ty, tz = self.route[self.idx]
            dist = ((self.pos[0] - tx) ** 2 +
                    (self.pos[1] - ty) ** 2 +
                    (self.pos[2] - tz) ** 2) ** 0.5

            self.get_logger().info(
                f'cell {self.idx}/{len(self.route)-1} '
                f'target=({tx:.0f},{ty:.0f},{tz:.0f}) '
                f'pos=({self.pos[0]:.1f},{self.pos[1]:.1f},{self.pos[2]:.1f}) '
                f'dist={dist:.1f}', throttle_duration_sec=1.0)

            self.ticks_on_cell += 1

            if self.idx < len(self.route) - 1:
                if dist < ACCEPT_RADIUS:
                    self.idx += 1
                    self.ticks_on_cell = 0
                    self.get_logger().info(f'>>> entering cell {self.idx}')
                elif self.ticks_on_cell > CELL_TIMEOUT_TICKS:
                    self.get_logger().warn(
                        f'timeout on cell {self.idx} (dist={dist:.1f}) - advancing')
                    self.idx += 1
                    self.ticks_on_cell = 0

        self.counter += 1

    def publish_heartbeat(self):
        m = OffboardControlMode()
        m.position = True
        m.velocity = m.acceleration = m.attitude = m.body_rate = False
        m.timestamp = self.now_us()
        self.ctrl_pub.publish(m)

    def publish_setpoint(self, ned, yaw):
        m = TrajectorySetpoint()
        m.position = [float(ned[0]), float(ned[1]), float(ned[2])]
        m.yaw = float(yaw)
        m.timestamp = self.now_us()
        self.sp_pub.publish(m)

    def arm(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, p1=1.0)

    def set_offboard(self):
        self.send_cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, p1=1.0, p2=6.0)

    def send_cmd(self, command, p1=0.0, p2=0.0):
        m = VehicleCommand()
        m.command = command
        m.param1 = p1
        m.param2 = p2
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        m.timestamp = self.now_us()
        self.cmd_pub.publish(m)

    def now_us(self):
        return int(self.get_clock().now().nanoseconds / 1000)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
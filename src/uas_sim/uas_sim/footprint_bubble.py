"""Dynamic footprint bubble around the drone (reservation-footprint method).

Draws a translucent sphere ("footprint") on the drone in Gazebo whose radius
grows when a condition is active, and reports which air cells the footprint
currently claims. The cell grid stays fixed; only the claimed volume changes.

Drawing uses the Gazebo marker service via gz-transport python bindings. If
those bindings are not installed, the node still runs and logs the footprint
(the logic is what matters; the bubble is best-effort visualisation).
Install (Harmonic):  sudo apt install python3-gz-transport13 python3-gz-msgs10

Place at:  uas_sim/uas_sim/footprint_bubble.py
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy,
                       DurabilityPolicy, HistoryPolicy)
from px4_msgs.msg import VehicleLocalPosition

# Gazebo marker drawing is optional; the node runs without it.
try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.marker_pb2 import Marker
    from gz.msgs10.boolean_pb2 import Boolean
    GZ_OK = True
except Exception:
    GZ_OK = False


# Air-cell centers from test_world.sdf (ENU: east, north, up).
AIR_CELLS = {
    'EW_up':   (40.0,  2.5, 25.0),
    'EW':      (0.0,   2.5, 52.5),
    'EW_down': (-40.0, 2.5, 25.0),
    'WE_up':   (40.0, -2.5, 25.0),
    'WE':      (0.0,  -2.5, 52.5),
    'WE_down': (-40.0, -2.5, 25.0),
}
CELL_CLAIM_PAD = 25.0     # crude cell half-size for the overlap test (m)

BUBBLE_NORMAL = 5.0       # footprint radius in normal conditions (m)
BUBBLE_LARGE  = 15.0      # footprint radius when the condition is active (m)
SPAWN_Z = 0.2             # matches PX4_GZ_MODEL_POSE z


def ned_to_enu(x, y, z):
    return (y, x, -z + SPAWN_Z)   # PX4 NED (N,E,D) -> Gazebo ENU (E,N,U)


class FootprintBubble(Node):
    def __init__(self):
        super().__init__('footprint_bubble')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.on_pos, qos)

        self.gz = GzNode() if GZ_OK else None
        if not GZ_OK:
            self.get_logger().warn(
                'gz marker bindings not found - logic only, no bubble drawn')

        self.pos = None
        self.create_timer(0.2, self.tick)    # 5 Hz

    def on_pos(self, msg):
        self.pos = ned_to_enu(msg.x, msg.y, msg.z)

    # --- the condition that drives footprint size ----------------------
    def condition_active(self):
        # EXAMPLE: grow while crossing the high span (up > 40 m).
        # Swap for a weather flag, emergency-vehicle flag, or traffic count.
        return self.pos is not None and self.pos[2] > 40.0

    # --- main loop -----------------------------------------------------
    def tick(self):
        if self.pos is None:
            return
        radius = BUBBLE_LARGE if self.condition_active() else BUBBLE_NORMAL
        self.draw_bubble(self.pos, radius)
        claimed = self.claimed_cells(self.pos, radius)
        self.get_logger().info(
            f'r={radius:.0f}m  claims={claimed}', throttle_duration_sec=1.0)

    def claimed_cells(self, pos, radius):
        e, n, u = pos
        out = []
        for name, (ce, cn, cu) in AIR_CELLS.items():
            d = ((e - ce) ** 2 + (n - cn) ** 2 + (u - cu) ** 2) ** 0.5
            if d < radius + CELL_CLAIM_PAD:
                out.append(name)
        return out

    def draw_bubble(self, pos, radius):
        if not GZ_OK:
            return
        m = Marker()
        m.ns = 'footprint'
        m.id = 1
        m.action = Marker.ADD_MODIFY
        m.type = Marker.SPHERE
        m.pose.position.x = pos[0]
        m.pose.position.y = pos[1]
        m.pose.position.z = pos[2]
        m.scale.x = m.scale.y = m.scale.z = radius * 2.0
        c = m.material.diffuse
        c.r, c.g, c.b, c.a = (0.1, 0.8, 0.2, 0.25)
        self.gz.request('/marker', m, Marker, Boolean, 300)


def main(args=None):
    rclpy.init(args=args)
    node = FootprintBubble()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
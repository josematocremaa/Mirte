import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from yolo_msgs.msg import DetectionArray


# ---------------------------------------------------------------------------
# Object that uniquely identifies each room (raw YOLO class names).
# ---------------------------------------------------------------------------
ROOM_TO_TARGET_LABEL = {
    "LectureRoom":    "potted plant",
    "LivingRoomSSER": "backpack",
}

# States
IDLE = "IDLE"
SEARCHING = "SEARCHING"
APPROACHING = "APPROACHING"
ARRIVED = "ARRIVED"


class ReactiveApproachNode(Node):

    def __init__(self):
        super().__init__("reactive_approach_node")

        # --- tunable parameters -------------------------------------------
        self.declare_parameter("search_angular_speed", 0.4)   # rad/s spin
        self.declare_parameter("max_linear_speed", 0.22)      # m/s cap
        self.declare_parameter("max_angular_speed", 1.0)      # rad/s cap
        self.declare_parameter("k_att_lin", 0.4)              # attraction fwd gain
        self.declare_parameter("k_att_ang", 1.5)              # attraction turn gain
        self.declare_parameter("k_rep", 0.35)                 # repulsion gain
        self.declare_parameter("rep_influence", 0.6)          # m, laser influence radius
        self.declare_parameter("approach_distance", 0.6)      # m, stop distance to object
        self.declare_parameter("max_detection_distance", 4.0) # m
        self.declare_parameter("laser_yaw_offset", 0.0)       # rad, lidar vs base mounting
        self.declare_parameter("lost_timeout", 1.5)           # s before giving up detection

        self.p = lambda n: self.get_parameter(n).value

        # --- state --------------------------------------------------------
        self.state = IDLE
        self.target_room = None
        self.target_label = None
        self.last_target = None          # (angle, distance) in base frame
        self.last_seen_time = None
        self.latest_scan = None

        # --- I/O ----------------------------------------------------------
        self.create_subscription(String, "/navigation/goal_room",
                                 self.goal_room_callback, 10)
        self.create_subscription(DetectionArray, "/yolo/detections",
                                 self.detections_callback, 10)
        self.create_subscription(LaserScan, "/scan",
                                 self.scan_callback, 10)

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_pub = self.create_publisher(String, "/navigation/status", 10)

        self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Reactive approach node ready. "
                               "Send a room to /navigation/goal_room.")

    # ------------------------------------------------------------------ #
    def goal_room_callback(self, msg: String):
        room = msg.data.strip()
        if room not in ROOM_TO_TARGET_LABEL:
            self.get_logger().warn(f"Unknown room '{room}'.")
            return
        self.target_room = room
        self.target_label = ROOM_TO_TARGET_LABEL[room]
        self.last_target = None
        self.last_seen_time = None
        self.state = SEARCHING
        self.publish_status(f"Searching for {room} ('{self.target_label}')")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def detections_callback(self, msg: DetectionArray):
        if self.state not in (SEARCHING, APPROACHING):
            return

        best = None
        best_dist = float("inf")
        for det in msg.detections:
            if det.class_name != self.target_label:
                continue
            x = det.bbox3d.center.position.x   # lateral (camera frame)
            z = det.bbox3d.center.position.z   # forward
            dist = math.hypot(x, z)
            if z <= 0.0 or dist > self.p("max_detection_distance"):
                continue
            if dist < best_dist:
                best_dist = dist
                best = (x, z, dist)

        if best is None:
            return

        x, z, dist = best
        angle = math.atan2(x, z)      # >0 -> object to the right in camera x
        self.last_target = (angle, dist)
        self.last_seen_time = self.get_clock().now()

        if self.state == SEARCHING:
            self.state = APPROACHING
            self.publish_status(f"Object spotted, approaching {self.target_room}")

    # ------------------------------------------------------------------ #
    def control_loop(self):
        if self.state == SEARCHING:
            twist = Twist()
            twist.angular.z = self.p("search_angular_speed")
            self.cmd_vel_pub.publish(twist)
            return

        if self.state != APPROACHING:
            return

        # Lost the object?
        if self.last_seen_time is not None:
            dt = (self.get_clock().now() - self.last_seen_time).nanoseconds / 1e9
            if dt > self.p("lost_timeout"):
                self.state = SEARCHING
                self.publish_status("Lost object, searching again")
                return

        if self.last_target is None:
            return

        angle, dist = self.last_target

        # Arrived?
        if dist <= self.p("approach_distance"):
            self.cmd_vel_pub.publish(Twist())
            self.state = ARRIVED
            self.publish_status(f"Arrived at {self.target_room}")
            self.get_logger().info(f"Reached {self.target_room}.")
            self.state = IDLE
            return

        # --- attraction vector (in base frame) ---------------------------
        # Point toward object: forward component along heading, plus turn.
        att_x = math.cos(angle)
        att_y = math.sin(angle)

        # --- repulsion vector from laser ---------------------------------
        rep_x, rep_y = self.compute_repulsion()

        # --- combine -----------------------------------------------------
        fx = self.p("k_att_lin") * att_x + rep_x
        fy = self.p("k_att_ang") * att_y + rep_y

        twist = Twist()
        twist.linear.x = max(0.0, min(self.p("max_linear_speed"), fx))
        twist.angular.z = max(-self.p("max_angular_speed"),
                              min(self.p("max_angular_speed"), fy))
        self.cmd_vel_pub.publish(twist)

    def compute_repulsion(self):
        """Sum repulsive vectors from every laser ray within influence radius,
        expressed in the base frame (x forward, y left)."""
        scan = self.latest_scan
        if scan is None:
            return 0.0, 0.0

        infl = self.p("rep_influence")
        k = self.p("k_rep")
        offset = self.p("laser_yaw_offset")

        rx = ry = 0.0
        angle = scan.angle_min
        for r in scan.ranges:
            a = angle
            angle += scan.angle_increment
            if not math.isfinite(r) or r <= 0.0 or r > infl:
                continue
            # magnitude grows as obstacle gets closer
            mag = k * (1.0 / r - 1.0 / infl) / (r * r)
            beam = a + offset
            # push away from the obstacle (opposite direction)
            rx -= mag * math.cos(beam)
            ry -= mag * math.sin(beam)
        return rx, ry

    # ------------------------------------------------------------------ #
    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main():
    rclpy.init()
    node = ReactiveApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

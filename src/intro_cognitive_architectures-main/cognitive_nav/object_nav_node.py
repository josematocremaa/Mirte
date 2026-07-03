import math

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from yolo_msgs.msg import DetectionArray

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped for tf_buffer.transform)


# Characteristic objects (YOLO class_names) that identify each goal room.
# Any one of the listed objects is enough to localize the room.
# These strings must match the labels published in /yolo/detections_3d
# (same vocabulary as LABEL_TO_ONTOLOGY in symbolic_node.py).
GOAL_TARGET_OBJECT = {
    "LectureRoom": ["tv", "backpack"],
    "LivingRoom":  ["sofa", "potted plant"],
}


class ObjectNavNode(Node):
    """
    Object-goal navigation.

    Given a goal *room*, look up its characteristic object, detect that object
    in the 3D YOLO detections, transform its pose into the global (map) frame,
    compute a safe approach pose in front of it, and send it to Nav2.

    States:
        SEARCHING  -> rotate in place looking for the characteristic object
        NAVIGATING -> a Nav2 goal is active, wait for the result
        DONE       -> goal reached
    """

    def __init__(self):
        super().__init__("object_nav_node")

        # --- parameters ---
        self.declare_parameter("goal_room", "LectureRoom")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("approach_distance", 0.7)   # meters in front of object
        self.declare_parameter("search_angular_speed", 0.4)  # rad/s while searching
        self.declare_parameter("enable_search_motion", True)

        self.goal_room = self.get_parameter("goal_room").value
        self.global_frame = self.get_parameter("global_frame").value
        self.robot_frame = self.get_parameter("robot_frame").value
        self.approach_distance = self.get_parameter("approach_distance").value
        self.search_speed = self.get_parameter("search_angular_speed").value
        self.enable_search = self.get_parameter("enable_search_motion").value

        self.state = "SEARCHING"

        # --- tf ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- I/O ---
        self.create_subscription(
            DetectionArray, "/yolo/detections_3d", self.detections_cb, 10)
        # Optional: let the goal manager retarget us at runtime.
        self.create_subscription(
            String, "/executive/current_goal", self.goal_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # Search-motion loop (decoupled from the perception callback).
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f"Object-nav started. Goal room: '{self.goal_room}' "
            f"-> looking for '{self.target_object()}'."
        )

    # ------------------------------------------------------------------ #
    # Goal handling
    # ------------------------------------------------------------------ #
    def target_object(self):
        return GOAL_TARGET_OBJECT.get(self.goal_room, [])

    def goal_cb(self, msg: String):
        room = msg.data.strip()
        if room == self.goal_room:
            return
        if room not in GOAL_TARGET_OBJECT:
            self.get_logger().warn(f"No characteristic object known for '{room}'.")
            return
        self.goal_room = room
        self.state = "SEARCHING"
        self.get_logger().info(
            f"New goal room '{room}' -> looking for '{self.target_object()}'."
        )

    # ------------------------------------------------------------------ #
    # Perception -> Nav2 goal
    # ------------------------------------------------------------------ #
    def detections_cb(self, msg: DetectionArray):
        if self.state != "SEARCHING":
            return

        targets = self.target_object()
        if not targets:
            return

        for det in msg.detections:
            if det.class_name not in targets:
                continue

            object_map = self.to_global_frame(det, msg.header)
            if object_map is None:
                return  # tf not ready yet; try again on the next detection

            goal_pose = self.compute_approach_pose(object_map)
            if goal_pose is None:
                return

            self.get_logger().info(
                f"Found '{det.class_name}' (id={det.id}). Sending Nav2 goal at "
                f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})."
            )
            self.send_nav_goal(goal_pose)
            self.state = "NAVIGATING"
            self.stop_robot()
            return

    def to_global_frame(self, det, header):
        """Transform the detected object position into the global frame."""
        source = PoseStamped()
        source.header.frame_id = header.frame_id
        # stamp = 0 -> use the latest available transform (robust to sim-time skew)
        source.header.stamp = rclpy.time.Time().to_msg()
        source.pose.position = det.bbox3d.center.position
        source.pose.orientation.w = 1.0
        try:
            return self.tf_buffer.transform(
                source, self.global_frame, timeout=Duration(seconds=1.0))
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"TF {header.frame_id} -> {self.global_frame} failed: {exc}")
            return None

    def compute_approach_pose(self, object_pose: PoseStamped):
        """
        Pose `approach_distance` meters in front of the object (on the robot
        side) facing the object, so Nav2 does not try to drive into it.
        """
        robot = self.robot_position()
        if robot is None:
            return None

        ox, oy = object_pose.pose.position.x, object_pose.pose.position.y
        rx, ry = robot

        dx, dy = rx - ox, ry - oy
        dist = math.hypot(dx, dy)
        if dist < 1e-3:
            ux, uy = -1.0, 0.0
        else:
            ux, uy = dx / dist, dy / dist

        ax = ox + ux * self.approach_distance
        ay = oy + uy * self.approach_distance
        yaw = math.atan2(oy - ay, ox - ax)  # face the object

        goal = PoseStamped()
        goal.header.frame_id = self.global_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = ax
        goal.pose.position.y = ay
        goal.pose.position.z = 0.0
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def robot_position(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_frame,
                rclpy.time.Time(), timeout=Duration(seconds=1.0))
            return (tf.transform.translation.x, tf.transform.translation.y)
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(f"Cannot get robot pose: {exc}")
            return None

    # ------------------------------------------------------------------ #
    # Nav2 action client
    # ------------------------------------------------------------------ #
    def send_nav_goal(self, pose: PoseStamped):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 'navigate_to_pose' action server not available.")
            self.state = "SEARCHING"
            return
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_goal_response)

    def on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Nav2 rejected the goal. Resuming search.")
            self.state = "SEARCHING"
            return
        handle.get_result_async().add_done_callback(self.on_nav_result)

    def on_nav_result(self, future):
        status = future.result().status
        # 4 == STATUS_SUCCEEDED (action_msgs/GoalStatus)
        if status == 4:
            self.get_logger().info(f"Reached '{self.goal_room}'. Goal complete.")
            self.state = "DONE"
            self.stop_robot()
        else:
            self.get_logger().warn(f"Nav2 failed (status={status}). Resuming search.")
            self.state = "SEARCHING"

    # ------------------------------------------------------------------ #
    # Search motion
    # ------------------------------------------------------------------ #
    def control_loop(self):
        if self.state == "SEARCHING" and self.enable_search:
            twist = Twist()
            twist.angular.z = self.search_speed  # rotate in place to scan the room
            self.cmd_pub.publish(twist)

    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = ObjectNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

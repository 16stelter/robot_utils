#!/usr/bin/env python3

"""
Written by dhaval-lad under Apache License, Version 2.0, January 2004. See http://www.apache.org/licenses/ for more details. Modified.
"""

import rclpy, math, time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateThroughPoses
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener

def yaw_from_quaternion(x, y, z, w):
    t1 = +2.0 * (w * z + x * y)
    t2 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t1, t2)
    return yaw_z

def pure_pursuit(current_x, current_y, current_heading, path, index, rot_limit=0.0, speed=0.5):
    done = False
    target_point = path[index]

    target_heading = math.atan2(target_point[1] - current_y,
                                target_point[0] - current_x)

    desired_steering_angle = target_heading - current_heading
    desired_steering_angle = (desired_steering_angle + math.pi) % (2 * math.pi) - math.pi


    final_dist = math.hypot(current_x - target_point[0],
                            current_y - target_point[1])

    if abs(desired_steering_angle) > math.pi / 2 or (rot_limit > 0.0 and abs(desired_steering_angle) > rot_limit):
        v = 0.0
    else:
        speed_scale = math.cos(abs(desired_steering_angle))
        v = speed * max(speed_scale, 0.0)

    if rot_limit > 0.0:
        if desired_steering_angle > rot_limit:
            desired_steering_angle = rot_limit
        elif desired_steering_angle < -rot_limit:
            desired_steering_angle = -rot_limit

    if final_dist < 0.2:
        v = 0.0
        index += 1
        if index >= len(path):
            done = True

    return v, desired_steering_angle * 2, index, done


# ROS2 Node for Pure Pursuit algorithm
class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.ns = self.get_namespace()
        if self.ns == "/":
            self.ns = ""

        self.rotation_speed_limit = self.declare_parameter("rotation_speed_limit", 0.5).value
        self.speed = self.declare_parameter("speed", 0.5).value

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        cmd_vel_topic = self.declare_parameter("cmd_vel_topic", "/cmd_vel").value
        action_topic = self.declare_parameter("action_topic", "/navigate_through_poses").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Twist, self.ns + cmd_vel_topic, 10)
        self.action_server = ActionServer(
            self,
            NavigateThroughPoses,
            self.ns + action_topic,
            self.path_action_callback,
            callback_group=ReentrantCallbackGroup()
        )

        # State variables
        self.path = None  # Holds the planned path
        self.index = 0  # Index of the current closest point on the path
        self.current_x = 0.0  # Robot's current x-coordinate
        self.current_y = 0.0  # Robot's current y-coordinate
        self.current_yaw = 0.0  # Robot's current orientation (yaw)

    def path_action_callback(self, goal_handle):
        self.path = [(pose.pose.position.x, pose.pose.position.y) for pose in goal_handle.request.poses]
        self.index = 0
        result = NavigateThroughPoses.Result()
        twist = Twist()

        while self.path:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.path = None
                self.get_logger().warn("Path goal canceled.")
                return result

            try:
                transform = self.tf_buffer.lookup_transform(self.odom_frame, self.base_frame, rclpy.time.Time())
                self.current_x = transform.transform.translation.x
                self.current_y = transform.transform.translation.y
                self.current_yaw = yaw_from_quaternion(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
            except Exception as e:
                self.get_logger().warn(f"Could not get transform from {self.odom_frame} to {self.base_frame}: {e}")
                time.sleep(0.05)
                continue

            # Calculate linear and angular velocities based on the pure pursuit algorithm
            twist.linear.x, twist.angular.z, self.index, done = pure_pursuit(
                self.current_x, self.current_y, self.current_yaw, self.path, self.index, self.rotation_speed_limit, self.speed
            )

            # Check if the robot has reached the goal
            if done:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.publisher.publish(twist)
                goal_handle.succeed(result)
                self.get_logger().info("Path goal succeeded.")
                return result

            # Publish the computed control command
            self.publisher.publish(twist)
            time.sleep(0.05)  # Control loop rate

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
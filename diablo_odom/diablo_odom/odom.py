# /user/bin/python3
# coding = utf-8

import math
import time
import rclpy
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from motion_msgs.msg import LegMotors
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose, Twist, Vector3

class Odom(Node):
    def __init__(self, name):
        super().__init__(name)
        self.sub_odom = self.create_subscription(\
            LegMotors, "diablo/sensor/Motors", self.odom_callback, 10)
        self.sub_imu = self.create_subscription(\
            Imu, "diablo/sensor/Imu", self.imu_callback, 10)
        self.pub = self.create_publisher(\
            Odometry, "odom", 10)
        self.timer = self.create_timer(0.05, self.odom_pub)

        self.odom_frame_id = 'odom'
        self.child_frame_id = 'base_link'
        self.odom_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.track_width = 0.49
        self.time_t0 = time.time()

    def imu_callback(self, imu_msg):
        self.ang_vel = imu_msg.angular_velocity.z

    def odom_callback(self, motor_msg):
        left_vel = motor_msg.left_wheel_vel
        right_vel = motor_msg.right_wheel_vel
        self.lin_vel = (left_vel + right_vel) / 2
        # self.ang_vel = (left_vel - right_vel) / self.track_width

    def odom_pub(self):
        self.time_t1 = time.time()
        dt = self.time_t1 - self.time_t0
        self.time_t0 = self.time_t1

        self.theta = self.ang_vel * dt
        self.x += self.lin_vel * math.cos(self.theta) * dt * 0.01
        self.y += self.lin_vel * math.sin(self.theta) * dt * 0.01

        odom_trans = TransformStamped()
        odom_trans.header.stamp = self.get_clock().now().to_msg()
        odom_trans.header.frame_id = self.odom_frame_id
        odom_trans.child_frame_id = self.child_frame_id
        odom_trans.transform.translation.x = self.x
        odom_trans.transform.translation.y = self.y
        odom_trans.transform.rotation.w = math.cos(self.theta / 2)
        odom_trans.transform.rotation.z = math.sin(self.theta / 2)
        self.odom_broadcaster.sendTransform(odom_trans)

        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2)
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom_msg.twist.twist.linear.x = self.lin_vel
        odom_msg.twist.twist.angular.z = self.ang_vel

        self.pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Odom("odom_node")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

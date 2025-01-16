#!/usr/bin/env python3

import rospy
import threading
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from controller import Robot


class TurtleController:
    def __init__(self):
        self.robot = Robot()
        self.time_step = self.robot.getBasicTimeStep()
        self.cmd_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_callback)
        self.left_motor = self.robot.getDevice("left wheel motor")
        self.right_motor = self.robot.getDevice("right wheel motor")

        self.laser_scan = self.robot.getDevice("LDS-01")
        self.laser_scan.enable(int(self.time_step))
        self.imu_acc = self.robot.getDevice("accelerometer")
        self.imu_acc.enable(int(self.time_step))
        self.imu_gyro = self.robot.getDevice("gyro")
        self.imu_gyro.enable(int(self.time_step))

        self.scan_pub = rospy.Publisher("/laser/scan", LaserScan, queue_size=10)

        self.scan_publish_timer = rospy.Timer(rospy.Duration(0.1), self.scan_publish)

        self.imu_pub = rospy.Publisher("/laser/imu", Imu, queue_size=10)
        self.imu_publish_timer = rospy.Timer(rospy.Duration(0.025), self.imu_publish)

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        self.scan_message_lock = threading.Lock()
        self.scan_array = []
        self.scan_time = rospy.Time.now()
        self.min_range = self.laser_scan.getMinRange()
        self.max_range = self.laser_scan.getMaxRange()
        self.fov = self.laser_scan.getFov()

        self.wheel_radius = 0.033
        self.wheel_track = 0.178
        self.wheel_rot_vel = 6.669
        self.max_speed = self.wheel_radius * self.wheel_rot_vel

        self.imu_message_lock = threading.Lock()
        self.imu_acc_data = [0.0, 0.0, 0.0]
        self.imu_gyro_data = [0.0, 0.0, 0.0]
        self.imu_time = rospy.Time.now()

        self.cmd_message_lock = threading.Lock()
        self.cmd_vel = [0.0, 0.0]
        self.cmd_time = rospy.Time.now()

    def scan_publish(self, _):
        self.scan_message_lock.acquire_lock()
        current_message = self.scan_array.copy()
        current_time = self.scan_time
        self.scan_message_lock.release_lock()
        scan_msg = LaserScan()
        scan_msg.header.stamp = current_time
        scan_msg.header.frame_id = "laser"
        scan_msg.angle_min = self.fov / 2.0
        scan_msg.angle_max = -self.fov / 2.0
        scan_msg.scan_time = 100.0
        scan_msg.angle_increment = -self.fov / float(len(current_message))
        scan_msg.range_min = self.min_range
        scan_msg.range_max = self.max_range
        scan_msg.ranges = current_message
        scan_msg.intensities = [1.0] * len(current_message)
        self.scan_pub.publish(scan_msg)

    def imu_publish(self, _):
        imu_msg = Imu()
        imu_msg.header.stamp = self.imu_time
        imu_msg.header.frame_id = "imu"
        imu_msg.linear_acceleration.x = self.imu_acc_data[0]
        imu_msg.linear_acceleration.y = self.imu_acc_data[1]
        imu_msg.linear_acceleration.z = self.imu_acc_data[2]
        imu_msg.angular_velocity.x = self.imu_gyro_data[0]
        imu_msg.angular_velocity.y = self.imu_gyro_data[1]
        imu_msg.angular_velocity.z = self.imu_gyro_data[2]
        self.imu_pub.publish(imu_msg)

    def cmd_to_wheel_vel(self, linear_vel, angular_vel):
        if angular_vel == 0.0:
            factor = (
                1.0
                if abs(linear_vel) <= self.max_speed
                else self.max_speed / abs(linear_vel)
            )
            left_wheel_vel = linear_vel / self.wheel_radius * factor
            right_wheel_vel = linear_vel / self.wheel_radius * factor
        else:
            turning_radius = linear_vel / angular_vel
            max_side_speed = (abs(turning_radius) + self.wheel_track / 2.0) * abs(
                angular_vel
            )

            if max_side_speed > self.max_speed:
                if linear_vel != 0:
                    linear_vel_abs = (
                        self.max_speed / abs(angular_vel) - self.wheel_track / 2.0
                    ) * abs(angular_vel)
                    linear_vel = linear_vel_abs if linear_vel > 0.0 else -linear_vel_abs
                    turning_radius = linear_vel / angular_vel
                else:
                    angular_vel_abs = self.max_speed / self.wheel_track * 2.0
                    angular_vel = (
                        angular_vel_abs if angular_vel > 0.0 else -angular_vel_abs
                    )

            left_wheel_vel = (
                (turning_radius - self.wheel_track / 2.0)
                * angular_vel
                / self.wheel_radius
            )
            right_wheel_vel = (
                (turning_radius + self.wheel_track / 2.0)
                * angular_vel
                / self.wheel_radius
            )
        return left_wheel_vel, right_wheel_vel

    def run(self):
        while self.robot.step(int(self.time_step)) != -1 and not rospy.is_shutdown():
            # 发布SCAN 消息
            self.scan_message_lock.acquire_lock()
            self.scan_array = self.laser_scan.getLayerRangeImage(0)
            self.scan_time = rospy.Time.now()
            self.scan_message_lock.release_lock()

            # print(self.laser_scan.getFrequency())
            # 发布imu的消息
            self.imu_message_lock.acquire_lock()
            self.imu_acc_data = self.imu_acc.getValues()
            self.imu_gyro_data = self.imu_gyro.getValues()
            self.imu_time = rospy.Time.now()
            self.imu_message_lock.release_lock()

            self.cmd_message_lock.acquire_lock()
            last_cmd_time = self.cmd_time
            current_cmd = self.cmd_vel.copy()
            self.cmd_message_lock.release_lock()

            if rospy.Time.now() - last_cmd_time > rospy.Duration(0.5):
                self.left_motor.setVelocity(0.0)
                self.right_motor.setVelocity(0.0)
            else:
                left_wheel_vel, right_wheel_vel = self.cmd_to_wheel_vel(
                    current_cmd[0], current_cmd[1]
                )
                self.left_motor.setVelocity(left_wheel_vel)
                self.right_motor.setVelocity(right_wheel_vel)

    def cmd_callback(self, msg):
        self.cmd_message_lock.acquire_lock()
        self.cmd_vel = [msg.linear.x, msg.angular.z]
        self.cmd_time = rospy.Time.now()
        self.cmd_message_lock.release_lock()


if __name__ == "__main__":
    rospy.init_node("diff_controller", anonymous=True)
    turtle_controller = TurtleController()
    turtle_controller.run()

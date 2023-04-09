import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from std_msgs.msg import String

import math
import numpy as np
from typing import Tuple
from dataclasses import dataclass

class Safety(Node):
    def __init__(self) -> None:
        super().__init__('aeb')
        # TODO: get topics from param file
        self.safety_pub = self.create_publisher(AckermannDriveStamped, "/safety", 1)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, "/ego_racecar/odom", self.odom_callback, 1)

        self.brake_active = False
        self.ttc_threshold = 0.5
        self.velocity = np.array([0,0])

    def odom_callback(self, odom_data: Odometry):
        self.odom_data = odom_data
        self.velocity = np.array([odom_data.twist.twist.linear.x, odom_data.twist.twist.linear.y])

    def scan_callback(self, scan_data: LaserScan):
        self.scan_data = scan_data
        self.clean_scan()
        self.nearest = self.nearest_object()
        ttc = self.ttc()
        if (ttc < self.ttc_threshold):
            self.brake_active = True
            print(f"AEB activated with velocity: ({self.velocity[0]}, {self.velocity[1]}) m/s")
        self.publish_drive()

    def nearest_object(self) -> np.ndarray:
        angle = 0.0
        distance = 0.0
        min_idx = int(self.ranges.argmin()) # get index of min range

        angle = float(self.scan_data.angle_min) + float(min_idx * self.scan_data.angle_increment) # get angle of nearest object
        distance = self.ranges[min_idx]
        return np.array([distance, angle])
 
    def ttc(self) -> float:
        r_dot = Safety.proj(self.velocity, self.nearest)
        return self.nearest[0] / max(r_dot, 0.000001)

    # projection of x onto y
    @staticmethod
    def proj(x: np.ndarray, y:np.ndarray) -> float:
        return x.dot(y) / np.linalg.norm(y)
    def clean_scan(self) -> None:
        self.ranges = np.array(self.scan_data.ranges.tolist())
        r_max = self.scan_data.range_max
        r_min = self.scan_data.range_min
        for i in range(len(self.ranges)):
            if math.isnan(self.ranges[i]):
                self.ranges[i] = 0
            elif math.isinf(self.ranges[i]):
                self.ranges[i] = r_max
            else:
                self.ranges[i] = min(self.ranges[i], r_max)
                self.ranges[i] = max(self.ranges[i], r_min)


    def publish_drive(self):
        if self.brake_active:
            ack_msg = AckermannDriveStamped()
            ack_msg.drive.speed = 0.0
            self.safety_pub.publish(ack_msg)


def main(args = None):
    rclpy.init(args=args)

    safety = Safety()
    print("AEB Initialized")

    rclpy.spin(safety)

    safety.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


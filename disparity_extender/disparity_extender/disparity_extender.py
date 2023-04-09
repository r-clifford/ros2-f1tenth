import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

from std_msgs.msg import String

import math
import numpy as np
from typing import Tuple
from dataclasses import dataclass

@dataclass
class Disparity:
    # members are np arrays, magnitude, angle (rad)
    near: np.ndarray
    far: np.ndarray

class DisparityExtender(Node):
    def __init__(self) -> None:
        super().__init__('disparity_extender')
        # TODO: get topics from param file
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 1000)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)

        self.angle_range = 75 * math.pi / 180
        self.disparity_threshold = 0.15 # difference in distance to be considered disparity
        self.car_width = 0.50
        self.max_angle = 0.4189


    def scan_callback(self, scan_data: LaserScan):
        self.scan_data = scan_data
        self.clean_scan()
        self.min_idx = int((-self.angle_range - scan_data.angle_min) / scan_data.angle_increment)
        self.max_idx = int((self.angle_range- scan_data.angle_min) / scan_data.angle_increment)
        self.find_disparities()

        self.target_angle = self.find_target_angle()
        self.publish_drive()

    def find_disparities(self) -> None:
        self.disparities = []
        i = self.min_idx
        while (i < self.max_idx):
            near = self.ranges[i]
            near_angle = float(self.scan_data.angle_min + i * self.scan_data.angle_increment)
            far = self.ranges[i+1]
            far_angle = near_angle + float(self.scan_data.angle_increment)
            # if disparity is large enough, adjust adjacent ranges and add to list of disparities
            if (abs(near - far) > self.disparity_threshold):
                if near > far: # set near and far points correctly
                    tmp = near
                    near = far
                    far = tmp

                    tmp = near_angle
                    near_angle = far_angle
                    far_angle = tmp
                disparity = Disparity(
                    np.array([near, near_angle]),
                    np.array([far, far_angle])
                )
                self.disparities.append(disparity)
                width = self.projected_width(near)
                # set all values within a radius of projected car width to min(distance, near)
                for j in range(2 * width):
                    tmp_idx = min(self.max_idx, max(0,i - width + j))
                    self.ranges[tmp_idx] = min(self.ranges[tmp_idx], near)
                i+=width+1
            else:
                i+=1

    def find_target_angle(self) -> float:
        i = self.min_idx
        current_max = 0
        idx_max = 0
        while (i < self.max_idx):
            if (self.ranges[i] > current_max):
                idx_max = i
                current_max = self.ranges[i]
            i+=1
        return self.scan_data.angle_min + idx_max * self.scan_data.angle_increment


            
    def projected_width(self, distance: float) -> int:
        # approximate the number of indices corresponding to the car width at some distance
        safety_offset = 0.1
        theta = (self.car_width + safety_offset) / distance 
        width = theta / self.scan_data.angle_increment
        return int(width)


    def clean_scan(self) -> None:
        self.ranges = np.asarray(self.scan_data.ranges.tolist())
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

    # TODO
    def calculate_speed(self):
        return 6.0
    def publish_drive(self):
        ack_msg = AckermannDriveStamped()
        ack_msg.drive.speed = self.calculate_speed()
        steering_angle = min(abs(self.target_angle), self.max_angle)
        if self.target_angle < 0:
            steering_angle *= -1
        ack_msg.drive.steering_angle = steering_angle 
        self.drive_pub.publish(ack_msg)


def main(args = None):
    rclpy.init(args=args)

    disparity_extender = DisparityExtender()
    print("Disparity Extender Initialized")

    rclpy.spin(disparity_extender)

    disparity_extender.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



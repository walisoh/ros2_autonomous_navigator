import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from geometry_msgs.msg import Point, Twist
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge


class Getobjrange(Node):
    def __init__(self):
        super().__init__('find_angrang')

        qos_profile = QoSProfile(depth = 10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE

        self.pt = Point()

        self.angletoo = 0.0

        self._coord_subscriber = self.create_subscription(Point, '/coord', self._point_callback, 1)
        self._coord_subscriber

        self._lidar_subscriber = self.create_subscription(LaserScan, '/scan', self._laser_callback, qos_profile)
        self._lidar_subscriber

        self._angrang_publisher = self.create_publisher(Point, '/AngleRange', 1)
        self._angrang_publisher

        self.declare_parameter('x_center', 160)
        self.x_center = self.get_parameter('x_center').get_parameter_value().integer_value

        self.su=False

    def _point_callback(self, Point):
        
        msg = Point
        msg_x = Point.x

        if msg_x != 0.0:

            fov = 67.2
            x_res = 320

            e_p = (self.x_center - msg_x)
            self.angle = e_p * (fov/x_res)

            self.pt.x = 0.0
            self.pt.x = self.angle * (np.pi/180)

            #self._angrang_publisher.publish(self.pt)

            self.su = True

            self.angletoo = self.angle 

        else:
            self.pt.x = 0.0000
            #self._angrang_publisher.publish(self.pt)

            self.su = True

            self.angletoo = 0.0

    def _laser_callback(self, LaserScan):
        print('hello')

        self.ls = LaserScan

        if self.su:

            if self.angletoo != 0.0:

                self.distance = self.ls.ranges[math.floor(self.angletoo)]

                self.pt.y = 0.0

                self.pt.y = float(self.distance)
                self._angrang_publisher.publish(self.pt)

            else: 

                self.distance = 0.0000

                self.pt.y = float(self.distance)
                self._angrang_publisher.publish(self.pt)

            #self.su = False

def main():
    rclpy.init()
    obj = Getobjrange()


    while rclpy.ok():
        rclpy.spin_once(obj)

    obj.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
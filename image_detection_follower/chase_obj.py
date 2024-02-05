import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point, Twist
import sys

import numpy as np
import cv2
from cv_bridge import CvBridge


class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_object')

        # rotation speed
        self.declare_parameter('rotate_speed', 0.3)
        self.rotate_speed = self.get_parameter('rotate_speed').get_parameter_value().double_value

        # x coord that I saw was the center point of the ball
        self.declare_parameter('x_center', 160)
        self.x_center = self.get_parameter('x_center').get_parameter_value().integer_value

        self.declare_parameter('kp_v', 1)
        self.kp_v = self.get_parameter('kp_v').get_parameter_value().integer_value

        self.declare_parameter('kp_w', 5)
        self.kp_w = self.get_parameter('kp_w').get_parameter_value().integer_value

        self._coord_subscriber = self.create_subscription(Point, '/AngleRange', self._rotation, 1)
        self._coord_subscriber

        self._vel_publisher = self.create_publisher(Twist, '/cmd_vel', 5)
        self._vel_publisher

    def _rotation(self, Point):
        pt_x = Point.x
        pt_y = Point.y
        b_v = 0.4
        b_w = 0.1 #0.07

        w = self.kp_w*(pt_x)
        v = self.kp_v*(pt_y - b_v)
        
        ang_msg = Twist()

        if pt_y != 0.0:
            ang_msg.linear.x = v
        else:
            ang_msg.linear.x = 0.0

        ang_msg.linear.y = 0.0
        ang_msg.linear.z = 0.0
        ang_msg.angular.x = 0.0
        ang_msg.angular.y = 0.0


        if pt_x == 0.0:
            ang_msg.angular.z = 0.0

        elif pt_x < -b_w:
            ang_msg.angular.z = w * 1.0

        elif pt_x > b_w:
            ang_msg.angular.z = w * 1.0


        self._vel_publisher.publish(ang_msg)


def main():
    rclpy.init()
    obj = RotateRobot()


    while rclpy.ok():
        rclpy.spin_once(obj)

#    rclpy.spin(obj)

    obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import sys
import time
import numpy as np
import cv2
from cv_bridge import CvBridge


class GoToGoal(Node):
    def __init__(self):
        super().__init__('gotogoal')

        self.Init = True
        self.vel = Twist()
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.angular.z = 0.0

        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos_profile.durability = QoSDurabilityPolicy.VOLATILE
        self.su = False

        self.kpx = 0.18
        self.kpz = 1.4

        self.Init_pos = Point()
        self.globalPos = Point()

        self.cx = 0.0
        self.cy = 0.0
        self.cz = 0.0

        self.gx = 0.0
        self.gy = 0.0
        self.gz = 0.0

        self.pos = np.array([[1.65,0],[1.65,1.55],[0,1.55]])
        self.ind = 0

        self.goal = [1.65, 0]

        self._odom_subscriber = self.create_subscription(Odometry, '/odom', self._motion_controller, 1)
        self._odom_subscriber

        self._vector_subscriber = self.create_subscription(LaserScan, '/scan', self._laser_callback, qos_profile)
        self._vector_subscriber

        self._velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self._velocity_publisher

    def _laser_callback(self, LaserScan):

        self.ls = LaserScan

        b1 = 0.45
        b2 = 0.05
        b3 = 0.35
        b4 = 0.9
        b5 = np.deg2rad(40)

        if self.su:

            dr = self.ls.ranges[200:223]
            minx = min(dr)
            print('minx=', minx)
            minz = np.argmin(dr)
            for i in range(1, len(dr)):
                if dr[i] > minx + b3:
                    if np.argmin(dr[i]) > minz:
                        self.gz = self.cz - b5
                        self.gx = self.cx + b4 * np.cos(-b5)
                        self.gy = self.cy + b4 * np.sin(-b5)
                    elif np.argmin(dr[i]) < minz:
                        self.gz = self.cz + b5
                        self.gx = self.cx + b4 * np.cos(b5)
                        self.gy = self.cy + b4 * np.sin(b5)
            if minx > b1:
                self.goal = self.pos[self.ind, :]
            elif self.ind > 2:
                if minx < b3:
                    self.goal = [self.gx, self.gy]
                    print('entering obstacle avoider')
                    print(f"minx is {minx} and minz is {minz}")
                self.su = False

    def _motion_controller(self, Odometry):
        self.update_Odometry(Odometry)

        print("New Time Step")

        current = Odometry.pose.pose
        self.cx = self.globalPos.x
        self.cy = self.globalPos.y
        self.cz = self.globalAng

        self.su = True

        print(self.goal)
        print(f"GX = {self.goal[0]} // GY = {self.goal[1]}")
        if np.linalg.norm([(self.goal[1] - self.cy), (self.goal[0] - self.cx)]) < 0.15:
            self.ind += 1
            self.vel.linear.x = 0.0
            self.vel.angular.z = 0.0
            time.sleep(10)
            self.goal = self.pos[self.ind, :]

        print(self.ind)

        gx = self.goal[0]
        gy = self.goal[1]

        ey = gy - self.cy
        ex = gx - self.cx
        gz = np.arctan2(ey, ex)

        ld = np.linalg.norm([(gy - self.cy), (gx - self.cx)])

        print(f"cx = {self.cx} // cy = {self.cy} // cz = {self.cz}")
        print(f"ld = {ld} and gz = {gz}")

        if abs(gz - self.cz) > 0.1:

            if (gz - self.cz) < -np.pi:
                ez = gz - self.cz + 2 * np.pi
                self.vel.angular.z = min((self.kpz * ez), 1.0)
            elif (gz - self.cz) > np.pi:
                ez = gz - self.cz - 2 * np.pi
                self.vel.angular.z = min((self.kpz * ez), 1.0)
            else:
                self.vel.angular.z = min((self.kpz * (gz - self.cz)), 1.0)

        if abs(self.vel.angular.z) > 0.15:
            self.vel.linear.x = 0.0

        else:
            if ld < 0.17:
                self.vel.linear.x = 0.0

            elif ld > 0.19:
                self.vel.linear.x = 0.19

        print(f"vx = {self.vel.linear.x} // vz = {self.vel.angular.z}")
        self._velocity_publisher.publish(self.vel)

    def update_Odometry(self, Odometry):
        position = Odometry.pose.pose.position

        # Orientation uses the quaternion aprametrization.
        # To get the angular position along the z-axis, the following equation is required.
        q = Odometry.pose.pose.orientation
        orientation = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

        if self.Init:
            # The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix(
                [[np.cos(self.Init_ang), np.sin(self.Init_ang)], [-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
            self.Init_pos.x = Mrot.item((0, 0)) * position.x + Mrot.item((0, 1)) * position.y
            self.Init_pos.y = Mrot.item((1, 0)) * position.x + Mrot.item((1, 1)) * position.y
            self.Init_pos.z = position.z

        Mrot = np.matrix(
            [[np.cos(self.Init_ang), np.sin(self.Init_ang)], [-np.sin(self.Init_ang), np.cos(self.Init_ang)]])

        # We subtract the initial values
        self.globalPos.x = Mrot.item((0, 0)) * position.x + Mrot.item((0, 1)) * position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1, 0)) * position.x + Mrot.item((1, 1)) * position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        self.get_logger().info('Position index is {}'.format(self.ind))


def main():
    rclpy.init()
    obj = GoToGoal()
    while rclpy.ok():
        rclpy.spin_once(obj)
    obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

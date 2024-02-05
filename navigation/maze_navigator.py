import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist
import numpy as np

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
        self.kpz = 0.5

        self.Init_pos = Point()
        self.globalPos = Point()

        self.cx = 0.0
        self.cy = 0.0
        self.cz = 0.0

        self.sd = 0.0
        self.rd = 0.0
        self.ld = 0.0

        self.sign = 0.0
        self.state = 2

        self._odom_subscriber = self.create_subscription(Odometry, '/odom', self._state_machine, 1)
        self._odom_subscriber

        self._vector_subscriber = self.create_subscription(LaserScan, '/scan', self._laser_callback, qos_profile)
        self._vector_subscriber

        self._coord_subscriber = self.create_subscription(Point, '/coord', self._point_callback, 1)
        self._coord_subscriber

        self._velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self._velocity_publisher

        self._bool_publisher = self.create_publisher(Point, '/bolo', 1)
        self._bool_publisher

    def _laser_callback(self, LaserScan):

        self.ls = LaserScan
        if self.su:
            self.sd = self.ls.ranges[0]
            self.ld = self.ls.ranges[59]
            self.rd = self.ls.ranges[169]
            print(f"LD = {self.ld}")
            print(f"RD = {self.rd}")

    def _point_callback(self, Point):
        self.sign = Point.x
        print(self.sign)

    def _state_machine(self, Odometry):

        # Initialize Odometry and LIDAR
        self.update_Odometry(Odometry)
        self.su = True

        print(f"Front Dist is {self.sd}")
        print(f"State is {self.state}")

        # Odometry Global Coordinates
        self.cx = self.globalPos.x
        self.cy = self.globalPos.y
        self.cz = self.globalAng

        if self.sd < 0.45:
            self.state = 2
        elif self.sd > (0.45 + 0.5):
            self.state = 1

        # Go Straight State
        if self.state == 1:
            self.vx = 0.15
            if self.ld < 0.5:
                self.vz = self.kpz * (0.45 - self.ld)
            elif self.rd < 0.5:
                self.vz = self.kpz * (0.45 - self.rd)
            else:
                self.vz = 0.0

            self.vel.linear.x = self.vx
            self.vel.angular.z = self.vz
            self._velocity_publisher.publish(self.vel)

        # Classify State
        if self.state == 2:

            # Set Bool to 1.0 and Publish
            pt = Point()
            self.vx = 0.0
            self.vz = 0.0
            pt.y = 1.0
            self._bool_publisher.publish(pt)

            # Publish Zero Velocities
            self.vel.linear.x = self.vx
            self.vel.angular.z = self.vz
            self._velocity_publisher.publish(self.vel)

            # Store Initial Heading
            self.angle = self.cz

            # Calculate Final Heading
            if self.sign == 1.0:
                self.gz = self.angle + np.p i /2
            if self.sign == 2.0:
                self.gz = self.angle - np.p i /2
            if self.sign == 3.0:
                self.gz = self.angle + np.pi
            if self.sign == 4.0:
                self.gz = self.angle + np.pi
            if self.sign == 5.0:
                self.gz = self.angle
            if self.sign == 0.0:
                self.gz = self.angle + np.p i /4

            # Exit Classify State
            if pt.y == 1.0:
                self.state = 3

        # Turn State
        if self.state == 3:

            # Stop and Define Angular Error
            self.vx = 0.0
            self.ez = self.gz - self.cz

            # Angle Wrapping
            if abs(self.ez) > 0.1:

                if (self.ez) < -np.pi:
                    self.vz = self.kpz * (self.ez + 2 * np.pi)

                elif (self.ez) > np.pi:
                    self.vz = self.kpz * (self.ez - 2 * np.pi)

                else:
                    self.vz = (self.kpz * self.ez)

            # Publish Turning Velocity
            self.vel.linear.x = self.vx
            self.vel.angular.z = self.vz
            self._velocity_publisher.publish(self.vel)

            # Exit Turning State
            if abs(self.vz) < 0.1:
                state = 1
            else:
                state = 3

        print(f"VX = {self.vel.linear.x} and VZ = {self.vel.angular.z}")


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

        # self.get_logger().info('Position index is {}'.format(self.ind))


def main():
    rclpy.init()
    obj = GoToGoal()
    while rclpy.ok():
        rclpy.spin_once(obj)
    obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from alphabot2_interfaces.msg import Obstacle

VIRTUAL_ODOMETRY_TOPIC = "virtual_odometry"
CMD_VEL_TOPIC = "cmd_vel"
OBSTACLES_TOPIC = "obstacles"
SPIN_TIMER_PERIOD_SEC = 0.025   # Okres wywolania zwrotnego timer'a
CMD_VEL_PUB_PERIOD_SEC = 0.025  # okres wysylania polecen przez cmd_vel


class VirtualOdometer(Node):
    """
    Wezel odpowiedzialny za symulowanie odometrii, z racji ze AlphaBot2 nie posiada zadnego czujnika odometrii
    Wezel subskrybuje topic'i cmd_vel i obstacles oraz publikuje szacowana predkosc do topic'a virtual_odometry 

    """
    def __init__(self):

        super().__init__("virtual_odometer")

        self.get_logger().info("Node init ...")

        self.timer = self.create_timer(SPIN_TIMER_PERIOD_SEC, self.virtual_odometry_pub_callback)

        # Publisher i subskrybent
        self.cmd_vel_sub = self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_sub_callback, 10)
        self.obstacles_sub = self.create_subscription(Obstacle, OBSTACLES_TOPIC, self.obstacles_sub_callback, 10)
        self.virtual_odometry_pub = self.create_publisher(Twist, VIRTUAL_ODOMETRY_TOPIC, 10)

        # Wewnetrzny status
        self.linear = 0                 # linear velocity
        self.angular = 0                # angular rate
        self.obstacle_detected = False  # True if there is an obstacle

        self.get_logger().info("Node init complete.")

    def cmd_vel_sub_callback(self, twist_msg):
        """
        Funkcja wzywana gdy pojawi sie nowe polecenie od cmd_vel
        """
        # Kopiowanie wartosci liniowej i katowej od cmd_vel
        self.linear = twist_msg.linear.x
        self.angular = twist_msg.angular.z

    def obstacles_sub_callback(self, obstacle_msg):
        """
        Funkcja wzywana gdy pojawi sie nowa informacja od topic'a Obstacles
        """
        # Aktualizacja statusu wewnetrznego
        self.obstacle_detected = obstacle_msg.right_obstacle or obstacle_msg.left_obstacle

    def virtual_odometry_pub_callback(self):
        """
        Funkcja wzywana gdy pojawi sie nowa wiadomosc od topic'a virtual_odometry
        Reprezentuje szacowane predkosci robota
        """
        # Tworzenie wiadomosci twist
        twist_msg = Twist()
        twist_msg.linear.x = float(self.linear)
        twist_msg.angular.z = float(self.angular)

        # Zabrania ruchowi naprzod jesli wykryto przeszkode
        # if self.obstacle_detected and twist_msg.linear.x > 0:
        #    twist_msg.linear.x = float(0) 

        # Wyslanie wiadomosci
        self.virtual_odometry_pub.publish(twist_msg)
        self.get_logger().info(f"Publishing [{VIRTUAL_ODOMETRY_TOPIC}] >> "
                               f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")


def main(args=None):
    """
    Funkcja main wezla virtual_odometer.py
    """
    rclpy.init(args=args)

    virtual_odometer = VirtualOdometer()
    rclpy.spin(virtual_odometer)
    virtual_odometer.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()

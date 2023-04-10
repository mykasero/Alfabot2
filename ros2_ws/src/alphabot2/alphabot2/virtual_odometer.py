import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from alphabot2_interfaces.msg import Obstacle

VIRTUAL_ODOMETRY_TOPIC = "virtual_odometry"
CMD_VEL_TOPIC = "cmd_vel"
OBSTACLES_TOPIC = "obstacles"


class VirtualOdometer(Node):
    """
    Wezel odpowiedzialny za symulowanie odometrii, z racji ze AlphaBot2 nie posiada zadnego czujnika odometrii
    Wezel subskrybuje topic'i cmd_vel i obstacles oraz publikuje szacowana predkosc do topic'a virtual_odometry 

    """
    def __init__(self):
        super().__init__("virtual_odometer")
        self.get_logger().info("Node init ...")
        # Utworzenie timera funkcji virtual_odometry_pub_callback z wywolaniem co 0.025s
        self.timer = self.create_timer(0.025, self.virtual_odometry_pub_callback)
        # Utworzenie subscribera tematu cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_sub_callback, 10)
        # Utworzenie subscribera tematu obstacles
        self.obstacles_sub = self.create_subscription(Obstacle, OBSTACLES_TOPIC, self.obstacles_sub_callback, 10)
        # Utworzenie publishera do tematu virtual_odometry
        self.virtual_odometry_pub = self.create_publisher(Twist, VIRTUAL_ODOMETRY_TOPIC, 10)

        # Wewnetrzny status
        self.linear = 0                 # predkosc liniowa
        self.angular = 0                # predkosc katowa
        self.obstacle_detected = False  # True jesli wykryto przeszkode

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
        Funkcja wzywana gdy pojawi sie nowa informacja od tematu Obstacles
        """
        # Aktualizacja statusu wewnetrznego
        self.obstacle_detected = obstacle_msg.right_obstacle or obstacle_msg.left_obstacle

    def virtual_odometry_pub_callback(self):
        """
        Funkcja wzywana gdy pojawi sie nowa informacja z funkcji cmd_vel_sub_callback
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

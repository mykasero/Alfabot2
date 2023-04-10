import time
import rclpy
from rclpy.node import Node

from alphabot2_interfaces.msg import Obstacle
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

OBSTACLES_TOPIC = "obstacles"
CMD_VEL_TOPIC = "cmd_vel"

def gpio_close():

    # Funkcja zamykajaca GPIO

    GPIO.cleanup()

class Avoiding(Node):
    def __init__(self):
        super().__init__("avoiding")
        self.get_logger().info("Node init ...")
        
        # Utworzenie timer'a z funkcja wywolania zwrotnego co 1s
        self.timer_time = self.create_timer(0.1, self.timer_time_count)
        self.timer_avoiding = self.create_timer(0.1, self.avoiding_pub_callback)
        #self.cmdvel_sub = self.create_subscription(Twist,CMD_VEL_TOPIC,self.cmd_vel_sub_callback,10)
        self.obstacles_sub = self.create_subscription(Obstacle, OBSTACLES_TOPIC, self.obstacles_sub_callback, 10)
        self.avoiding_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.time = 0
        self.dist = 0
        self.side = False
        self.main_track = False
        self.bok1 = True
        self.bok2 = False
        self.bok3 = False
        self.get_logger().info("avoiding init complete.")
    
    def timer_time_count(self):
        self.time+=0.1

    def obstacles_sub_callback(self, obstacle_msg):
        """
        Funkcja wzywana gdy pojawi sie nowa informacja od topic'a Obstacles
        """
        # Aktualizacja statusu wewnetrznego
        self.obstacle_detected = obstacle_msg.right_obstacle or obstacle_msg.left_obstacle

    def avoiding_pub_callback(self):
        
        """
        Funkcja wysylajaca instrukcje do robota w celu ominiecia przeszkody
        """
        #print("czas: ",self.time,"droga: ",self.dist)
        # Tworzenie wiadomosci twist
        twist_msg = Twist()
        #starting trip to point B
        twist_msg.linear.x = float(0.3)
        twist_msg.angular.z = float(0)
        self.avoiding_pub.publish(twist_msg)
        self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        
        if self.main_track and self.time>101:
            print("powrot na maintrack")
            twist_msg.linear.x = float(0.3)
            twist_msg.angular.z = float(0)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        #3bok
        if self.dist == 0 and self.bok3:
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(2)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            self.main_track = True
            self.time=100
            self.bok3=False
        if not self.obstacle_detected and self.time>18 and self.time<100:
            print("jazda wzdluz 3 boku")
            twist_msg.linear.x = float(0.3)
            twist_msg.angular.z = float(0)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            self.dist-=1
            self.bok2=False
            self.bok3=True
        #2bok
        if self.bok2 and self.obstacle_detected and self.time>17 and self.time<=18:
            print("sprawdzam przeszkode 2bok")
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(2)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            if self.time > 17.8 and self.time<=18:
                self.time=14
        if self.bok2 and not self.obstacle_detected and self.time>16 and self.time<=17:
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(-2)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        if not self.obstacle_detected and self.time>=14 and self.time <=16:
            print("Jazda wzdluz 2boku")
            twist_msg.linear.x = float(0.3)
            twist_msg.angular.z = float(0)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            self.bok1 = False
            self.bok2= True
            self.side = True
        #bok1
        if not self.obstacle_detected and self.time >= 13 and self.time<14:
            print("Sprawdzam przeszkode 1bok")
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(-2)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            if self.time==13.9:
                self.time=10
        if not self.obstacle_detected and self.time >11 and self.time<13:
            print("jazda wzdluz 1 boku")
            twist_msg.linear.x = float(0.3)
            twist_msg.angular.z = float(0)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            self.dist+=1
        if not self.obstacle_detected and self.time<=11:
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(2)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        if self.obstacle_detected and self.bok1:
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(2)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            
        if not self.obstacle_detected and self.time<10:
            twist_msg.linear.x = float(0.3)
            twist_msg.angular.z = float(0)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
      
       
def main(args=None):
    """
    Funkcja main wezla Avoiding.py
    """
    rclpy.init(args=args)

    # Utworz, spin'uj i zniszcz wezel
    avoid = Avoiding()
    rclpy.spin(avoid)
    avoid.destroy_node()

    gpio_close()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
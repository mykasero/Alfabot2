import time
import rclpy
from rclpy.node import Node
from alphabot2_interfaces.msg import Obstacle
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

OBSTACLES_TOPIC = "obstacles" #temat obstacles
CMD_VEL_TOPIC = "cmd_vel"   #temat cmd_vel

def gpio_close():

    # Funkcja zamykajaca GPIO

    GPIO.cleanup()

class Avoiding(Node):
    def __init__(self):
        super().__init__("avoiding")
        self.get_logger().info("Node init ...")
        
        # Utworzenie timer'a z funkcja wywolania zwrotnego co 0.1s
        self.timer_time = self.create_timer(0.1, self.timer_time_count)
        # Utworzenie timera funkcji avoiding_pub_callback z wywolaniem co 0.1s
        self.timer_avoiding = self.create_timer(0.1, self.avoiding_pub_callback)
        # Utworzenie subscribera tematu obstacles
        self.obstacles_sub = self.create_subscription(Obstacle, OBSTACLES_TOPIC, self.obstacles_sub_callback, 10)
        # Utworzenie publishera do tematu CMD_VEL
        self.avoiding_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        #zmienne pomocniczne (W celu poprawnego działania zmienne pomocnicze należy definiować wewnątrz funkcji init)
        self.time = 0
        self.dist = 0
        self.side = False
        self.main_track = False
        self.bok1 = False
        self.bok2 = False
        self.bok3 = False
        self.get_logger().info("avoiding init complete.")
    
    # timer iterujący zmienną pomocniczą
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
        # Tworzenie wiadomosci twist
        twist_msg = Twist()
        # Ustalenie i wysłanie prędkości początkowej
        twist_msg.linear.x = float(0.3)
        twist_msg.angular.z = float(0)
        self.avoiding_pub.publish(twist_msg)
        self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        # Sekwencja omijania przeszkody
        if self.main_track and self.time>101:
            print("Powrot na glowna trase")
            twist_msg.linear.x = float(0.3)
            twist_msg.angular.z = float(0)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        #3bok
        if self.dist == 0 and self.bok3:
            print("Dojechalem do glownej trasy, ustawiam sie w kierunku punktu B")
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(5)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            self.main_track = True
            self.time=100
            self.bok3=False
        if not self.obstacle_detected and self.time>18 and self.time<100:
            if self.time > 18 and self.time<18.3:
                print("Nie wykryto przeszkody - Jazda wzdluz trzeciego boku")
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
            print("Wykryto przeszkode - drugi bok")
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(5)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
 
            if self.time>=17.8 and self.time<=18:
                self.time=14
        if self.bok2 and not self.obstacle_detected and self.time>16 and self.time<=17:
            if self.time>16 and self.time<16.3:
                print("Sprawdzam przeszkode - drugi bok")
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(-5)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        if not self.obstacle_detected and self.time>=14 and self.time <=16:
            if self.time>14 and self.time < 14.3:
                print("Nie wykryto przeszkody - Jazda wzdluz drugiego boku")
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
            if self.time >13 and self.time<13.3:
                print("Sprawdzam przeszkode - pierwszy bok")
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(-5)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            
        if not self.obstacle_detected and self.time >11 and self.time<13:
            if self.time >= 11.1 and self.time <= 11.3:
                print("Jazda wzdluz pierwszego boku")
            twist_msg.linear.x = float(0.3)
            twist_msg.angular.z = float(0)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            self.dist+=1
        if not self.obstacle_detected and self.time<=11:
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(5)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        if self.obstacle_detected and self.bok1:
            twist_msg.linear.x = float(0)
            twist_msg.angular.z = float(5)
            self.avoiding_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
            if self.time>13.8 and self.time<14:
                self.time=10
            if self.time==10:
                print("Wykryto przeszkode - pierwszy bok")
        if self.obstacle_detected and not self.bok1 and not self.bok2 and not self.bok3:
            print("Wykryto przeszkode, rozpoczynam omijanie") 
            self.bok1 = True
            self.time=10
        if not self.obstacle_detected and self.time<10:
            print("Jazda z punktu A do B")
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
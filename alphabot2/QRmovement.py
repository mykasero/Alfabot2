import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import RPi.GPIO as GPIO

QR_TOPIC = "qr_codes"
CMD_VEL_TOPIC = "cmd_vel"


def gpio_close():

    # Funkcja zamykajaca GPIO

    GPIO.cleanup()
class QRmove(Node):
    def __init__(self):
        super().__init__("QRmovement")
        self.get_logger().info("Node init ...")
        
        # Utworzenie timer'a z funkcja wywolania zwrotnego co 1s
        self.timer_sub = self.create_timer(0.5, self.QRmove_pub_callback)
        self.x = 2
        self.time = 2
        self.timer_time = self.create_timer(0.5, self.timer_callback)
        self.QRtopic_sub = self.create_subscription(String, QR_TOPIC, self.QRtopic_sub_callback, 10)
        self.QR_message=""
        self.QRmove_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.get_logger().info("QR movement init complete.")
        self.start= False

    def timer_callback(self):
        self.time += 0.5

    def QRtopic_sub_callback(self, string_msg):
        
        self.QR_message = string_msg.data
        if self.QR_message in ['1','2','3','4','5','6']:
            self.time=0
    
    def QRmove_pub_callback(self):
        twist_msg = Twist() 
        
        if (self.QR_message == '' and self.time >self.x):
            print("Searching for QR")
            twist_msg.linear.x = float(0.45)
            twist_msg.angular.z = float(0.0)
            self.QRmove_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}") 

        if (self.QR_message == '' and self.time <self.x):
            print("Searching for QR")
            twist_msg.linear.x = float(0.0)
            twist_msg.angular.z = float(5)
            self.QRmove_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}") 
                                
        if self.QR_message == '1':
            print("Found QR code number 1, moving to number 2")
            self.x = 2.0
            twist_msg.linear.x = float(0.0)
            twist_msg.angular.z = float(5)
            self.QRmove_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        if self.QR_message == '2':
            print("Found QR code number 2, moving to number 3")
            self.x = 4.0
            twist_msg.linear.x=float(0)
            twist_msg.angular.z = float(5)
            self.QRmove_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        if self.QR_message == '3':
            print("Found QR code number 3, moving to number 5")
            self.x = 4.0
            twist_msg.linear.x=float(0)
            twist_msg.angular.z = float(5)
            self.QRmove_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        if self.QR_message == '5':
            print("Found QR code number 5, moving to start point")
            self.x = 4.0
            twist_msg.linear.x=float(0)
            twist_msg.angular.z = float(5)
            self.start=True
            self.QRmove_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        if self.QR_message == '6':
            print("Found QR code number 6, moving to start point")
            self.x = 4.0
            twist_msg.linear.x=float(0)
            twist_msg.angular.z = float(5)
            self.start=True
            self.QRmove_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")
        """if self.time>ustalic and self.start= True:
            print("Koniec jazdy")
            twist_msg.linear.x=float(0)
            twist_msg.angular.z = float(0)
            self.QRmove_pub.publish(twist_msg)
            self.get_logger().info(f"Publishing [{CMD_VEL_TOPIC}] >> "
                                f"linear={twist_msg.linear.x:.3}, angular={twist_msg.angular.z:.3}")"""
        print("Czas: ",self.time)

def main(args=None):

    rclpy.init(args=args)

    # Utworz, spin'uj i zniszcz wezel
    qrmovement = QRmove()
    rclpy.spin(qrmovement)
    qrmovement.destroy_node()

    gpio_close()

    rclpy.shutdown()


if __name__ == '__main__':
    main() 

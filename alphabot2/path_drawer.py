import math
import time
import turtle

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

VIRTUAL_ODOMETRY_TOPIC = "virtual_odometry"
QR_TOPIC = "qr_codes"

s = turtle.getscreen()
global map 
map = turtle.Turtle()
turtle.title("Mapa drogi pokonanej przez robota")
map.pendown()
#pozycje markerow QR, dodaj pozsotale
#QR1_x = 300 
#QR1_z = 300
#QR2_x = -300
#QR2_z =  150

class PathDrawer(Node):
    global self
    def __init__(self):
        super().__init__("path_drawer")
        self.get_logger().info("Node init ...")
            
        self.VirtualOdometer_sub = self.create_subscription(Twist, VIRTUAL_ODOMETRY_TOPIC, self.VOtopic_sub_callback, 10)

        self.QRCodes_sub = self.create_subscription(String, QR_TOPIC, self.QRtopic_sub_callback, 10)
        self.distance=0
        self.distance1=0
        self.distance2=0
        self.distance3=0
        self.distance4=0
        self.distance5=0
        self.distance6=0
        self.get_logger().info("path_drawer init complete.")
    
    #Rysowanie trasy na mapie
    def VOtopic_sub_callback(self, Twist_msg):
        self.Odometry= Twist_msg
        if(self.Odometry.linear.x > 0):
            map.fd(self.Odometry.linear.x)
            self.distance+=self.Odometry.linear.x
        if(self.Odometry.linear.x < 0):
            map.bk(self.Odometry.linear.x)
            self.distance-=abs(self.Odometry.linear.x)
        if(self.Odometry.angular.z > 0):
            map.rt(0.2*(self.Odometry.angular.z))
        if(self.Odometry.angular.z < 0):
            map.lt(0.2*(self.Odometry.angular.z))
        #print("\nDystans", self.distance)
    #Relokacja na mapie do kodu QR
    def QRtopic_sub_callback(self, QR_msg):
        self.QR = QR_msg.data
        if(self.QR == '1'):
            #pozycja QR1
            print("Przemieszczenie do QR1 na mapie")
            self.distance1 = self.distance
            print("Distance1: ",self.distance1)
            self.distance = 0
            #map.goto(QR1_x,QR1_z)
        if(self.QR == '2'):
            #print("Korekta do QR2 na mapie")
            self.distance2=self.distance
            print("Distance2: ", self.distance2)
            self.distance=0
            #map.goto(QR2_x,QR2_z)
        if(self.QR == '3'):
            self.distance3=self.distance
            print("Distance3: ", self.distance3)
            self.distance=0
        if(self.QR == '5'):
            self.distance5=self.distance
            print("Distance5: ", self.distance5)
            self.distance=0

def main(args=None):
    
    rclpy.init(args=args)

    path_drawer = PathDrawer()
    rclpy.spin(path_drawer)
    path_drawer.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
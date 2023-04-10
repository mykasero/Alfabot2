import rclpy
from rclpy.node import Node
from alphabot2_interfaces.msg import Obstacle
import RPi.GPIO as GPIO

"""
Alphabot2 jest wyposazony w dwa czujniki podczerwieni ST188, do wykrywania przeszkod.
Czujniki posiadaja odwrocona logike i wysylaja sygnal LOW gdy widza przeszkode a sygnal HIGH gdy jej nie widza
"""

# ST188 / Raspberry Pi GPIO mapowanie pin'ów
DL_PIN = 16  # Lewy czujnik
DR_PIN = 19  # Prawy czujnik

OBSTACLES_TOPIC = "obstacles" 


def gpio_init():
    """
    Funkcja ustawiajaca GPIO
    """
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)


def gpio_close():
    """
    Funkcja zamykajaca GPIO
    """
    GPIO.cleanup()


class IRObstacleSensors(Node):
    """
    Wezel kontrolujacy czujniki podczerwieni AlphaBot'a 
    Wysyla wiadomosc do tematu ,,obstacles"
    """
    def __init__(self, left_sensor_pin=DL_PIN, right_sensor_pin=DR_PIN):

        super().__init__("IR_obstacle_sensors")
        self.get_logger().info("Node init ...")
        # Ustawienie numerow PIN'ow
        self.left_sensor_pin = left_sensor_pin
        self.right_sensor_pin = right_sensor_pin
        ## Ustawienie PIN'ow jako wejscie
        GPIO.setup(self.left_sensor_pin, GPIO.IN, GPIO.PUD_UP)
        GPIO.setup(self.right_sensor_pin, GPIO.IN, GPIO.PUD_UP)
        # Utworzenie timer'a funkcji obstacles_pub_callback z wywolaniem co 0.025s
        self.timer = self.create_timer(0.025, self.obstacles_pub_callback)
        # Utworzenie publishera do tematu obstacles
        self.obstacles_pub = self.create_publisher(Obstacle, OBSTACLES_TOPIC, 10)
        # Utworzenie zmiennych odpowiedzialnych za przechowywanie statusu czujnikow
        self.left_obstacle = not GPIO.input(self.left_sensor_pin)  
        self.right_obstacle = not GPIO.input(self.right_sensor_pin)

        self.get_logger().info("Node init complete.")

    def check_for_obstacles(self):
        """
        Sprawdza czy czujniki wykrywaja przeszkode
        Uzyto inwersji NOT, poniewaz czujniki maja odwrocona logike
        """
        self.left_obstacle = not GPIO.input(self.left_sensor_pin)
        self.right_obstacle = not GPIO.input(self.right_sensor_pin)

    def obstacles_pub_callback(self):
        """
        # Wysyla nowa wiadomosc o przeszkodzie do tematu ,,obstacles"
        """
        self.check_for_obstacles()

        # Utworzenie wiadomosci Obstacles
        obstacle_msg = Obstacle()
        obstacle_msg.left_obstacle = self.left_obstacle
        obstacle_msg.right_obstacle = self.right_obstacle

        # Wyslanie wiadomosci
        self.obstacles_pub.publish(obstacle_msg)
        self.get_logger().info(f"Publishing >> left_obstacle={self.left_obstacle}, right_obstacle={self.right_obstacle}")


def main(args=None):
    """
    Funkcja main wezla IR_obstacle_sensors.py
    """
    rclpy.init(args=args)
    gpio_init()

    # Utworz, spin'uj i zniszcz wezel
    ir_obstacle_sensors = IRObstacleSensors()
    rclpy.spin(ir_obstacle_sensors)
    ir_obstacle_sensors.destroy_node()

    gpio_close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

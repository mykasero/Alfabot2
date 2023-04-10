import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from alphabot2_interfaces.msg import Obstacle

import RPi.GPIO as GPIO

# Serwa Toshiba TB6612FNG / Mapowanie GPIO
PWMA_PIN = 6    # PWM lewego serwa
AIN1_PIN = 12   # Lewy motor naped do tylu
AIN2_PIN = 13   # Lewy motor naped do przodu
PWMB_PIN = 26   # PWM prawego serwa
BIN1_PIN = 20   # Prawy motor naped do tylu
BIN2_PIN = 21   # Prawy motor naped do przodu

DEFAULT_MOTORS_PWM_DUTY_CYCLE = 50  # Domyslny PWM duty cycle prawego i lewego serwa
DEFAULT_MOTORS_PWM_FREQUENCY = 500  

SPIN_TIMER_PERIOD_SEC = 0.025       # Okres wywołania zwrotnego timera
MAX_BRAKING_DURATION_SEC = 0.10     # Maksymalna dlugosc trwania obrotu serwa w przeciwna strone w celu hamowania

WHEELS_DIST_M = 0.085                       # 85 mm, odleglosc miedzy kolami
WHEEL_RADIUS_M = 0.021                      # 21 mm, srednica kol
MAX_MOTOR_RPM = 750                         # Maksymalne RPM Alphabota
RPM_TO_RAD_PER_SEC = 2 * math.pi / 60       # Zamiana wartosci RPM na rad/s

OBSTACLES_TOPIC = "obstacles"
CMD_VEL_TOPIC = "cmd_vel"


def gpio_init():
    
    # Funkcja inicjalizujaca GPIO

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)


def gpio_close():
   
    # Funkcja zamykajaca GPIO

    GPIO.cleanup()


def clip(value, minimum, maximum):
    
    # Funkcja zapobiegajaca przekraczaniu granic minimum i maksimum dla zmiennej value

    if value < minimum:
        return minimum
    elif value > maximum:
        return maximum
    return value


class Motor:

    # Klasa "Motor" odpowiedzialna za serwa Alphabot'a

    def __init__(self, forward_pin, backward_pin, speed_pwm_pin):

        # Przydzielanie numerow pin'ow
        self.forward_pin = forward_pin
        self.backward_pin = backward_pin
        self.speed_pwm_pin = speed_pwm_pin

        # Ustawia pin'y jako wyjście
        GPIO.setup(self.forward_pin, GPIO.OUT)
        GPIO.setup(self.backward_pin, GPIO.OUT)
        GPIO.setup(self.speed_pwm_pin, GPIO.OUT)

        # Uruchamianie pin'u PWMs
        self.speed_pwm = GPIO.PWM(self.speed_pwm_pin, DEFAULT_MOTORS_PWM_FREQUENCY)
        self.speed_pwm.start(DEFAULT_MOTORS_PWM_DUTY_CYCLE)

        # Zatrzymanie serwa, w celach bezpieczeństwa
        self.stop()

    def stop(self):
        
        #Funkcja zatrzymujaca serwo
        
        # Ustawienie PWM duty cycle jako zero
        self.speed_pwm.ChangeDutyCycle(0)

        # Ustawienie stanu pin'ow na low
        GPIO.output(self.forward_pin, GPIO.LOW)
        GPIO.output(self.backward_pin, GPIO.LOW)

    def run(self, signed_speed_percent):

        # Funkcja nadająca serwu zadana predkosc (dodatnie wartosci oznaczaja obrot kol naprzod, ujemne do tylu)
        
        # Zmiana predkosci serwa (duty cycle moze byc pomiedzy 0 a 100)
        duty_cycle = clip(abs(signed_speed_percent), 0, 100)
        self.speed_pwm.ChangeDutyCycle(duty_cycle)

        # Zaleznie od kierunku jazdy, ustawia poszczegolne piny na HIGH lub LOW
        if signed_speed_percent >= 0:
            GPIO.output(self.forward_pin, GPIO.HIGH)
            GPIO.output(self.backward_pin, GPIO.LOW)
        else:
            GPIO.output(self.forward_pin, GPIO.LOW)
            GPIO.output(self.backward_pin, GPIO.HIGH)


class MotionDriver(Node):
    
    # Wezel kontrolujacy ruch Alphabot'a. Subskrybuje on temat "cmd_vel" i "obstacles"
    
    def __init__(self,
                 l_motor_fw_pin=AIN2_PIN,
                 l_motor_bw_pin=AIN1_PIN,
                 l_motor_speed_pwm_pin=PWMA_PIN,
                 r_motor_fw_pin=BIN2_PIN,
                 r_motor_bw_pin=BIN1_PIN,
                 r_motor_speed_pwm_pin=PWMB_PIN):

        super().__init__("motion_driver")

        self.get_logger().info("Node init ...")

        # Utworzenie timera wykorzystanego przez funkcje spin_timer_callback
        self.spin_timer = self.create_timer(SPIN_TIMER_PERIOD_SEC, self.spin_timer_callback)

        # Utworzenie subskrypcji tematu cmd_vel
        self.cmd_vel_sub = self.create_subscription(Twist, CMD_VEL_TOPIC, self.cmd_vel_sub_callback, 10)

        # Instancje Lewego i Prawego serwa
        self.left_motor = Motor(l_motor_fw_pin, l_motor_bw_pin, l_motor_speed_pwm_pin)
        self.right_motor = Motor(r_motor_fw_pin, r_motor_bw_pin, r_motor_speed_pwm_pin)

        self.left_signed_speed_percent = 0   # pomiedzy -100 and +100
        self.right_signed_speed_percent = 0  # pomiedzy -100 and +100

        self.last_received_vel_timestamp_sec = self.get_clock().now().to_msg().sec  # czas otrzymania ostatniego polecenia od cmd_vel
                                                                                    
        self.obstacle_detected = False  # Jesli nie wykryto przeszkody, umozliwia jazde
        self.braked = False             # Hamowanie wylaczone

        self.get_logger().info("Node init complete.")

    def brake(self):

        # Funkcja hamująca przy uzyciu serw, wykorzystujaca obrot w przeciwnym kierunku przez krotki okres czasu
        
        self.braked = True
        
        self.left_motor.run(-self.left_signed_speed_percent)
        self.right_motor.run(-self.right_signed_speed_percent)
        
        max_abs_speed_percent = max(abs(self.right_signed_speed_percent), abs(self.left_signed_speed_percent))
        time.sleep(MAX_BRAKING_DURATION_SEC * max_abs_speed_percent / 100)

        # Zatrzymanie serw
        self.left_motor.stop()
        self.right_motor.stop()

    def move(self):

        # Funkcja ruszająca serwami zależnie od podanej predkosci

        self.braked = False

        self.left_motor.run(self.left_signed_speed_percent)
        self.right_motor.run(self.right_signed_speed_percent)

    def set_wheels_speed(self, linear_velocity, angular_rate):
        """
        Funkcja ustawiajaca speed_percent dla obu serw, rozpoczynajaca od linear_velocity i angular_rate.
        Jesli wybrana predkosc przekracza mozliwosci serw, predkosc jest normalizowana wzgledem mozliwosci silniczkow
        
        Z racji, że Alphabot2 nie posiada predkosciomierzy, wykorzystany zostanie wzor na konwersje z RPM na wartosc predkosci
        
        Wyliczanie RPM:
            left_rpm = (linear_velocity - 0.5 * angular_rate * WHEEL_DIST) / (RPM_TO_RAD_PER_S * DIST_PER_RAD)
            right_rpm = (linear_velocity + 0.5 * angular_rate * WHEEL_DIST) / (RPM_TO_RAD_PER_S * DIST_PER_RAD)
        gdzie:
            RPM_TO_RAD_PER_S = 2 * pi / 60
            DIST_PER_RAD = (2 * pi * WHEEL_RADIUS) / (2 * pi) = WHEEL_RADIUS
        
        Wzor na zamiane RPM na speed_percent:
            wheel_speed_percent = 100 * wheel_rpm / MAX_WHEEL_RPM
        
        Zrodlo: https://robotics.stackexchange.com/questions/18048/inverse-kinematics-for-differential-robot-knowing-linear-and-angular-velocities
        """
        # Zamiana linear_velocity i angular_rate na RPM lewego i prawego kola
        left_rpm = (linear_velocity - 0.5 * angular_rate * WHEELS_DIST_M) / (RPM_TO_RAD_PER_SEC * WHEEL_RADIUS_M)
        right_rpm = (linear_velocity + 0.5 * angular_rate * WHEELS_DIST_M) / (RPM_TO_RAD_PER_SEC * WHEEL_RADIUS_M)

        # Normalizacja RPM, jesli zadana wartosc przekroczy mozliwosci robota
        max_abs_rpm = max(abs(left_rpm), abs(right_rpm))
        if max_abs_rpm > MAX_MOTOR_RPM:
            exceed_ratio = max_abs_rpm / MAX_MOTOR_RPM
            left_rpm = left_rpm / exceed_ratio
            right_rpm = right_rpm / exceed_ratio

        # Zamiana z RPM na speed_percent
        self.left_signed_speed_percent = (100 * left_rpm / MAX_MOTOR_RPM)  # pomiedzy -100 a 100           
        self.right_signed_speed_percent = (100 * right_rpm / MAX_MOTOR_RPM)                                   
        #print("Lewe kolo: ",self.left_signed_speed_percent,"  | Prawe kolo: ",self.right_signed_speed_percent)  #testowanie w celu odpowiedniego nastawu predkosci kol 
    def spin_timer_callback(self):                                                                                                        
        """
        Funkcja poruszajaca robotem zaleznie od zadanych przez cmd_vel wartosci
        
        """
        self.get_logger().info(f"Moving >> left_signed_speed_percent={self.left_signed_speed_percent:.2f}, "
                                   f"right_signed_speed_percent={self.right_signed_speed_percent:.2f}")
        self.move()

    def cmd_vel_sub_callback(self, cmd_vel_msg):
        """
        Funkcja wywolywana gdy pojawia sie nowe polecenie od cmd_vel. 
        Ustawia predkosci (-100 < x <100) ktore maja byc przeniesione na kola

        """
        # Aktualizacja czasu
        self.last_received_vel_timestamp_sec = self.get_clock().now().to_msg().sec
		
        # Otrzymanie wartosci predkosci liniowej i obrotowej
        linear = cmd_vel_msg.linear.x
        angular = cmd_vel_msg.angular.z
        self.get_logger().info(f"Received cmd_vel >> "
                               f"linear={cmd_vel_msg.linear.x:.3}, angular={cmd_vel_msg.angular.z:.3}")

        # Ustawia predkosc kol
        self.set_wheels_speed(linear_velocity=linear, angular_rate=angular)

	# Funkcja wywolywana gdy napotkano przeszkode, powoduje zatrzymanie ruchu robota
    """ def obstacles_sub_callback(self, obstacle_msg):
        if obstacle_msg.left_obstacle or obstacle_msg.right_obstacle:
            if not self.braked and not self.obstacle_detected:
                self.obstacle_detected = True
                self.get_logger().warn(f"Braking >> obstacle detected")
                self.brake()
        else:
            self.obstacle_detected = False """
        
def main(args=None):
    
    # Funkcja main wezel motion_driver.py
    
    rclpy.init(args=args)
    gpio_init()

    # Utworz, spin'uj i zniszcz wezel
    
    motion_driver = MotionDriver() 
    rclpy.spin(motion_driver)
    motion_driver.destroy_node()

    gpio_close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

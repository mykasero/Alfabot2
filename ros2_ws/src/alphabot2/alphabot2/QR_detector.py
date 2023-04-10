import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

QR_CODES_TOPIC = "qr_codes"
COMPRESSED_IMAGE_TOPIC = "image_raw/compressed"  

CAMERA_IMAGE_HEIGHT_PX = 240    #wysokosc (w pikselach) obrazu przychodzacego z kamery
QR_MIN_HEIGHT_PX = 130      # Wysokosc kodu QR powinna byc przynajmniej 170px, 
                            # eksperymentalnie zostalo dobrane 130px co pozwala na detekcje z odleglosci okolo 30-35cm


class QRDetector(Node):
    """
    Węzeł odpowiedzialny za analize otrzymanego obrazu i wyszukiwanie w nim QR kodów, w przypadku znalezienia markera,
    wysyłana jest treść kodu QR do tematu qr_codes. W przypadku nie wykrycia QR, wysyłany jest pusty string.
    """
    def __init__(self):
        super().__init__("QR_detector")
        self.get_logger().info("Node init ...")
        # Utworzenie timera wykorzystywanego przez funkcje qr_codes_pub_callback
        self.timer = self.create_timer(0.2, self.qr_codes_pub_callback)
        # Utworzenie subskrybcji tematu image_raw/compressed
        self.compressed_image_sub = self.create_subscription(CompressedImage, COMPRESSED_IMAGE_TOPIC,
                                                             self.compressed_image_sub_callback, 10)
        # Utworzenie publishera do tematu qr_codes
        self.qr_codes_pub = self.create_publisher(String, QR_CODES_TOPIC, 10)
        # Obiekty CVBridge i OpenCV QRCodeDetector 
        self.cv_bridge = CvBridge()
        self.qr_detector = cv2.QRCodeDetector()
        # Status wewnętrzny
        self.last_compressed_image = CompressedImage()  # ostatnia otrzymana wiadomość z tematu image_raw/compressed
        self.qr_content = ""                            # zawartość ostatnio otrzymanego kodu QR
        self.get_logger().info("Node init complete.")

    def compressed_image_sub_callback(self, compressed_image_msg):
        """
        Funkcja odpowiedzialna za odbieranie wiadomości z tematu ,,image_raw/compressed".
        """
        # Update the internal status
        self.last_compressed_image = compressed_image_msg

    def qr_codes_pub_callback(self):
        """
        Funkcja odpowiedzialna za odebranie zawartosci QR z ostatniego otrzymanego obrazu i publikowanie go jako wiadomosc
        typu string w temacie "qr_codes". Jesli nie wykryje QR kodu, opublikuje pusty string
        """
        if  self.last_compressed_image.format:
            self.decode_qr()
        # Utworzenie wiadomosci string
        string_msg = String()
        string_msg.data = self.qr_content
        # Publikacja wiadomosci
        self.qr_codes_pub.publish(string_msg)
        #print("test:    ",string_msg)
        self.get_logger().info(f"Publishing [{QR_CODES_TOPIC}] >> qr={string_msg.data}")

    def decode_qr(self):
        """
        Funkcja wykrywajaca QR kod w ostatnim otrzymanym obrazie, dostaje zawartosc typu string przy uzyciu OpenCV
        Jesli nie wykryto QR kodu, zawartosc bedzie pustym stringiem
        """
        # Resetowanie zawartosci QR
        self.qr_content = ""
        # Konwersja z CompressedImage do obrazu OpenCV przy uzyciu cv_bridge
        cv2_image = self.cv_bridge.compressed_imgmsg_to_cv2(self.last_compressed_image, desired_encoding="passthrough")
        # Detekcja QR i dekodowanie wiadomosci typu string (pierwszy element tablicy)
        qr_content, qr_points, _ = self.qr_detector.detectAndDecode(cv2_image)

        if qr_content:
            # qr_points jest lista list - [[lewy_gorny_rog, prawy_gorny_rog, prawy_dolny_rog, lewy_dolny_rog]]
            # nas interesuje wewnetrzna czesc
            qr_vertices = qr_points[0]

            #Obliczanie rozmiaru krawedzi QR uzywajac Euklidesowego dystansu (norma) 
            # pomiedzy wierzcholkami lewy_gorny_rog a prawy_dolny_rog
            qr_height_px = np.linalg.norm(qr_vertices[0] - qr_vertices[3])

            self.get_logger().info(f"Debugging [{QR_CODES_TOPIC}] >> "
                                   f"qr_vertices={qr_vertices}, "
                                  f"qr_height_px={qr_height_px:.2}")

            # Bierze pod uwage QR tylko wtedy, gdy jest wystarczajaco duzy (tzn. robot jest blisko QR)
            if qr_height_px > QR_MIN_HEIGHT_PX:
                self.qr_content = qr_content


def main(args=None):
    """
    Main function of the QR_detector node.
    """
    np.set_printoptions(suppress=True)  # Prevent numpy scientific notation
    rclpy.init(args=args)

    qr_detector = QRDetector()
    rclpy.spin(qr_detector)
    qr_detector.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()


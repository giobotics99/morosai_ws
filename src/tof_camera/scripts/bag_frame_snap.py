#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys

class BagFrameSaver(Node):
    def __init__(self, topic):
        super().__init__('bag_frame_saver')
        self.bridge = CvBridge()
        self.current_image = None
        self.frame_count = 0

        # Subscriber al topic immagine
        self.sub = self.create_subscription(
            Image,
            topic,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Ascoltando {topic}. Premi 's' per salvare il frame corrente, 'q' per uscire.")

    def image_callback(self, msg):
        # Converte ROS Image -> OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.current_image = cv_image
        cv2.imshow("BGR Feed", cv_image)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):  # salva il frame corrente
            filename = f"frame_{self.frame_count}.png"
            cv2.imwrite(filename, self.current_image)
            self.get_logger().info(f"Frame salvato come {filename}")
            self.frame_count += 1
        elif key == ord('q'):  # esce
            self.get_logger().info("Chiusura programma...")
            rclpy.shutdown()

def main(args=None):
    if len(sys.argv) < 2:
        print("Uso: python3 bag_frame_snap.py <topic>")
        return

    topic = sys.argv[1]

    rclpy.init(args=args)
    node = BagFrameSaver(topic)
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
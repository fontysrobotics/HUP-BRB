import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import math
import time
from std_msgs.msg import String, Header
from hupbrb_msgs.msg import Identifier
import numpy as np 
import cv2
import time

class Scanner(Node):

    detector = cv2.QRCodeDetector()

    def __init__(self):
        super().__init__('hupbrb_id_scanner')
        self.subscription = self.create_subscription(
        	Image,
        	'camera/image_raw',
        	self.camera_input,
            qos_profile_sensor_data,
            )

        self.srv = self.create_publisher(Identifier, 'scanned_ids', 10)

    def camera_input(self, message: Image):
        # Transforming flat array into shape (height, width, RGB)``
        data = np.array(message.data).reshape(message.height, message.width, 3)
        
        # Try to detect a QR code
        try:
            qr_text,bbox,straight_qrcode = self.detector.detectAndDecode(data)  
            self.get_logger().info(f"qt text = {qr_text}")

            if qr_text:
                # Publish the value of the QR code 
                # or an empty string if no QR code is detected
                msg = Identifier()
                msg.name = qr_text
                self.srv.publish(msg)

        except cv2.error:
            self.get_logger().warn("Something went wrong with cv2")

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = Scanner()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
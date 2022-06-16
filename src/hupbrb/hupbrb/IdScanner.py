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

        inf = self.get_publishers_info_by_topic('camera/image_raw')
        if not inf:
            self.get_logger().info("No publishers on camera/image_raw. Opening hardware camera.")
            self.camera = cv2.VideoCapture()
            self.camera.open(0, cv2.CAP_ANY)
            if not self.camera.isOpened():
                self.get_logger().error("Camera topic does not exist, and cannot open physical camera")
            self.cam_publisher = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
            self.create_timer(0.04, self.publish_frame)

        self.srv = self.create_publisher(Identifier, 'scanned_ids', 10)

    def publish_frame(self):
        _ , frame = self.camera.read()
        ros_image = Image()
        ros_image.header = Header()
        ros_image.height, ros_image.width, elemsize = frame.shape
        ros_image.encoding = "bgr8"
        ros_image.is_bigendian = False
        ros_image.step = ros_image.width * elemsize
        size = ros_image.step * ros_image.height
        # ros_image.data.resize(size)
        ros_image.data = bytes(frame.flatten())

        self.cam_publisher.publish(ros_image)

    def camera_input(self, message: Image):
        self.get_logger().debug("Received camera input")

        # Transforming flat array into shape (height, width, RGB)
        data = np.array(message.data).reshape(message.height, -1, 3)

        # Try to detect a QR code
        try:
            qr_text,bbox,straight_qrcode = self.detector.detectAndDecode(data)  

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
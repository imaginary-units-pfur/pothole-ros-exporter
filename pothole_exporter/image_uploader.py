import rclpy
from rclpy.node import Node
import requests
import time

from sensor_msgs.msg import Image, NavSatFix

class ImageUploader(Node):
    def __init__(self):
        super().__init__('image_uploader')
        self.img_subscription = self.create_subscription(
            Image,
            '/OAK_D_back/color/image',
            self.img_listener_callback,
            10)

        self.gps_subscription = self.create_subscription(
            NavSatFix,
            '/imu_sensor/imu/nav_sat_fix',
            self.gps_listener_callback,
            10)
        self.image = None
        self.gps = None
        self.last_send = 0

    def img_listener_callback(self, msg):
        self.get_logger().info("Image received!")
        self.image = msg
        self.maybe_send()
    
    def gps_listener_callback(self, msg):
        self.get_logger().info("GPS received!")
        self.gps = gps
        self.maybe_send()

    def maybe_send(self):
        if time.time() > self.last_send+10:
            if self.image is None or self.gps is None:
                self.get_logger().info("Missing some data, not sending yet")
                return
            self.get_logger().info("Sending data!")
            ...
            self.last_send = time.time()
            self.image = None
            self.gps = None


def main(args=None):
    rclpy.init(args=args)
    uploader = ImageUploader()
    rclpy.spin(uploader)

    uploader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

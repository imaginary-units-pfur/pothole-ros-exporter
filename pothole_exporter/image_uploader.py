import rclpy
from rclpy.node import Node
import requests
import time
import gzip
import base64

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
        self.gps = msg
        self.maybe_send()

    def maybe_send(self):
        if time.time() > self.last_send+10:
            if self.image is None or self.gps is None:
                self.get_logger().info("Missing some data, not sending yet")
                return
            self.get_logger().info("Sending data!")
            lat = self.gps.latitude
            lon = self.gps.longitude
            data = bytes(self.image.data)
            shape = (self.image.width, self.image.height)
            jdata = {
                'latitude': lat, 'longitude': lon,
                'image_shape': shape, 'image_encoding': self.image.encoding,
                'image_bigendian': self.image.is_bigendian,
                'image_step': self.image.step,
                'image_data_gzip_base64': base64.b64encode(gzip.compress(data)).decode()
            }
            requests.post('http://10.0.0.128:8080/analyze', json=jdata)
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

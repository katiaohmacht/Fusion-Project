# yolo_image_stamper.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class YoloImageStamper(Node):
    def __init__(self):
        super().__init__('yolo_image_stamper')
        self.sub = self.create_subscription(Image, '/yolov8_processed_image', self.cb, 10)
        self.pub = self.create_publisher(Image, '/yolov8_processed_image_stamped', 10)
        self.frame_id = self.declare_parameter('frame_id', 'default_cam').value

    def cb(self, msg: Image):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()
        if not msg.header.frame_id:
            msg.header.frame_id = self.frame_id
        self.pub.publish(msg)

def main():
    rclpy.init()
    n = YoloImageStamper()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



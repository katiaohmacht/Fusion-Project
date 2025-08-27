# yolov_ros/yolov_ros/yolo_node.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # topic from usb_cam
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # smallest YOLOv8 model
        self.pub = self.create_publisher(Image, '/yolo/image_raw', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(cv_image)[0]  # run detection
        annotated_frame = results.plot()
        annotated_frame = annotated_frame.astype('uint8')
        # cv2.imshow("YOLO Detection", annotated_frame)
        # cv2.waitKey(1)
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.pub.publish(annotated_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally: 
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




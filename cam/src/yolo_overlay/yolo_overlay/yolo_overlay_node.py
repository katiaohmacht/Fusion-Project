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
        annotated_msg.header.stamp = msg.header.stamp
        self.pub.publish(annotated_msg)
        for r in results:
            boxes = r.boxes  # Access the Boxes object
            for box in boxes:
                # Get bounding box coordinates in xyxy format (top-left x, top-left y, bottom-right x, bottom-right y)
                x1, y1, x2, y2 = box.xyxy[0].tolist() 
            
                # Get bounding box coordinates in xywh format (center x, center y, width, height)
                # x_center, y_center, width, height = box.xywh[0].tolist()

                confidence = box.conf[0].item() # Confidence score
                class_id = box.cls[0].item()    # Class ID
            
                print(f"Box: ({x1}, {y1}, {x2}, {y2}), Confidence: {confidence:.2f}, Class ID: {int(class_id)}")

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




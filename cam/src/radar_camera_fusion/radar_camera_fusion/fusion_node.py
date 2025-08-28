#!/usr/bin/env python3

import rclpy
import time 
import cv2
import numpy as np
import tf2_geometry_msgs
import sensor_msgs_py.point_cloud2 as pc2

from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped
from message_filters import Subscriber, ApproximateTimeSynchronizer
from collections import deque    


class RadarCameraOverlay(Node):
   def __init__(self):
       super().__init__('radar_camera_overlay')
      
       self.bridge = CvBridge()
       self.tf_buffer = Buffer()
       self.tf_listener = TransformListener(self.tf_buffer, self)

       self.dist_coeffs = None
       self.image_width = 640
       self.image_height = 480

       self.camera_matrix = np.array([
           [600, 0, -700],   
           [0, 600, 100],
           [0, 0, 1]        
       ], dtype=np.float32)
      
       self.dist_coeffs = np.zeros((4, 1)) 
      
       self.image_sub = Subscriber(self, Image, '/yolo/image_raw')
       self.radar_sub = Subscriber(self, PointCloud2, '/ti_mmwave/radar_scan_pcl')
      
       self.camera_info_sub = self.create_subscription(
           CameraInfo,
           '/camera_info',
           self.camera_info_callback,
           10
       )
      
       self.ts = ApproximateTimeSynchronizer(
           [self.image_sub, self.radar_sub],
           queue_size=200,
           slop=0.1 
       )
       self.ts.registerCallback(self.sync_callback)
      
       self.overlay_pub = self.create_publisher(Image, '/radar_camera_overlay', 10)
       self.declare_parameter('camera_frame', 'camera_link')
       self.declare_parameter('radar_frame', 'radar_link')
       self.declare_parameter('point_size', 3)
       self.declare_parameter('point_color_r', 0)
       self.declare_parameter('point_color_g', 255)
       self.declare_parameter('point_color_b', 0)
       self.declare_parameter('max_distance', 50.0) 
       self.declare_parameter('linger_time', 0.3)   

       self.point_history = deque(maxlen=2000)
       self.get_logger().info('Radar-Camera Overlay node initialized')
  
   def camera_info_callback(self, msg):
       if self.camera_matrix is None:
           self.camera_matrix = np.array(msg.k).reshape(3, 3)
           self.dist_coeffs = np.array(msg.d)
           self.image_width = msg.width
           self.image_height = msg.height
           self.get_logger().info('Camera parameters updated from camera_info')
  
   def sync_callback(self, image_msg, radar_msg):
       """Process synchronized image and radar data"""
       try:
           cv_image = self.bridge.imgmsg_to_cv2(image_msg, 'bgr8')
           radar_points_camera = self.transform_radar_to_camera(radar_msg)
           if radar_points_camera is not None and len(radar_points_camera) > 0:
               current_time = time.time()
               for p in radar_points_camera:
                   self.point_history.append((p, current_time))
           overlay_image = self.project_and_draw(cv_image)
           overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, 'bgr8')
           overlay_msg.header = image_msg.header
           self.overlay_pub.publish(overlay_msg)
       except Exception as e:
           self.get_logger().error(f'Error in sync_callback: {str(e)}')
  
   def transform_radar_to_camera(self, radar_msg):
       """Transform radar points to camera frame"""
       try:
           camera_frame = self.get_parameter('camera_frame').value
           radar_frame = self.get_parameter('radar_frame').value
           transform = self.tf_buffer.lookup_transform(
               camera_frame,
               radar_frame,
               rclpy.time.Time()
           )
          
           points_3d = []
           for point in pc2.read_points(radar_msg, field_names=("x", "y", "z"), skip_nans=True):
               distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
               if distance <= self.get_parameter('max_distance').value:
                   point_radar = PointStamped()
                   point_radar.header.frame_id = radar_frame
                   point_radar.header.stamp = radar_msg.header.stamp
                   point_radar.point.x = float(point[0])
                   point_radar.point.y = float(point[1])
                   point_radar.point.z = float(point[2])
                  
                   point_camera = tf2_geometry_msgs.do_transform_point(point_radar, transform)
                   points_3d.append([
                       point_camera.point.x,
                       point_camera.point.y,
                       point_camera.point.z
                   ])
           return np.array(points_3d) if points_3d else None
          
       except Exception as e:
           self.get_logger().warn(f'Transform failed: {str(e)}')
           return None
  
   def project_and_draw(self, image):
       """Project lingering 3D points to 2D and draw on image"""
       if not self.point_history:
           return image

       current_time = time.time()
       linger_time = self.get_parameter('linger_time').value

       valid_points = [p for p, t in self.point_history if current_time - t <= linger_time]
       if not valid_points:
           return image

       points_3d = np.array(valid_points)
       if len(points_3d) == 0:
           return image
      
       points_2d, _ = cv2.projectPoints(
           points_3d,
           np.zeros(3), 
           np.zeros(3),
           self.camera_matrix,
           self.dist_coeffs)

       points_2d = points_2d.reshape(-1, 2)
       overlay_image = image.copy()
       point_size = self.get_parameter('point_size').value
       color = (
           self.get_parameter('point_color_b').value,
           self.get_parameter('point_color_g').value,
           self.get_parameter('point_color_r').value)
      
       for idx, point in enumerate(points_2d):
           x, y = int(point[0]), int(point[1])
           if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
               cv2.circle(overlay_image, (x, y), point_size, color, -1)
               distance = np.sqrt(np.sum(points_3d[idx]**2))
               cv2.putText(overlay_image, f'{distance:.1f}m', (x+5, y-5),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.3, color, 1)
       return overlay_image

def main(args=None):
   rclpy.init(args=args)
   node = RadarCameraOverlay()
   try:
       rclpy.spin(node)
   except KeyboardInterrupt:
       pass
   finally:
       node.destroy_node()
       rclpy.shutdown()


if __name__ == '__main__':
   main()




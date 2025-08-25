#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import sensor_msgs_py.point_cloud2 as pc2
from message_filters import Subscriber, ApproximateTimeSynchronizer

class RadarCameraOverlayDebug(Node):
    def __init__(self):
        super().__init__('radar_camera_overlay_debug')
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Camera parameters - more generous defaults for USB cameras
        self.camera_matrix = None
        self.dist_coeffs = None
        self.image_width = 640
        self.image_height = 480
        
        # More typical USB camera matrix (much wider FOV)
        self.camera_matrix = np.array([
            [300, 0, 320],    # Much lower fx for typical USB camera
            [0, 300, 240],    # Much lower fy for typical USB camera  
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.dist_coeffs = np.zeros((4, 1))
        
        # Parameters (declare first)
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('radar_topic', '/ti_mmwave/radar_scan_pcl')
        self.declare_parameter('camera_info_topic', '/camera_info')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('radar_frame', 'radar_link')
        self.declare_parameter('point_size', 5)
        self.declare_parameter('max_distance', 100.0)
        self.declare_parameter('min_distance', 0.1)
        self.declare_parameter('debug_mode', True)
        self.declare_parameter('use_transform', True)
        self.declare_parameter('manual_fx', 300.0)  # Manual camera calibration
        self.declare_parameter('manual_fy', 300.0)
        self.declare_parameter('manual_cx', 320.0)
        self.declare_parameter('manual_cy', 240.0)
        
        # Get topic names from parameters
        image_topic = self.get_parameter('image_topic').value
        radar_topic = self.get_parameter('radar_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        
        # Subscribers with individual callbacks for debugging
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.radar_sub = self.create_subscription(PointCloud2, radar_topic, self.radar_callback, 10)
        
        # Check if topics exist
        self.create_timer(2.0, self.check_topics)
        
        # Store latest messages for debugging
        self.latest_image = None
        self.latest_radar = None
        self.image_count = 0
        self.radar_count = 0
        
        # Optional: Subscribe to camera info
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)
        
        # Publishers
        self.overlay_pub = self.create_publisher(Image, '/radar_camera_overlay', 10)
        self.debug_pub = self.create_publisher(Image, '/radar_debug_image', 10)
        
        # Timer for processing
        self.timer = self.create_timer(0.1, self.process_data)  # 10 Hz
        
        # Update camera matrix from parameters
        self.update_camera_matrix()
        
        self.get_logger().info('Debug Radar-Camera Overlay node initialized')
        self.get_logger().info(f'Subscribing to: {image_topic}, {radar_topic}')
        self.get_logger().info('Publishing debug info to /radar_debug_image')
    
    def update_camera_matrix(self):
        """Update camera matrix from manual parameters"""
        fx = self.get_parameter('manual_fx').value
        fy = self.get_parameter('manual_fy').value
        cx = self.get_parameter('manual_cx').value
        cy = self.get_parameter('manual_cy').value
        
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float32)
        
        self.get_logger().info(f'Using manual camera matrix: fx={fx}, fy={fy}, cx={cx}, cy={cy}')
    
    def check_topics(self):
        """Check if our subscribed topics are actually publishing"""
        topic_names_and_types = self.get_topic_names_and_types()
        available_topics = [name for name, _ in topic_names_and_types]
        
        image_topic = self.get_parameter('image_topic').value
        radar_topic = self.get_parameter('radar_topic').value

    def camera_info_callback(self, msg):
        """Update camera parameters from camera_info topic"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.image_width = msg.width
        self.image_height = msg.height
        
    def image_callback(self, msg):
        self.latest_image = msg
        self.image_width = msg.width
        self.image_height = msg.height
        self.image_count += 1
        
        # if self.image_count % 30 == 1:  # Log every 30 frames
            #self.get_logger().info(f'Received image {self.image_count}: {msg.width}x{msg.height}, frame: {msg.header.frame_id}')
    
    def radar_callback(self, msg):
        self.latest_radar = msg
        self.radar_count += 1
        
        # Debug: Print radar data info
        if self.get_parameter('debug_mode').value:
            point_count = 0
            points_info = []
            
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                point_count += 1
                if point_count <= 5:  # Show first 5 points
                    distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                    points_info.append(f"({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}, d={distance:.2f})")
                    self.get_logger().info(f'POINTS INFO: ({point[0]:.2f}, {point[1]:.2f}, {point[2]:.2f}, d={distance:.2f})')
            
            # if self.radar_count % 10 == 1:  # Log every 10 frames
            #     self.get_logger().info(f'Radar {self.radar_count}: {point_count} points, frame: {msg.header.frame_id}')
            #     if points_info:
            #         self.get_logger().info(f'  First points: {", ".join(points_info)}')
    
    def process_data(self):
        """Process latest image and radar data"""
        if self.latest_image is None or self.latest_radar is None:
            return
        
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, 'bgr8')
            
            # Process radar points
            if self.get_parameter('use_transform').value:
                radar_points_camera = self.transform_radar_to_camera(self.latest_radar)
            else:
                radar_points_camera = self.get_radar_points_direct(self.latest_radar)
            
            # Create debug visualization
            debug_image = self.create_debug_visualization(cv_image, radar_points_camera)
            
            # Create overlay
            if radar_points_camera is not None and len(radar_points_camera) > 0:
                overlay_image = self.project_and_draw(cv_image.copy(), radar_points_camera)
            else:
                overlay_image = cv_image
            
            # Publish both images
            overlay_msg = self.bridge.cv2_to_imgmsg(overlay_image, 'bgr8')
            overlay_msg.header = self.latest_image.header
            self.overlay_pub.publish(overlay_msg)
            
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            debug_msg.header = self.latest_image.header
            self.debug_pub.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in process_data: {str(e)}')
    
    def get_radar_points_direct(self, radar_msg):
        """Get radar points without transform (for debugging)"""
        points_3d = []
        max_dist = self.get_parameter('max_distance').value
        min_dist = self.get_parameter('min_distance').value
        
        for point in pc2.read_points(radar_msg, field_names=("x", "y", "z"), skip_nans=True):
            distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            if min_dist <= distance <= max_dist:
                points_3d.append([float(point[0]), float(point[1]), float(point[2])])
        
        # if self.get_parameter('debug_mode').value and len(points_3d) > 0:
        #     self.get_logger().info(f'Direct radar points: {len(points_3d)} points in range')
        
        return np.array(points_3d) if points_3d else None
    
    def transform_radar_to_camera(self, radar_msg):
        """Transform radar points to camera frame with debug info"""
        try:
            camera_frame = self.get_parameter('camera_frame').value
            radar_frame = self.get_parameter('radar_frame').value
            
            # Get transform
            transform = self.tf_buffer.lookup_transform(
                camera_frame, radar_frame, rclpy.time.Time())
            
            if self.get_parameter('debug_mode').value:
                t = transform.transform.translation
                r = transform.transform.rotation
                # self.get_logger().info(
                #     f'Transform {radar_frame}->{camera_frame}: '
                #     f'pos=({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) '
                #     f'rot=({r.x:.2f}, {r.y:.2f}, {r.z:.2f}, {r.w:.2f})'
                # )
            
            points_3d = []
            max_dist = self.get_parameter('max_distance').value
            min_dist = self.get_parameter('min_distance').value
            
            for point in pc2.read_points(radar_msg, field_names=("x", "y", "z"), skip_nans=True):
                distance = np.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
                if min_dist <= distance <= max_dist:
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
            
            # if self.get_parameter('debug_mode').value and len(points_3d) > 0:
            #     # self.get_logger().info(f'Transformed points: {len(points_3d)} points')
            #     # Show first few transformed points
            #     for i, p in enumerate(points_3d[:3]):
            #         self.get_logger().info(f'  Point {i}: ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f})')
            
            return np.array(points_3d) if points_3d else None
            
        except Exception as e:
            self.get_logger().warn(f'Transform failed: {str(e)}')
            return None
    
    def create_debug_visualization(self, image, points_3d):
        """Create debug image with lots of info"""
        debug_image = image.copy()
        
        # Add text info
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_offset = 30
        
        # Camera info
        cv2.putText(debug_image, f'Camera: {self.image_width}x{self.image_height}', 
                   (10, y_offset), font, 0.7, (255, 255, 255), 2)
        y_offset += 25
        
        if self.camera_matrix is not None:
            fx, fy = self.camera_matrix[0,0], self.camera_matrix[1,1]
            cx, cy = self.camera_matrix[0,2], self.camera_matrix[1,2]
            cv2.putText(debug_image, f'fx={fx:.0f}, fy={fy:.0f}, cx={cx:.0f}, cy={cy:.0f}', 
                       (10, y_offset), font, 0.6, (255, 255, 255), 2)
            y_offset += 25
        
        # Point count
        if points_3d is not None:
            cv2.putText(debug_image, f'Radar points: {len(points_3d)}', 
                       (10, y_offset), font, 0.7, (0, 255, 0), 2)
            y_offset += 25
            
            # Show point positions
            front_points = points_3d[points_3d[:, 2] > 0] if len(points_3d) > 0 else []
            cv2.putText(debug_image, f'Points in front: {len(front_points)}', 
                       (10, y_offset), font, 0.7, (0, 255, 255), 2)
            y_offset += 25
            
            # Show some point coordinates
            for i, point in enumerate(points_3d[:3]):
                cv2.putText(debug_image, f'P{i}: ({point[0]:.1f}, {point[1]:.1f}, {point[2]:.1f})', 
                           (10, y_offset), font, 0.5, (100, 255, 100), 1)
                y_offset += 20
        else:
            cv2.putText(debug_image, 'No radar points', (10, y_offset), font, 0.7, (0, 0, 255), 2)
        
        # Draw crosshairs at image center
        center_x, center_y = self.image_width // 2, self.image_height // 2
        cv2.line(debug_image, (center_x-20, center_y), (center_x+20, center_y), (255, 0, 255), 2)
        cv2.line(debug_image, (center_x, center_y-20), (center_x, center_y+20), (255, 0, 255), 2)
        
        return debug_image
    
    def project_and_draw(self, image, points_3d):
        """Enhanced projection with debug info"""
        if points_3d is None or len(points_3d) == 0:
            return image
        
        # Filter points in front of camera
        front_mask = points_3d[:, 2] > 0
        if not np.any(front_mask):
            # if self.get_parameter('debug_mode').value:
            #     self.get_logger().warn('No points in front of camera (z > 0)')
            return image
        
        points_3d_front = points_3d[front_mask]
        
        # if self.get_parameter('debug_mode').value:
        #     self.get_logger().info(f'Projecting {len(points_3d_front)} points in front of camera')
        
        # Project to 2D
        try:
            points_2d, _ = cv2.projectPoints(
                points_3d_front,
                np.zeros(3), np.zeros(3),
                self.camera_matrix,
                self.dist_coeffs
            )
            points_2d = points_2d.reshape(-1, 2)
            
            # Draw ALL projected points, even if outside image
            point_size = self.get_parameter('point_size').value
            
            for i, (point_2d, point_3d) in enumerate(zip(points_2d, points_3d_front)):
                x, y = int(point_2d[0]), int(point_2d[1])
                distance = np.sqrt(np.sum(point_3d**2))
                
                # Use different colors for points inside/outside image bounds
                if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                    color = (0, 255, 0)  # Green for visible points
                    cv2.circle(image, (x, y), point_size, color, -1)
                    cv2.putText(image, f'{distance:.1f}m', (x+5, y-5),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                else:
                    # Draw indicators for points outside image bounds
                    color = (0, 165, 255)  # Orange for outside points
                    # Clamp to image bounds for visualization
                    x_clamped = max(0, min(x, image.shape[1]-1))
                    y_clamped = max(0, min(y, image.shape[0]-1))
                    cv2.circle(image, (x_clamped, y_clamped), point_size//2, color, 2)
                    
                    # if self.get_parameter('debug_mode').value and i < 5:
                    #     self.get_logger().info(f'Point {i} outside image: projected to ({x}, {y}), clamped to ({x_clamped}, {y_clamped})')
            
        except Exception as e:
            self.get_logger().error(f'Projection failed: {str(e)}')
        
        return image


def main(args=None):
    rclpy.init(args=args)
    node = RadarCameraOverlayDebug()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()



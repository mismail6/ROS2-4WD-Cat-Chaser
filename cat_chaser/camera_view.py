import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Initialize YOLO model
        self.get_logger().info('Loading YOLOv8 model...')
        self.model = YOLO('yolov8s.pt')  # Using small model for good balance
        self.get_logger().info('YOLOv8 model loaded successfully!')
        
        # Subscribe to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera',
            self.image_callback,
            10
        )
        
        # Subscribe to LiDAR topic
        self.lidar_subscription = self.create_subscription(
            LaserScan,
            '/lidar',
            self.lidar_callback,
            10
        )
        
        # Publisher for cmd_vel topic
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Store the latest LiDAR scan
        self.latest_scan = None
        
        # Field of view of the camera (in degrees, approximate)
        self.camera_fov = 60.0
        
        self.get_logger().info('Camera viewer node started. Press "q" to quit.')
        
        # Create OpenCV window
        cv2.namedWindow('Robot Camera Feed with Detection', cv2.WINDOW_AUTOSIZE)
        
    def lidar_callback(self, msg):
        """Store the latest LiDAR scan"""
        self.latest_scan = msg
        
    def get_distance_to_object(self, angle_ratio):
        """
        Get distance to object using LiDAR data
        angle_ratio: position of object in camera view (0.0 to 1.0, from left to right)
        """
        if self.latest_scan is None:
            return None
            
        # Convert camera position (0-1) to LiDAR angle
        # Map the camera FOV to the corresponding LiDAR angles
        # Assuming the camera is centered with the LiDAR
        angle_in_camera = (angle_ratio - 0.5) * self.camera_fov
        
        # Convert to radians and adjust to LiDAR frame
        # LiDAR typically has 0 degrees at the front and increases counterclockwise
        angle_in_lidar_rad = math.radians(-angle_in_camera)
        
        # Find the corresponding index in the LiDAR scan
        angle_min = self.latest_scan.angle_min
        angle_max = self.latest_scan.angle_max
        angle_increment = self.latest_scan.angle_increment
        
        # Ensure angle is within LiDAR range
        if angle_in_lidar_rad < angle_min:
            angle_in_lidar_rad = angle_min
        elif angle_in_lidar_rad > angle_max:
            angle_in_lidar_rad = angle_max
            
        # Calculate the index
        index = int((angle_in_lidar_rad - angle_min) / angle_increment)
        
        # Get distance from LiDAR data, considering a small window to handle noise
        window_size = 5
        start_idx = max(0, index - window_size // 2)
        end_idx = min(len(self.latest_scan.ranges), index + window_size // 2 + 1)
        
        # Filter out invalid readings (typically represented as inf or 0)
        valid_ranges = [r for r in self.latest_scan.ranges[start_idx:end_idx] 
                        if r > self.latest_scan.range_min and r < self.latest_scan.range_max]
        
        if valid_ranges:
            # Return the minimum valid distance in the window (closest obstacle)
            return min(valid_ranges)
        else:
            return None
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model(cv_image, verbose=False)
            
            # Process detections
            annotated_image = cv_image.copy()
            
            # Initialize movement command
            twist = Twist()
            cat_detected = False
            
            # Get image dimensions for calculating center
            image_height, image_width = cv_image.shape[:2]
            image_center_x = image_width // 2
            
            for result in results:
                boxes = result.boxes
                if boxes is not None:
                    for box in boxes:
                        # Get class ID and confidence
                        class_id = int(box.cls[0])
                        confidence = float(box.conf[0])
                        class_name = self.model.names[class_id]

                        cat_name = self.model.names[15]
                        
                        # Debug: print all detections
                        self.get_logger().info(f'Detected: {class_name} (ID: {class_id}) with confidence: {confidence:.2f}')
                        
                        # Check if it's a cat, dog, horse, or other animals (common misclassifications for simple shapes)
                        animal_classes = [15, 16, 17, 18, 19, 20]  # cat, dog, horse, sheep, cow, elephant
                        if class_id in animal_classes and confidence > 0.25:
                            cat_detected = True
                            
                            # Get bounding box coordinates
                            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                            
                            # Calculate center of bounding box
                            box_center_x = (x1 + x2) // 2
                            box_center_y = (y1 + y2) // 2
                            
                            # Calculate box area (for distance estimation)
                            box_area = (x2 - x1) * (y2 - y1)
                            
                            # Calculate position ratio for LiDAR angle mapping
                            position_ratio = box_center_x / image_width
                            
                            # Get distance from LiDAR
                            lidar_distance = self.get_distance_to_object(position_ratio)
                            
                            # Draw bounding box
                            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            
                            # Draw center point
                            cv2.circle(annotated_image, (box_center_x, box_center_y), 5, (255, 0, 0), -1)
                            
                            # Draw label with actual detected class and distance
                            distance_text = f"Distance: {lidar_distance:.2f}m" if lidar_distance is not None else "Distance: Unknown"
                            label = f'{cat_name}: {confidence:.2f} - {distance_text}'
                            
                            # Add distance to the label
                            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                            cv2.rectangle(annotated_image, (x1, y1 - label_size[1] - 10), 
                                        (x1 + label_size[0], y1), (0, 255, 0), -1)
                            cv2.putText(annotated_image, label, (x1, y1 - 5), 
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                            
                            # Display distance prominently at the top of the screen
                            if lidar_distance is not None:
                                top_label = f"Cat Distance: {lidar_distance:.2f} meters"
                                cv2.putText(annotated_image, top_label, (10, 30), 
                                          cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
                            
                            # Calculate movement commands
                            # Angular velocity based on horizontal offset from center
                            x_offset = box_center_x - image_center_x
                            angular_z = -x_offset / (image_width / 2) * 0.5  # Scale factor for turning speed
                            
                            # Linear velocity based on LiDAR distance if available, otherwise use box size
                            if lidar_distance is not None:
                                if lidar_distance > 2.0:  # If cat is far
                                    linear_x = 1.5
                                elif lidar_distance > 1.0:  # Medium distance
                                    linear_x = 1.2
                                elif lidar_distance > 0.5:  # Close to cat
                                    linear_x = 0.5
                                else:  # Very close - stop
                                    linear_x = 0.0
                            else:
                                # Fallback to using box size
                                normalized_area = box_area / (image_width * image_height)
                                if normalized_area < 0.1:  # If cat is small (far away)
                                    linear_x = 1.5
                                elif normalized_area < 0.3:  # Medium distance
                                    linear_x = 1.2
                                else:  # Close to cat
                                    linear_x = 0.5
                            
                            # Set twist values
                            twist.linear.x = linear_x
                            twist.angular.z = angular_z
                            
                            # Log detection and movement
                            distance_info = f", Distance: {lidar_distance:.2f}m" if lidar_distance is not None else ""
                            self.get_logger().info(f'{class_name} detected! Moving towards it{distance_info}. '
                                                 f'Linear: {linear_x:.2f}, Angular: {angular_z:.2f}')
                            
                            # Only process the first detected cat to avoid conflicting commands
                            break
                    
                    if cat_detected:
                        break
            
            # If no cat detected, stop the robot
            if not cat_detected:
                twist.linear.x = 0.1
                twist.angular.z = 0.0
            
            # Publish movement command
            self.cmd_vel_publisher.publish(twist)
            
            # Draw image center line for reference
            #cv2.line(annotated_image, (image_center_x, 0), (image_center_x, image_height), (255, 255, 255), 1)
            
            # Display the annotated image
            cv2.imshow('Robot Camera Feed with Detection', annotated_image)
            
            # Check for 'q' key press to quit
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self.get_logger().info('Quitting camera viewer...')
                # Stop the robot before quitting
                stop_twist = Twist()
                self.cmd_vel_publisher.publish(stop_twist)
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_viewer = CameraViewer()
        rclpy.spin(camera_viewer)
    except KeyboardInterrupt:
        # Stop the robot on keyboard interrupt
        stop_twist = Twist()
        if 'camera_viewer' in locals():
            camera_viewer.cmd_vel_publisher.publish(stop_twist)
    finally:
        # Clean up
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class CameraDetector(Node):
    def __init__(self):
        super().__init__('camera_detector')
        
        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.3)  # LOWERED from 0.5
        self.declare_parameter('publish_annotated', True)
        
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.publish_annotated = self.get_parameter('publish_annotated').value
        
        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)
        self.get_logger().info(f'YOLO model loaded. Confidence threshold: {self.conf_threshold}')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Frame counter for debugging
        self.frame_count = 0
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/camera/detections',
            10
        )
        
        if self.publish_annotated:
            self.annotated_pub = self.create_publisher(
                Image,
                '/camera/annotated_image',
                10
            )
        
        self.get_logger().info('Camera detector initialized')
        self.get_logger().info('YOLO can detect these classes: person, car, truck, bus, motorcycle, bicycle, etc.')
    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            self.frame_count += 1
            
            # Run detection with verbose output every 30 frames
            verbose = (self.frame_count % 30 == 0)
            results = self.model(cv_image, conf=self.conf_threshold, verbose=verbose)
            
            # Create Detection2DArray message
            detection_array = Detection2DArray()
            detection_array.header = msg.header
            
            # Process detections
            total_detections = 0
            for result in results:
                boxes = result.boxes
                total_detections += len(boxes)
                
                for box in boxes:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    conf = float(box.conf[0])
                    cls = int(box.cls[0])
                    class_name = self.model.names[cls]
                    
                    # Create Detection2D message
                    detection = Detection2D()
                    detection.header = msg.header
                    
                    # Bounding box
                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)
                    
                    # Hypothesis
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = class_name
                    hypothesis.hypothesis.score = conf
                    detection.results.append(hypothesis)
                    
                    detection_array.detections.append(detection)
                    
                    # Draw on image with thicker lines
                    if self.publish_annotated:
                        cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 3)
                        label = f'{class_name} {conf:.2f}'
                        cv2.putText(cv_image, label, (int(x1), int(y1) - 10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add debug info on image
            if self.publish_annotated:
                debug_text = f'Frame: {self.frame_count} | Detections: {total_detections} | Conf: {self.conf_threshold}'
                cv2.putText(cv_image, debug_text, (10, 30),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            # Publish detections
            self.detection_pub.publish(detection_array)
            
            # Publish annotated image
            if self.publish_annotated:
                annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                annotated_msg.header = msg.header
                self.annotated_pub.publish(annotated_msg)
            
            # Log detection info
            if total_detections > 0:
                self.get_logger().info(f'Frame {self.frame_count}: Detected {total_detections} objects')
            elif self.frame_count % 30 == 0:
                self.get_logger().warn(f'Frame {self.frame_count}: No detections found')
        
        except Exception as e:
            self.get_logger().error(f'Error in detection: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

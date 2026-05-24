#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rover_interfaces.msg import PerceptionResult 
from rover_interfaces.msg import PerceptionResultArray
from ultralytics import YOLO
import cv2
import os
from ament_index_python.packages import get_package_share_directory
from collections import deque
import numpy as np
from sensor_msgs.msg import LaserScan
import math



class PerceptionNode(Node): 
    def __init__(self):
        super().__init__("perception_node")
        self.subscriber_ = self.create_subscription(Image,"/image_raw",
                            self.image_callback,qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(PerceptionResultArray, "/perception/obstacle_info", 10)
        self.annotated_pub_ = self.create_publisher(Image, "/perception/annotated_image", 10)
        self.cv_bridge_ = CvBridge()
        self.pkg_path_ = get_package_share_directory('rover_perception_pkg')
        self.model_path_ = os.path.join(self.pkg_path_,'weights','sim_to_real_alpha_best.pt')
        #self.model_ = YOLO("yolov8n.pt")
        self.model_ = YOLO(self.model_path_)
        self.detection_buffer = deque(maxlen=5) 
        self.is_rock_active = False
        #<width>1280</width><height>720</height>
        # self.roi_x_min = 400
        # self.roi_x_max = 880
        # self.roi_y_min = 240
        # self.roi_y_max = 480
        self.roi_polygon = np.array([
            [100, 650], 
            [450, 240], 
            [830, 240], 
            [1180, 650]
        ], np.int32)
        #Adding lidar to the perception to determine the distance
        self.latest_scan = None
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0
        self.scan_sub_ = self.create_subscription(
            LaserScan, 
            "/scan", 
            self.scan_callback, 
            qos_profile_sensor_data
        )

    def image_callback(self, msg:Image):
        #self.get_logger().info("Image data passed")
        cv_image = self.cv_bridge_.imgmsg_to_cv2(msg,"bgr8")
        results = self.model_(cv_image,verbose=False)
        current_frame_danger = False
        array_perception_msg = PerceptionResultArray()
        for box in results[0].boxes:
            cx = int(box.xywh[0][0])
            cy = int(box.xywh[0][1])
            
            if self.if_in_danger_zone(cx, cy):
                current_frame_danger = True
                detected_rock = PerceptionResult()
                detected_rock.confidence = float(box.conf[0])
                detected_rock.class_name = self.model_.names[int(box.cls[0])]
                detected_rock.bbox_center_x = cx
                detected_rock.bbox_center_y = cy
                
                # 3. Append it to the master array
                estimated_distance = 0.0 
                
                if self.latest_scan is not None:
                    # 1. Calculate Camera Angle (Assuming 1280 width, 1.396 FOV)
                    # We use -1 so that rocks on the left (x < 640) yield a POSITIVE angle.
                    theta_camera = -1.0 * ((cx - 640.0) / 640.0) * (1.396 / 2.0)
                    
                    # 2. Add camera mount offset (If camera is perfectly forward, this is 0)
                    theta_global = theta_camera + 0.0 
                    
                    # 3. Find the exact LiDAR index
                    # Check to prevent division by zero just in case
                    if self.scan_angle_increment > 0:
                        center_index = int((theta_global - self.scan_angle_min) / self.scan_angle_increment)
                        
                        # 4. Create the Window (+/- 3 beams)
                        # We use max() and min() to ensure we don't crash if the index is at the very edge of the array
                        start_idx = max(0, center_index - 3)
                        end_idx = min(len(self.latest_scan), center_index + 4) # +4 because Python slices are exclusive at the end
                        
                        window = self.latest_scan[start_idx:end_idx]
                        self.get_logger().info(f"Raw Window: {window}")
                        
                        # 5. Filter out Infinity/NaN and find the minimum
                        valid_distances = [d for d in window if not math.isinf(d) and not math.isnan(d) and d > 0.0]
                        
                        if len(valid_distances) > 0:
                            estimated_distance = min(valid_distances)
                        else:
                            self.get_logger().info("Window was empty or full of Inf/0.0!")
                    else:
                        self.get_logger().info("Waiting for LiDAR data... /scan is None")
                
                detected_rock.distance_meters = float(estimated_distance)

                array_perception_msg.detections.append(detected_rock)

        # The Hysteresis logic stays EXACTLY the same!
        self.detection_buffer.append(current_frame_danger)
        true_count = sum(self.detection_buffer)

        if (not self.is_rock_active) and (true_count >= 4):
            self.is_rock_active = True
        elif self.is_rock_active and (true_count == 0):
            self.is_rock_active = False            

        # 4. Finalize and Publish
        array_perception_msg.obstacle_detected = self.is_rock_active
        self.publisher_.publish(array_perception_msg)
        
        self.get_logger().info(f"State: {self.is_rock_active} | Rocks in ROI: {len(array_perception_msg.detections)} | Buffer: {true_count}/5")     
        annotated_frame = results[0].plot()
        cv2.polylines(annotated_frame, [self.roi_polygon], isClosed=True, color=(0,255, 0), thickness=2)

        
        # Convert the OpenCV image back into a ROS 2 Image message
        annotated_msg = self.cv_bridge_.cv2_to_imgmsg(annotated_frame, "bgr8")
        
        # Publish the image to the ROS network!
        self.annotated_pub_.publish(annotated_msg)

    def if_in_danger_zone(self, bbox_center_x: int, bbox_center_y: int):
        # check if bbox center is inside roi
        result = cv2.pointPolygonTest(self.roi_polygon, (bbox_center_x, bbox_center_y), False)
        if result >= 0:
            return True
        return False
    
    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg.ranges
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
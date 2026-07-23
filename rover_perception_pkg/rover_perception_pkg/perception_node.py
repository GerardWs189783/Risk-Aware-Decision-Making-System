#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image,JointState,LaserScan
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
        self.cv_bridge_ = CvBridge()
        self.subscriber_ = self.create_subscription(Image,"/image_raw",
                            self.image_callback,qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(Image,'/camera/depth_image',
                            self.depth_callback,10)
        self.joint_sub_ = self.create_subscription(JointState,"/joint_states",
                            self.joint_state_callback,10)


        self.publisher_ = self.create_publisher(PerceptionResultArray, "/perception/obstacle_info", 10)
        self.annotated_pub_ = self.create_publisher(Image, "/perception/annotated_image", 10)
        
        self.latest_depth_image = None
        self.pkg_path_ = get_package_share_directory('rover_perception_pkg')
        
        self.model_path_ = os.path.join(self.pkg_path_,'weights','sim_to_real_alpha_best.pt')
        self.model_ = YOLO(self.model_path_)
        self.detection_buffer = deque(maxlen=5) 
        self.is_rock_active = False
        self.current_mast_angle = 0.0
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
        self.image_width = 1280.0
        self.image_height = 720.0
        self.cam_center_y = self.image_height / 2.0
        self.cam_center_x = self.image_width / 2.0
        self.cam_vfov = 1.047
        self.cam_hfov = 1.396

    def image_callback(self, msg: Image):
        cv_image = self.cv_bridge_.imgmsg_to_cv2(msg, "bgr8")
        results = self.model_(cv_image, verbose=False)
        
        current_frame_danger = False
        array_perception_msg = PerceptionResultArray() # class for saving all of the rock detection
        
        for box in results[0].boxes:
            # extracting data from yolo detection (bbox paraemetrs)
            cx = int(box.xywh[0][0])
            cy = int(box.xywh[0][1])
            bbox_w = int(box.xywh[0][2])
            bbox_h = int(box.xywh[0][3])
            
            # only rocks inside the ROI danger_zone are processed
            if self.if_in_danger_zone(cx, cy):
                current_frame_danger = True
                
                # calculation of theta and phi camera angles for further c
                theta_camera = -1.0 * ((cx - self.cam_center_x) / self.cam_center_x) * (self.cam_hfov / 2.0)
                phi_camera = ((cy - self.cam_center_y) / self.cam_center_y) * (self.cam_vfov / 2.0)
                abs_phi_camera = phi_camera - self.current_mast_angle

                estimated_distance = 0.0
                valid_distance = False

                if self.latest_depth_image is not None:
                    # 5x5 patch around the center
                    patch_size = 2
                    y_min = max(0, cy - patch_size)
                    y_max = min(self.latest_depth_image.shape[0], cy + patch_size + 1)
                    x_min = max(0, cx - patch_size)
                    x_max = min(self.latest_depth_image.shape[1], cx + patch_size + 1)
                    
                    # 5x5 grid of depth values
                    depth_patch = self.latest_depth_image[y_min:y_max, x_min:x_max]
                    
                    # filtration from NanN and zero values
                    valid_depths = depth_patch[np.isfinite(depth_patch) & (depth_patch > 0.0)]
                    
                    if len(valid_depths) > 0:
                        # distance extractio, getting the closest distance between rock and camera
                        dist = np.min(valid_depths)
                        # saving the distance for ROS message
                        estimated_distance = float(dist)
                        # distance validate flag
                        valid_distance = True

                # custom ROS 2 message for the current rock
                detected_rock = PerceptionResult() # every rock is the PerceptionResult class for ROS2 msg
                detected_rock.class_name = self.model_.names[int(box.cls[0])] # class rock
                detected_rock.confidence = float(box.conf[0]) # confidence score
                detected_rock.valid_distance = bool(valid_distance) # valid distance flag
                detected_rock.bbox_center_x = int(cx) # x coordinate of center of the rock
                detected_rock.bbox_center_y = int(cy) # y coordinate of center of the rock
                detected_rock.distance_meters = float(estimated_distance) # estimated distance to the rock from RGBD camera
                detected_rock.phi_angle = float(abs_phi_camera) # phi_angle pitch of the rover
                detected_rock.theta_angle = float(theta_camera) # theta_angle yaw of the rover
                
                # every detected rock is then added to the array of the detected rock PerceptionResultArray
                array_perception_msg.detections.append(detected_rock)

        # implementation of temporal Hysteresis Filter -> to avoid sending and then processing one tick ghost rocks (filter)
        self.detection_buffer.append(current_frame_danger)
        true_count = sum(self.detection_buffer)

        if (not self.is_rock_active) and (true_count >= 4):
            self.is_rock_active = True
        elif self.is_rock_active and (true_count == 0):
            self.is_rock_active = False            

        # After filtering, the rock is send to the array of confirmed rocks
        array_perception_msg.obstacle_detected = self.is_rock_active
        self.publisher_.publish(array_perception_msg)
        
        # publish to ROS topic
        annotated_frame = results[0].plot()
        annotated_msg = self.cv_bridge_.cv2_to_imgmsg(annotated_frame, "bgr8")
        self.annotated_pub_.publish(annotated_msg)

    # drawing the ROI polygon
    def if_in_danger_zone(self, bbox_center_x: int, bbox_center_y: int):
        # check if bbox center is inside roi
        result = cv2.pointPolygonTest(self.roi_polygon, (bbox_center_x, bbox_center_y), False)
        if result >= 0:
            return True
        return False
    # ROS callback to obtain current pitch value of the camera mast
    def joint_state_callback(self, msg: JointState):
        camera_pitch = 'mast_cameras_joint' 
        if camera_pitch in msg.name:
            idx = msg.name.index(camera_pitch)
            self.current_mast_angle = msg.position[idx]

    def depth_callback(self, msg):
    # convert the ROS Img msg to an OpenCV numpy array
    # every ixel value is the distance in meters
        self.latest_depth_image = self.cv_bridge_.imgmsg_to_cv2(msg, desired_encoding='passthrough')

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
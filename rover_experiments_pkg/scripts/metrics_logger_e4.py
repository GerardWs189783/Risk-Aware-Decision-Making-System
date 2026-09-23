#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import os
import math
import numpy as np
import yaml

# ROS 2 Messages
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from ros_gz_interfaces.msg import Contacts

class E4ExperimentLogger(Node):
    def __init__(self):
        super().__init__('experiment_logger')
        
        # --- 1. E4 PARAMETERS & FILE SETUP ---
        # Fixed for E4
        self.declare_parameter('scenario', 'degraded')  
        self.declare_parameter('map_difficulty', 'W2_R2') 
        self.declare_parameter('max_mission_time', 900.0)
        
        # Sensitivity Analysis
        self.declare_parameter('test_param_name', 'baseline') 
        self.declare_parameter('test_param_value', 0.0)
        self.declare_parameter('run_id', 1)

        self.scenario = self.get_parameter('scenario').value
        self.map_difficulty = self.get_parameter('map_difficulty').value
        self.max_mission_time = self.get_parameter('max_mission_time').value
        self.param_name = self.get_parameter('test_param_name').value
        self.param_value = self.get_parameter('test_param_value').value
        self.run_id = self.get_parameter('run_id').value

        yaml_path = os.path.expanduser('~/ros2_ws/src/rover_experiments_pkg/config/ground_truth.yaml')
        try:
            with open(yaml_path, 'r') as file:
                gt_data = yaml.safe_load(file)
            maps_data = gt_data.get('maps', {})
            
            yaml_key = 'gauss' if self.scenario == 'degraded' else self.map_difficulty
            
            if yaml_key in maps_data:
                self.ground_truth_rocks = [(float(rock[0]), float(rock[1])) for rock in maps_data[yaml_key]]
                self.get_logger().info(f"Loaded {len(self.ground_truth_rocks)} GT rocks from YAML key '{yaml_key}'.")
            else:
                self.get_logger().error(f"Map key '{yaml_key}' not found in YAML!")
                self.ground_truth_rocks = []
        except Exception as e:
            self.get_logger().error(f"Failed to load GT YAML: {e}")
            self.ground_truth_rocks = []

        # Directory Setup
        base_path = os.path.expanduser('~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_4')
        metrics_dir = os.path.join(base_path, 'metrics_results')
        traj_dir = os.path.join(base_path, 'trajectory')
        os.makedirs(metrics_dir, exist_ok=True)
        os.makedirs(traj_dir, exist_ok=True)
        
        # Dynamic Filename based on the parameter being tested
        file_suffix = f"e4_{self.param_name}_{self.param_value}_run{self.run_id}.csv"        
        self.metrics_file = os.path.join(metrics_dir, f"metrics_{file_suffix}")
        self.traj_file = os.path.join(traj_dir, f"traj_{file_suffix}")
        
        # --- 2. METRIC STATES ---
        self.start_time_sec = None
        self.mission_success = False
        self.is_finalizing = False
        
        self.trajectory_x, self.trajectory_y = [], []
        self.bt_tick_times = []
        
        # Safety Metrics
        self.active_collisions = set()  
        self.active_unsafe = set()      
        self.total_collision_events = 0
        self.total_unsafe_events = 0
        self.is_bumper_hit = False
        self.was_bumper_hit = False

        self.current_r_x, self.current_r_y, self.current_yaw = 0.0, 0.0, 0.0
        
        # BT States        
        self.state_counts = {"MONITORING": 0, "PLANNING": 0, "DANGER": 0}
        self.confirmed_rocks_list = []

        # Risk Metrics
        self.max_risk = 0.0
        self.active_risk_values = []

        # Watchdog
        self.last_moving_time = None
        self.last_x, self.last_y = 0.0, 0.0
        self.create_timer(20.0, self.watchdog_check)
        
        # --- 3. SUBSCRIPTIONS ---
        self.create_subscription(Odometry, '/model/curiosity_mars_rover/odometry', self.odom_cb, 10)
        self.create_subscription(Float64, '/bt/tick_time_ms', self.bt_tick_cb, 10)
        self.create_subscription(String, '/bt/state_transition', self.bt_state_cb, 10)
        self.create_subscription(PointCloud2, '/semantic_obstacles', self.semantic_cb, 10)
        self.create_subscription(String, '/bt/mission_status', self.status_cb, 10)
        self.create_subscription(Contacts, '/rover/bumper', self.bumper_cb, 10)
        self.create_subscription(Float64, '/bt/current_risk', self.risk_cb, 10)      
        
        self.get_logger().info(f"E4 Logger Ready | Testing {self.param_name} = {self.param_value} | Run {self.run_id}")

    # --- CALLBACKS ---

    def bumper_cb(self, msg: Contacts) -> None:
        hit_rock = False
        for contact in msg.contacts:
            col1 = contact.collision1.name.lower()
            col2 = contact.collision2.name.lower()
            if "rock" in col1 or "rock" in col2 or "obstacle" in col1 or "obstacle" in col2:
                hit_rock = True
                break
        if hit_rock and not self.is_bumper_hit:
            self.get_logger().warn("PHYSICAL COLLISION DETECTED BY GAZEBO!")
        self.is_bumper_hit = hit_rock

    def odom_cb(self, msg:Odometry) -> None:
        self.current_r_x = msg.pose.pose.position.x
        self.current_r_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        self.current_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        
        self.trajectory_x.append(self.current_r_x)
        self.trajectory_y.append(self.current_r_y)
        
        rock_was_hit = False 
        
        for rock_id, (gt_x, gt_y) in enumerate(self.ground_truth_rocks):
            dx = gt_x - self.current_r_x
            dy = gt_y - self.current_r_y
            local_x = dx * math.cos(-self.current_yaw) - dy * math.sin(-self.current_yaw)
            local_y = dx * math.sin(-self.current_yaw) + dy * math.cos(-self.current_yaw)
            
            # Footprint Collision
            is_colliding = (-1.841 <= local_x <= 2.341) and (-1.469 <= local_y <= 1.469)
            if is_colliding:
                rock_was_hit = True
                
            # Unsafe Zone
            buffer = 0.5
            in_unsafe_zone = ((-1.841 - buffer) <= local_x <= (2.341 + buffer) and 
                              (-1.469 - buffer) <= local_y <= (1.469 + buffer))
            is_unsafe = in_unsafe_zone and not is_colliding
            
            # Debounce
            if is_colliding and rock_id not in self.active_collisions:
                self.active_collisions.add(rock_id)
                self.total_collision_events += 1
            elif not is_colliding and rock_id in self.active_collisions:
                self.active_collisions.remove(rock_id)
                
            if is_unsafe and rock_id not in self.active_unsafe:
                self.active_unsafe.add(rock_id)
                self.total_unsafe_events += 1
            elif not is_unsafe and rock_id in self.active_unsafe:
                self.active_unsafe.remove(rock_id)
                
        # Bumper Catch-all
        if self.is_bumper_hit and not self.was_bumper_hit:
            if not rock_was_hit:
                self.total_collision_events += 1
        self.was_bumper_hit = self.is_bumper_hit

    def semantic_cb(self, msg:PointCloud2) -> None:
        rocks = []
        for point in pc2.read_points(msg, field_names=("x", "y"), skip_nans=True):
            rocks.append((round(point[0], 2), round(point[1], 2)))
        self.confirmed_rocks_list = rocks

    def bt_state_cb(self, msg: String) -> None:
        state = msg.data.upper()
        if state in self.state_counts:
            self.state_counts[state] += 1

    def bt_tick_cb(self, msg: Float64) -> None:
        if self.start_time_sec is None:
            self.start_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.bt_tick_times.append(msg.data)

    def status_cb(self, msg: String) -> None:
        if msg.data in ["SUCCESS", "FAILED"]:
            self.mission_success = (msg.data == "SUCCESS")
            self.finalize_log()

    def risk_cb(self, msg: Float64) -> None:
        current_risk = msg.data
        if current_risk > self.max_risk:
            self.max_risk = current_risk
        if current_risk >= 0.3:
            self.active_risk_values.append(current_risk)

    def watchdog_check(self):
        if self.is_finalizing or self.start_time_sec is None:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Absolute Timeout
        if (current_time - self.start_time_sec) > self.max_mission_time:
            self.get_logger().error("MAX MISSION TIME REACHED! Terminating as FAILED.")
            self.mission_success = False
            self.finalize_log()
            return

        # Stuck Check
        if self.last_moving_time is None:
            self.last_moving_time = current_time
            self.last_x, self.last_y = self.current_r_x, self.current_r_y
            return

        dist_moved = math.hypot(self.current_r_x - self.last_x, self.current_r_y - self.last_y)
        if dist_moved > 0.5: 
            self.last_moving_time = current_time
            self.last_x, self.last_y = self.current_r_x, self.current_r_y
        elif (current_time - self.last_moving_time) > 60.0:
            self.get_logger().error("ROVER IS STUCK! Terminating as FAILED.")
            self.mission_success = False
            self.finalize_log()

    def finalize_log(self):
        if self.is_finalizing:
            return
        self.is_finalizing = True
        end_time_sec = self.get_clock().now().nanoseconds / 1e9
        
        total_time = end_time_sec - self.start_time_sec if self.start_time_sec is not None else 0.0
        
        mean_bt_tick = np.mean(self.bt_tick_times) if self.bt_tick_times else 0.0
        max_bt_tick = np.max(self.bt_tick_times) if self.bt_tick_times else 0.0
        std_bt_tick = np.std(self.bt_tick_times) if self.bt_tick_times else 0.0
        
        confirmed_count = len(self.confirmed_rocks_list)

        MATCH_TOLERANCE = 1.5
        final_false_positives = 0
        matched_gt_rocks = set()

        for bt_rx, bt_ry in self.confirmed_rocks_list:
            is_real = False
            for rock_id, (gt_x, gt_y) in enumerate(self.ground_truth_rocks):
                if math.hypot(bt_rx - gt_x, bt_ry - gt_y) <= MATCH_TOLERANCE:
                    is_real = True
                    matched_gt_rocks.add(rock_id)
                    break 
            if not is_real:
                final_false_positives += 1

        final_false_negatives = len(self.ground_truth_rocks) - len(matched_gt_rocks)

        mean_active_risk = np.mean(self.active_risk_values) if self.active_risk_values else 0.0
        
        # Save Trajectory
        with open(self.traj_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['type', 'x', 'y'])
            for x, y in zip(self.trajectory_x, self.trajectory_y):
                writer.writerow(['path', x, y])
            for rx, ry in self.confirmed_rocks_list:
                writer.writerow(['confirmed_rock', rx, ry])
            for gt_x, gt_y in self.ground_truth_rocks:
                writer.writerow(['gt_rock', gt_x, gt_y])
                
        # Save E4 Metrics
        with open(self.metrics_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Tested_Param', 'Param_Value', 'Run_ID', 'Success', 'Time_s', 
                'Collisions', 'Unsafe_States', 'False_Pos', 'False_Neg', 
                'Confirmed_Rocks', 'State_Monitor', 'State_Plan', 'State_Danger',
                'Mean_Tick_ms', 'Max_Tick_ms', 'Std_Tick_ms', 
                'Max_Risk', 'Mean_Active_Risk'
            ])
            
            writer.writerow([
                self.param_name, self.param_value, self.run_id, 
                int(self.mission_success), round(total_time, 2), self.total_collision_events, 
                self.total_unsafe_events, final_false_positives, final_false_negatives,
                confirmed_count, self.state_counts.get("MONITORING", 0), self.state_counts.get("PLANNING", 0), self.state_counts.get("DANGER", 0),
                round(mean_bt_tick, 3), round(max_bt_tick, 3), round(std_bt_tick, 3), 
                round(self.max_risk, 3), round(mean_active_risk, 3)
            ])
            
        self.get_logger().info(f"Log finalized and saved to {self.metrics_file}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = E4ExperimentLogger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
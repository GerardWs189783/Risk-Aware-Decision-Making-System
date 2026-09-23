#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import csv
import os
import math
import psutil
import numpy as np
import yaml

# ROS 2 Messages
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64, Bool
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from tf2_ros import Buffer, TransformListener
from ros_gz_interfaces.msg import Contacts

class ExperimentLogger(Node):
    def __init__(self):
        super().__init__('experiment_logger')
        
        # --- 1. PARAMETERS & FILE SETUP ---
        self.declare_parameter('experiment_name', 'E2_Deep_Dive')
        self.declare_parameter('bt_variant', 'uncertainty_aware') 
        self.declare_parameter('scenario', 'ideal')  
        self.declare_parameter('map_difficulty', 'easy')             
        self.declare_parameter('run_id', 1)
        self.declare_parameter('waypoint_config', 'wp2')
        self.declare_parameter('max_mission_time', 900.0) 
        
        self.exp_name = self.get_parameter('experiment_name').value
        self.variant = self.get_parameter('bt_variant').value
        self.scenario = self.get_parameter('scenario').value
        self.map_difficulty = self.get_parameter('map_difficulty').value
        self.run_id = self.get_parameter('run_id').value
        self.waypoint_config = self.get_parameter('waypoint_config').value
        self.max_mission_time = self.get_parameter('max_mission_time').value

        yaml_path = os.path.expanduser('~/ros2_ws/src/rover_experiments_pkg/config/ground_truth.yaml')

        try:
            with open(yaml_path, 'r') as file:
                gt_data = yaml.safe_load(file)
            maps_data = gt_data.get('maps', {})
            if self.map_difficulty in maps_data:
                self.ground_truth_rocks = [(float(rock[0]), float(rock[1])) for rock in maps_data[self.map_difficulty]]
            else:
                self.ground_truth_rocks = []
        except Exception as e:
            self.ground_truth_rocks = []

        # UPDATED DIRECTORY STRUCTURE
        base_path = os.path.expanduser('~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_2/add_exp_results')
        metrics_dir = os.path.join(base_path, 'metrics_results')
        traj_dir = os.path.join(base_path, 'trajectory')
        timeseries_dir = os.path.join(base_path, 'timeseries') # NEW TIMESERIES DIR
        
        os.makedirs(metrics_dir, exist_ok=True)
        os.makedirs(traj_dir, exist_ok=True)
        os.makedirs(timeseries_dir, exist_ok=True)
        
        file_suffix = f"{self.variant}_{self.scenario}_{self.map_difficulty}_{self.waypoint_config}_run{self.run_id}.csv"        
        self.metrics_file = os.path.join(metrics_dir, f"metrics_results_{file_suffix}")
        self.traj_file = os.path.join(traj_dir, f"traj_{file_suffix}")
        self.timeseries_file = os.path.join(timeseries_dir, f"timeseries_{file_suffix}")
        
        # --- 2. TF2 & METRIC STATES ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.start_time_sec = None
        self.mission_success = False
        self.is_finalizing = False
        
        # Path & Resources
        self.trajectory_x, self.trajectory_y = [], []
        self.cpu_usages, self.ram_usages = [], []
        self.bt_tick_times = []
        self.declare_parameter('bt_executable', 'risk_autonomy_node')
        self.bt_exec_name = self.get_parameter('bt_executable').value
        
        self.bt_process = None
        
        # Safety Metrics
        self.active_collisions = set()  
        self.active_unsafe = set()      
        self.total_collision_events = 0
        self.total_unsafe_events = 0
        self.is_bumper_hit = False
        self.was_bumper_hit = False

        self.current_r_x = 0.0
        self.current_r_y = 0.0
        self.current_yaw = 0.0
        
        # BT States & Deep Dive Tracking Variables
        self.state_counts = {"MONITORING": 0, "PLANNING": 0, "DANGER": 0}
        self.confirmed_rocks_list = []
        self.max_risk = 0.0
        self.active_risk_values = []
        self.max_confidence = 0.0
        self.active_confidence_values = []

        # NEW CONTINUOUS VARIABLES
        self.current_velocity = 0.0
        self.current_instant_risk = 0.0
        self.current_instant_confidence = 0.0
        self.current_bt_state = "MONITORING"
        self.timeseries_data = []

        self.last_moving_time = None
        self.last_x, self.last_y = 0.0, 0.0

        # --- 3. SUBSCRIPTIONS ---
        self.create_timer(20.0, self.watchdog_check)
        self.create_subscription(Odometry, '/model/curiosity_mars_rover/odometry', self.odom_cb, 10)
        self.create_subscription(Float64, '/bt/tick_time_ms', self.bt_tick_cb, 10)
        self.create_subscription(String, '/bt/state_transition', self.bt_state_cb, 10)
        self.create_subscription(PointCloud2, '/semantic_obstacles', self.semantic_cb, 10)
        self.create_subscription(String, '/bt/mission_status', self.status_cb, 10)
        self.bumper_sub = self.create_subscription(Contacts,'/rover/bumper',self.bumper_cb,10)
        
        if self.variant == 'classic':
            self.create_subscription(Float64, '/bt/current_confidence', self.confidence_cb, 10)      
        else:
            self.create_subscription(Float64, '/bt/current_risk', self.risk_cb, 10)      
            
        self.create_timer(0.5, self.track_resources)
        
        # NEW: 10Hz Time-Series Recorder
        self.create_timer(0.1, self.record_timeseries)
        
        self.get_logger().info(f"Deep-Dive Logger Ready: {self.variant} | {self.scenario} | Run {self.run_id}")

    # --- CALLBACKS ---
    def record_timeseries(self):
        """Records the continuous 10Hz data profile."""
        if self.start_time_sec is None or self.is_finalizing:
            return
            
        current_time = (self.get_clock().now().nanoseconds / 1e9) - self.start_time_sec
        val_to_log = self.current_instant_confidence if self.variant == 'classic' else self.current_instant_risk
        
        self.timeseries_data.append([
            round(current_time, 2), 
            round(self.current_r_x, 3), 
            round(self.current_r_y, 3), 
            round(self.current_velocity, 3), 
            round(val_to_log, 4), 
            self.current_bt_state
        ])

    def bumper_cb(self, msg: Contacts) -> None:
        hit_rock = False
        for contact in msg.contacts:
            col1 = contact.collision1.name.lower()
            col2 = contact.collision2.name.lower()
            if "rock" in col1 or "rock" in col2 or "obstacle" in col1 or "obstacle" in col2:
                hit_rock = True
                break
        self.is_bumper_hit = hit_rock

    def odom_cb(self, msg:Odometry) -> None:
        self.current_r_x = msg.pose.pose.position.x
        self.current_r_y = msg.pose.pose.position.y
        
        # Calculate speed for timeseries
        self.current_velocity = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        
        self.trajectory_x.append(self.current_r_x)
        self.trajectory_y.append(self.current_r_y)
        
        rock_was_hit = False
        
        for rock_id, (gt_x, gt_y) in enumerate(self.ground_truth_rocks):
            dx = gt_x - self.current_r_x
            dy = gt_y - self.current_r_y
            local_x = dx * math.cos(-yaw) - dy * math.sin(-yaw)
            local_y = dx * math.sin(-yaw) + dy * math.cos(-yaw)
            
            is_colliding = (-1.841 <= local_x <= 2.341) and (-1.469 <= local_y <= 1.469)
            if is_colliding: rock_was_hit = True
                
            buffer = 0.5
            in_unsafe_zone = ((-1.841 - buffer) <= local_x <= (2.341 + buffer) and 
                              (-1.469 - buffer) <= local_y <= (1.469 + buffer))
            is_unsafe = in_unsafe_zone and not is_colliding
            
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
        self.current_bt_state = state # Track current state for timeseries
        if state in self.state_counts:
            self.state_counts[state] += 1

    def bt_tick_cb(self, msg: Float64) -> None:
        if self.start_time_sec is None:
            self.start_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.bt_tick_times.append(msg.data)

    def track_resources(self):
        if self.bt_process is None:
            for proc in psutil.process_iter(['pid', 'name']):
                try:
                    if self.bt_exec_name in proc.info['name']:
                        self.bt_process = psutil.Process(proc.info['pid'])
                        self.bt_process.cpu_percent() 
                        break
                except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                    pass
        
        if self.bt_process is not None:
            try:
                cpu_usage = self.bt_process.cpu_percent() / psutil.cpu_count()
                ram_usage = self.bt_process.memory_percent()
                
                self.cpu_usages.append(cpu_usage)
                self.ram_usages.append(ram_usage)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                self.bt_process = None

    def status_cb(self, msg: String) -> None:
        if msg.data in ["SUCCESS", "FAILED"]:
            self.mission_success = (msg.data == "SUCCESS")
            self.finalize_log()

    def risk_cb(self, msg: Float64) -> None:
        self.current_instant_risk = msg.data
        if msg.data > self.max_risk:
            self.max_risk = msg.data
        if msg.data >= 0.3:
            self.active_risk_values.append(msg.data)

    def confidence_cb(self, msg: Float64) -> None:
        self.current_instant_confidence = msg.data
        if msg.data > self.max_confidence:
            self.max_confidence = msg.data
        self.active_confidence_values.append(msg.data)        

    def watchdog_check(self):
        if self.is_finalizing or self.start_time_sec is None:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self.start_time_sec) > self.max_mission_time:
            self.mission_success = False
            self.finalize_log()
            return

        if self.last_moving_time is None:
            self.last_moving_time = current_time
            self.last_x, self.last_y = self.current_r_x, self.current_r_y
            return

        dist_moved = math.hypot(self.current_r_x - self.last_x, self.current_r_y - self.last_y)
        if dist_moved > 0.5: 
            self.last_moving_time = current_time
            self.last_x, self.last_y = self.current_r_x, self.current_r_y
        elif (current_time - self.last_moving_time) > 60.0:
            self.mission_success = False
            self.finalize_log()

    def finalize_log(self):
        
        if self.is_finalizing: return
        self.is_finalizing = True
        
        end_time_sec = self.get_clock().now().nanoseconds / 1e9
        total_time = (end_time_sec - self.start_time_sec) if self.start_time_sec else 0.0
        
        mean_bt_tick = np.mean(self.bt_tick_times) if self.bt_tick_times else 0.0
        max_bt_tick = np.max(self.bt_tick_times) if self.bt_tick_times else 0.0
        std_bt_tick = np.std(self.bt_tick_times) if self.bt_tick_times else 0.0
        
        avg_cpu = np.mean(self.cpu_usages) if self.cpu_usages else 0.0
        avg_ram = np.mean(self.ram_usages) if self.ram_usages else 0.0
        confirmed_count = len(self.confirmed_rocks_list)

        MATCH_TOLERANCE = 1.5
        final_false_positives = 0
        matched_gt_rocks = set()

        for bt_rx, bt_ry in self.confirmed_rocks_list:
            is_real = False
            for rock_id, (gt_x, gt_y) in enumerate(self.ground_truth_rocks):
                dist = math.hypot(bt_rx - gt_x, bt_ry - gt_y)
                if dist <= MATCH_TOLERANCE:
                    is_real = True
                    matched_gt_rocks.add(rock_id)
                    break 
            if not is_real: final_false_positives += 1

        final_false_negatives = len(self.ground_truth_rocks) - len(matched_gt_rocks)
        mean_active_risk = np.mean(self.active_risk_values) if self.active_risk_values else 0.0
        mean_active_confidence = np.mean(self.active_confidence_values) if self.active_confidence_values else 0.0
        
        # 1. Save Trajectory and Confirmed Rocks
        with open(self.traj_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['type', 'x', 'y'])
            for x, y in zip(self.trajectory_x, self.trajectory_y):
                writer.writerow(['path', x, y])
            for rx, ry in self.confirmed_rocks_list:
                writer.writerow(['confirmed_rock', rx, ry])
            for gt_x, gt_y in self.ground_truth_rocks:
                writer.writerow(['gt_rock', gt_x, gt_y])
                
        # 2. Save Core Metrics
        with open(self.metrics_file, 'w', newline='') as f:
            writer = csv.writer(f)
            if self.variant == 'classic':
                writer.writerow(['Exp', 'Variant', 'Scenario', 'Map_Difficulty','Waypoints', 'Run_ID', 'Success', 'Time_s', 'Collisions', 'Unsafe_States', 'False_Pos', 'False_Neg', 'Confirmed_Rocks', 'State_Monitor', 'State_Plan', 'State_Danger', 'Mean_Tick_ms', 'Max_Tick_ms', 'Std_Tick_ms', 'CPU_Pct', 'RAM_Pct', 'Max_Confidence', 'Mean_Active_Confidence'])
                writer.writerow([self.exp_name, self.variant, self.scenario, self.map_difficulty, self.waypoint_config, self.run_id, int(self.mission_success), round(total_time, 2), self.total_collision_events, self.total_unsafe_events, final_false_positives, final_false_negatives, confirmed_count, self.state_counts.get("MONITORING", 0), self.state_counts.get("PLANNING", 0), self.state_counts.get("DANGER", 0), round(mean_bt_tick, 3), round(max_bt_tick, 3), round(std_bt_tick, 3), round(avg_cpu, 1), round(avg_ram, 1), round(self.max_confidence, 3), round(mean_active_confidence, 3)])
            else:
                writer.writerow(['Exp', 'Variant', 'Scenario', 'Map_Difficulty','Waypoints', 'Run_ID', 'Success', 'Time_s', 'Collisions', 'Unsafe_States', 'False_Pos', 'False_Neg', 'Confirmed_Rocks', 'State_Monitor', 'State_Plan', 'State_Danger', 'Mean_Tick_ms', 'Max_Tick_ms', 'Std_Tick_ms', 'CPU_Pct', 'RAM_Pct', 'Max_Risk', 'Mean_Active_Risk'])
                writer.writerow([self.exp_name, self.variant, self.scenario, self.map_difficulty, self.waypoint_config, self.run_id, int(self.mission_success), round(total_time, 2), self.total_collision_events, self.total_unsafe_events, final_false_positives, final_false_negatives, confirmed_count, self.state_counts.get("MONITORING", 0), self.state_counts.get("PLANNING", 0), self.state_counts.get("DANGER", 0), round(mean_bt_tick, 3), round(max_bt_tick, 3), round(std_bt_tick, 3), round(avg_cpu, 1), round(avg_ram, 1), round(self.max_risk, 3), round(mean_active_risk, 3)])

        # 3. SAVE TIMESERIES DATA
        metric_name = "Confidence" if self.variant == "classic" else "Risk"
        with open(self.timeseries_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['Sim_Time_s', 'X_Pos', 'Y_Pos', 'Velocity_ms', metric_name, 'BT_State'])
            writer.writerows(self.timeseries_data)

        self.get_logger().info(f"Log finalized and saved to {self.metrics_file} and {self.timeseries_file}")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ExperimentLogger()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray, String
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
import csv
import os
import sys

class E5Logger(Node):
    def __init__(self):
        super().__init__(
            'e5_logger_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)]
        )
        
        # Declare parameters for easy automation
        self.declare_parameter('scenario', 'degraded')
        scenario = self.get_parameter('scenario').get_parameter_value().string_value
        
        # Setup output directory
        self.output_dir = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_5")
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.csv_filename = os.path.join(self.output_dir, f"e5_trace_R2_W2_{scenario}.csv")
        
        # Data storage
        self.sim_time = 0.0
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.current_state = "MONITORING"
        self.data_log = []
        
        # Subscribers
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.state_sub = self.create_subscription(String, '/bt/state_transition', self.state_callback, 10)
        self.math_sub = self.create_subscription(Float64MultiArray, '/bt/e5_math_metrics', self.math_callback, 10)
        
        # Subscribe to Odometry to get the Trajectory (x, y)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.get_logger().info(f"🚀 E5 Explainability Logger Started. Saving to: {self.csv_filename}")

    def clock_callback(self, msg):
        self.sim_time = msg.clock.sec + (msg.clock.nanosec * 1e-9)

    def odom_callback(self, msg):
        # Update current rover position
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y

    def state_callback(self, msg):
        self.current_state = msg.data

    def math_callback(self, msg):
        # Only log if we have actual data
        if len(msg.data) == 6 and msg.data[0] > 0.0:
            
            # Combine the Math Trace with the Spatial Trace
            record = {
                'Sim_Time': round(self.sim_time, 2),
                'Rover_X': round(self.rover_x, 4),
                'Rover_Y': round(self.rover_y, 4),
                'Distance_m': round(msg.data[0], 4),
                'Confidence': round(msg.data[1], 4),
                'P_Obstacle': round(msg.data[2], 4),
                'K_Collision': round(msg.data[3], 4),
                'C_Severity': round(msg.data[4], 4),
                'Total_Risk': round(msg.data[5], 4),
                'BT_State': self.current_state
            }
            self.data_log.append(record)

    def save_data(self):
        if not self.data_log:
            self.get_logger().warn("No data was collected! CSV not saved.")
            return
            
        fieldnames = [
            'Sim_Time', 'Rover_X', 'Rover_Y', 'Distance_m', 
            'Confidence', 'P_Obstacle', 'K_Collision', 'C_Severity', 'Total_Risk', 'BT_State'
        ]
        
        with open(self.csv_filename, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.data_log)
            
        self.get_logger().info(f"\n✅ SUCCESSFULLY SAVED TRACE: {self.csv_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = E5Logger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nCtrl+C detected! Commencing save...")
    finally:
        # This block is GUARANTEED to run when the script is terminated
        node.save_data()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
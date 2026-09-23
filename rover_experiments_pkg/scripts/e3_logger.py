#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Float64MultiArray, String
from rosgraph_msgs.msg import Clock
import csv
import os
import signal
import sys

class E3Logger(Node):
    def __init__(self):
        # Correctly set use_sim_time for rclpy during Node initialization
        super().__init__(
            'e3_logger_node',
            parameter_overrides=[Parameter('use_sim_time', value=True)]
        )
        
        # Setup output directory
        self.output_dir = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_3")
        os.makedirs(self.output_dir, exist_ok=True)
        
        # User input for filename
        self.scenario_name = input("Enter the scenario name for this run (e.g., ideal, degraded, storm): ").strip().lower()
        self.csv_filename = os.path.join(self.output_dir, f"e3_math_{self.scenario_name}.csv")
        
        # Data storage
        self.sim_time = 0.0
        self.current_state = "MONITORING"
        self.data_log = []
        
        # Subscribers
        self.clock_sub = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.state_sub = self.create_subscription(String, '/bt/state_transition', self.state_callback, 10)
        self.math_sub = self.create_subscription(Float64MultiArray, '/bt/e3_math_metrics', self.math_callback, 10)
        
        self.get_logger().info(f" E3 Logger Started. Waiting for math metrics...")

    def clock_callback(self, msg):
        self.sim_time = msg.clock.sec + (msg.clock.nanosec * 1e-9)

    def state_callback(self, msg):
        self.current_state = msg.data

    def math_callback(self, msg):
        # Only log if we have actual data (ignore the empty arrays from zero-detections)
        if len(msg.data) == 6 and msg.data[0] > 0.0:
            
            # The indices match the C++ Float64MultiArray output
            record = {
                'Sim_Time': round(self.sim_time, 2),
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
            
        fieldnames = ['Sim_Time', 'Distance_m', 'Confidence', 'P_Obstacle', 'K_Collision', 'C_Severity', 'Total_Risk', 'BT_State']
        
        with open(self.csv_filename, mode='w', newline='') as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self.data_log)
            
        self.get_logger().info(f"\n SUCCESSFULLY SAVED: {self.csv_filename}")

def main(args=None):
    rclpy.init(args=args)
    node = E3Logger()

    # Capture Ctrl+C so we can save the dataframe to CSV before the script dies
    def signal_handler(sig, frame):
        node.save_data()
        rclpy.shutdown()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass # Handled by signal_handler

if __name__ == '__main__':
    main()
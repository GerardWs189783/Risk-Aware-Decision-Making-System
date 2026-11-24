#!/usr/bin/env python3
import os
import time
import subprocess
import itertools

# --- EXPERIMENT 4 CONFIGURATION ---
WAYPOINT_SET = "wp2"
MAP_NAME = "medium"
SCENARIO = "gauss"
NOISE_STDDEV = "0.3"
BT_EXEC = "risk_autonomy_node_e4"
RUNS_PER_COMBO = 3

# --- SENSITIVITY ANALYSIS GROUPS ---
# Format: (ROS_Parameter_Name, Value)
PARAM_TESTS = [
    ('tau_2', 0.65), ('tau_2', 0.85),
    
    # Group 3: Track Variance (var_track) - Baseline is 0.02
    ('var_track', 0.1), ('var_track', 0.25), ('var_track', 0.5),
    
    # Group 4: Safe Buffer (d_safe) - Baseline is 1.5
    ('d_safe', 0.5), ('d_safe', 1.0), ('d_safe', 2.0),
    
    # Group 5: Reaction Time (t_react) - Baseline is 0.5
    ('t_react', 0.3), ('t_react', 0.8), ('t_react', 1.2)
]

def main():
    # Generate the execution list (combining params and runs)
    combinations = list(itertools.product(PARAM_TESTS, range(1, RUNS_PER_COMBO + 1)))
    total_runs = len(combinations)
    
    print(f"==================================================")
    print(f"🚀 STARTING SENSITIVITY ANALYSIS (E4)")
    print(f"🌍 Environment: {SCENARIO.upper()} | Map: {MAP_NAME} | Waypoints: {WAYPOINT_SET}")
    print(f"📊 Total Scheduled Runs: {total_runs}")
    print(f"==================================================\n")
    
    for idx, ((param_name, param_val), run_id) in enumerate(combinations):
        try:
            percent = (idx / total_runs) * 100
            bar_length = 30
            filled_length = int(bar_length * idx / total_runs)
            bar = '█' * filled_length + '-' * (bar_length - filled_length)
            
            print(f"\n==================================================")
            print(f"PROGRESS: [{bar}] {percent:.1f}% ({idx}/{total_runs} completed)")
            print(f"STARTING RUN {idx+1}/{total_runs}")
            print(f"   Testing Param: {param_name} = {param_val} | Run: {run_id}")
            print(f"==================================================")
            
            world_file_name = f"map_{MAP_NAME}_{SCENARIO}.world"

            gz_env = os.environ.copy()
            gz_env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
            gz_env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
            
            print("-> Launching Gazebo (Headless)...")
            cmd_gazebo = [
                "ros2", "launch", "curiosity_gazebo", "test_e1.launch.py",
                f"world_name:={world_file_name}",
                "headless:=true",
                f"camera_noise_stddev:={NOISE_STDDEV}"
            ]
            p_gazebo = subprocess.Popen(cmd_gazebo, env=gz_env)
            time.sleep(12) 
            
            # 2. LAUNCH AUTONOMY (BT starts, but rover CANNOT move yet)
            print(f"-> Launching Autonomy Node: {BT_EXEC}")
            cmd_autonomy = [
                "ros2", "launch", "rover_navigation_pkg", "autonomy_bringup.launch.py",
                f"bt_executable:={BT_EXEC}" 
            ]       
            p_autonomy = subprocess.Popen(cmd_autonomy)
            time.sleep(10) # Give the BT node time to fully wake up
            
            # 3. INJECT PARAMETER (Rover is safely frozen at the start line)
            print(f"-> Attempting to inject E4 Parameter: {param_name} = {param_val}")
            param_set_success = False
            for attempt in range(15):
                cmd_set_param = [
                    "ros2", "param", "set", f"/{BT_EXEC}", param_name, str(param_val)
                ]
                result = subprocess.run(cmd_set_param, capture_output=True, text=True)
                
                if "successful" in result.stdout.lower():
                    print(f"   ✅ [SUCCESS] {param_name} successfully set to {param_val} (Attempt {attempt + 1})")
                    param_set_success = True
                    break
                time.sleep(2.0)
            
            if not param_set_success:
                print(f"   ❌ [FATAL ERROR] Could not set parameter!")
                print(f"   ROS 2 Error: {result.stderr}")

            # 4. LAUNCH CONTROL (Nav2 wakes up, rover begins driving WITH new math)
            print("-> Launching Control Pipeline...")
            cmd_control = ["ros2", "launch", "curiosity_rover_demo", "mars_rover.launch.py"]
            p_control = subprocess.Popen(cmd_control)
            time.sleep(5) 

            # 5. LAUNCH LOGGER
            print("-> Starting E4 Metrics Logger...")
            cmd_logger = [
                "ros2", "run", "rover_experiments_pkg", "metrics_logger_e4.py", 
                "--ros-args",
                "-p", f'scenario:="{SCENARIO}"',
                "-p", f'map_difficulty:="{MAP_NAME}"',
                "-p", f'test_param_name:="{param_name}"',
                "-p", f'test_param_value:={param_val}',
                "-p", f'run_id:={run_id}',
                "-p", 'use_sim_time:=true',
                "-p", 'max_mission_time:=900.0'
            ]
            p_logger = subprocess.Popen(cmd_logger)
            
            print("-> Running mission... (Waiting for logger to exit naturally)")
            p_logger.wait() 
            print("-> Mission Complete! Logger has saved the CSV.")
            
        finally:
            print("-> Cleaning up RAM and Zombie Processes...")
            
            processes_to_kill = [
                "gazebo", "ruby", "gz", "ign", 
                "ros2", "nav2", "robot_state_publisher", 
                "risk_autonomy_node_e4", "metrics_logger_e4.py"
            ]
            
            for proc in processes_to_kill:
                subprocess.run(["pkill", "-9", "-f", proc], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

            print("-> Clearing Shared Memory (Preventing FastDDS Deadlock)...")
            subprocess.run("rm -rf /dev/shm/*fastrtps*", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)    
            
            print("-> Cooldown period...")
            time.sleep(5)

    print(f"\n==================================================")
    print(f"✅ PROGRESS: [██████████████████████████████] 100.0% ({total_runs}/{total_runs} completed)")
    print(f"🎉 ALL E4 SENSITIVITY EXPERIMENTS COMPLETED SUCCESSFULLY! 🎉")
    print(f"==================================================")

if __name__ == "__main__":
    main()
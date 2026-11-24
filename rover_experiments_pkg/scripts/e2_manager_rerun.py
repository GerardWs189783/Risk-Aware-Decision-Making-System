#!/usr/bin/env python3
import os
import time
import subprocess

# --- EXPERIMENT CONFIGURATION ---
WAYPOINT_SET = "wp1" # Updated to match your corrupted list

def main():
    # 1. HARDCODE THE SPECIFIC RUNS TO REPEAT
    # Format: (variant, scenario, map_name, run_id)
    combinations = [
        ("no_platt","gauss","medium",1),
    ]
    
    total_runs = len(combinations)
        
    print(f"==================================================")
    print(f"🚀 STARTING ABLATION (E2) EXPERIMENTS: {WAYPOINT_SET.upper()}")
    print(f"📊 Total Runs Scheduled: {total_runs}")
    print(f"==================================================\n")
    
    for idx, (variant, scenario, map_name, run_id) in enumerate(combinations):
        try:    
            percent = (idx / total_runs) * 100
            bar_length = 30
            filled_length = int(bar_length * idx / total_runs)
            bar = '█' * filled_length + '-' * (bar_length - filled_length)
            
            print(f"\n==================================================")
            print(f"PROGRESS: [{bar}] {percent:.1f}% ({idx}/{total_runs} completed)")
            print(f"STARTING RUN {idx+1}/{total_runs}")
            print(f"   Variant: {variant} | Scenario: {scenario} | Map: {map_name} | Run: {run_id}")
            print(f"==================================================")
            
            if scenario == "ideal":
                noise_stddev = "0.007"
            elif scenario == "gauss":
                noise_stddev = "0.3"
            elif scenario == "storm":
                noise_stddev = "0.007"
            else:
                noise_stddev = "0.007"

            world_file_name = f"map_{map_name}_{scenario}.world"
            
            # 3. Setup Gazebo Environment Variables
            gz_env = os.environ.copy()
            gz_env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
            gz_env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
            
            # ==========================================
            # LAUNCH 1: GAZEBO (Headless)
            # ==========================================
            print("-> Launching Gazebo (Headless)...")
            cmd_gazebo = [
                "ros2", "launch", "curiosity_gazebo", "test_e1.launch.py",
                f"world_name:={world_file_name}",
                "headless:=true",
                f"camera_noise_stddev:={noise_stddev}"
            ]
            p_gazebo = subprocess.Popen(cmd_gazebo, env=gz_env)
            time.sleep(12) 
            
            # ==========================================
            # DYNAMIC EXECUTABLE NAME
            # ==========================================
            # This will perfectly match your C++ exec names!
            bt_exec = f"{variant}_autonomy_node"
            
            # ==========================================
            # LAUNCH 2 & 3: AUTONOMY & CONTROL
            # ==========================================
            print(f"-> Launching Autonomy Node: {bt_exec}")
            cmd_autonomy = [
                "ros2", "launch", "rover_navigation_pkg", "autonomy_bringup.launch.py",
                f"bt_executable:={bt_exec}" # <-- Passing the executable name to the launch file!
            ]       
            cmd_control = ["ros2", "launch", "curiosity_rover_demo", "mars_rover.launch.py"]
            
            p_autonomy = subprocess.Popen(cmd_autonomy)
            time.sleep(8)
            print("-> Launching Control")
            p_control = subprocess.Popen(cmd_control)
            time.sleep(5) 
            
            # ==========================================
            # LAUNCH 4: METRICS LOGGER
            # ==========================================
            print("-> Starting Metrics Logger...")
            
            cmd_logger = [
                "ros2", "run", "rover_experiments_pkg", "metrics_logger_e2.py", 
                "--ros-args",
                "-p", 'experiment_name:="E2_Ablation"',
                "-p", f'bt_variant:="{variant}"',
                "-p", f'scenario:="{scenario}"',
                "-p", f'map_difficulty:="{map_name}"',
                "-p", f'run_id:={run_id}',
                "-p", f'waypoint_config:="{WAYPOINT_SET}"',
                "-p", f'bt_executable:="{bt_exec}"',
                "-p", 'use_sim_time:=true',
                "-p", 'max_mission_time:=900.0'
            ]
            p_logger = subprocess.Popen(cmd_logger)
            
            # ==========================================
            # WAIT FOR COMPLETION
            # ==========================================
            print("-> Running mission... (Waiting for logger to exit naturally)")
            p_logger.wait() 
            print("-> Mission Complete! Logger has saved the CSV.")
        finally: # <--- THIS GUARANTEES CLEANUP NO MATTER WHAT
            # ==========================================
            # THE PURGE
            # ==========================================
            print("-> Cleaning up RAM and Zombie Processes...")
            
            processes_to_kill = [
                "gazebo", "ruby", "gz", "ign", 
                "ros2", "nav2", "robot_state_publisher", 
                "no_platt_autonomy_node", "no_pcol_autonomy_node", "no_severity_autonomy_node"
            ]
            
            for proc in processes_to_kill:
                subprocess.run(["pkill", "-9", "-f", proc], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

            print("-> Clearing Shared Memory (Preventing FastDDS Deadlock)...")
            subprocess.run("rm -rf /dev/shm/*fastrtps*", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)    
            
            print("-> Cooldown period...")
            time.sleep(5)

    print(f"\n==================================================")
    print(f"✅ PROGRESS: [██████████████████████████████] 100.0% ({total_runs}/{total_runs} completed)")
    print(f"🎉 ALL BATCH EXPERIMENTS COMPLETED SUCCESSFULLY! 🎉")
    print(f"==================================================")

if __name__ == "__main__":
    main()
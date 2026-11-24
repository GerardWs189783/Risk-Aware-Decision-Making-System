#!/usr/bin/env python3
"""Single-run interactive simulation manager for the Risk-Aware Rover."""

import argparse
import os
import signal
import subprocess
import time


VARIANT_TO_EXEC = {
    "uncertainty_aware": "risk_autonomy_node",
    "classic": "classic_autonomy_node",
    "no_platt": "ablation_no_platt_node",
    "no_pcol": "ablation_no_pcol_node",
    "no_severity": "ablation_no_severity_node",
}

SCENARIO_TO_NOISE = {
    "ideal": "0.007",
    "gauss": "0.3",
    "storm": "0.007",
}


def cleanup_processes():
    """Terminates lingering Gazebo/ROS 2 processes, clears FastDDS SHM, and resets daemon."""
    print("\n-> Cleaning up simulation processes, FastDDS shared memory, and ROS 2 daemon...")

    processes_to_kill = [
        "gazebo", "ruby", "gz", "ign",
        "nav2", "robot_state_publisher", "odom_tf_publisher",
        "parameter_bridge", "image_bridge", "perception_node.py",
        "risk_autonomy_node", "classic_autonomy_node",
        "ablation_no_platt_node", "ablation_no_pcol_node", "ablation_no_severity_node",
        "bt_autonomy_node", "rviz2",
    ]

    for proc in processes_to_kill:
        subprocess.run(
            ["pkill", "-9", "-f", proc],
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
        )

    # Clear FastDDS / CycloneDDS shared memory segments to prevent deadlocks
    subprocess.run(
        "rm -rf /dev/shm/*fastrtps* /dev/shm/*fastdds*",
        shell=True,
        stderr=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
    )

    # Restart ROS 2 daemon cleanly
    subprocess.run(["ros2", "daemon", "stop"], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    time.sleep(1.0)
    subprocess.run(["ros2", "daemon", "start"], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    print("-> Cleanup complete.")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Run a single Mars Rover simulation with the selected decision-making architecture."
    )
    parser.add_argument(
        "--variant",
        type=str,
        default="uncertainty_aware",
        choices=list(VARIANT_TO_EXEC.keys()),
        help="Decision-making architecture variant (default: uncertainty_aware)",
    )
    parser.add_argument(
        "--scenario",
        type=str,
        default="ideal",
        choices=list(SCENARIO_TO_NOISE.keys()),
        help="Environmental perception condition (default: ideal)",
    )
    parser.add_argument(
        "--map",
        type=str,
        default="medium",
        choices=["easy", "medium", "hard"],
        help="Obstacle map difficulty R1/R2/R3 (default: medium)",
    )
    parser.add_argument(
        "--wp",
        type=str,
        default="wp2",
        choices=["wp1", "wp2", "wp3"],
        help="Validated spawn & waypoint route preset (default: wp2)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run Gazebo headless and disable RViz (default: false, GUI enabled)",
    )
    parser.add_argument(
        "--no-nvidia",
        action="store_true",
        help="Disable NVIDIA PRIME render offload environment variables",
    )
    parser.add_argument(
        "--log-metrics",
        action="store_true",
        help="Also launch metrics_logger_e1_2.py and exit automatically upon mission completion",
    )
    parser.add_argument(
        "--run-id",
        type=int,
        default=1,
        help="Run ID used when --log-metrics is enabled (default: 1)",
    )
    return parser.parse_args()


def is_nvidia_driver_ready() -> bool:
    """Returns True only if nvidia-smi executes and communicates with an active driver."""
    try:
        res = subprocess.run(
            ["nvidia-smi"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3,
        )
        return res.returncode == 0
    except (FileNotFoundError, subprocess.SubprocessError):
        return False


def main():
    args = parse_args()

    bt_exec = VARIANT_TO_EXEC[args.variant]
    noise_stddev = SCENARIO_TO_NOISE[args.scenario]
    world_file_name = f"map_{args.map}_{args.scenario}.world"
    headless_str = "true" if args.headless else "false"

    print("==================================================")
    print(" MARS ROVER SIMULATION RUNNER")
    print(f"  Variant   : {args.variant} ({bt_exec})")
    print(f"  Scenario  : {args.scenario} (camera noise stddev={noise_stddev})")
    print(f"  Map       : {args.map} ({world_file_name})")
    print(f"  Route     : {args.wp}")
    print(f"  Mode      : {'Headless' if args.headless else 'GUI (Gazebo + RViz)'}")
    print("==================================================\n")

    # Configure GPU environment only when an NVIDIA driver is actively responding
    sim_env = os.environ.copy()
    if not args.no_nvidia and not args.headless and is_nvidia_driver_ready():
        print("-> NVIDIA GPU active: Enabling PRIME render offload.")
        sim_env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
        sim_env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
    else:
        sim_env.pop("__NV_PRIME_RENDER_OFFLOAD", None)
        sim_env.pop("__GLX_VENDOR_LIBRARY_NAME", None)
        print("-> Using default system GPU (Intel/Mesa).")

    launched_procs = []

    try:
        # Launch Gazebo Simulation
        print("-> [1/3] Launching Gazebo world and rover...")
        cmd_gazebo = [
            "ros2", "launch", "curiosity_gazebo", "curiosity_gazebo.launch.py",
            f"world_name:={world_file_name}",
            f"headless:={headless_str}",
            f"camera_noise_stddev:={noise_stddev}",
            f"waypoint_config:={args.wp}",
        ]
        p_gazebo = subprocess.Popen(cmd_gazebo, env=sim_env, preexec_fn=os.setsid)
        launched_procs.append(p_gazebo)
        time.sleep(12)

        if p_gazebo.poll() is not None:
            raise RuntimeError("Gazebo launch exited prematurely. Check OpenGL/world logs above.")

        # Launch Perception, Nav2, and Behavior Tree Autonomy
        print("-> [2/3] Launching YOLOv8 Perception, Nav2, and Behavior Tree...")
        cmd_autonomy = [
            "ros2", "launch", "rover_navigation_pkg", "autonomy_bringup.launch.py",
            f"bt_executable:={bt_exec}",
            f"waypoint_config:={args.wp}",
        ]
        launched_procs.append(subprocess.Popen(cmd_autonomy, env=sim_env, preexec_fn=os.setsid))
        time.sleep(8)

        # Launch Rover
        print("-> [3/3] Launching Rover Control node...")
        cmd_control = ["ros2", "launch", "curiosity_rover_demo", "mars_rover.launch.py"]
        launched_procs.append(subprocess.Popen(cmd_control, env=sim_env, preexec_fn=os.setsid))
        time.sleep(3)

        if args.log_metrics:
            print("-> Starting Metrics Logger (waiting for mission completion)...")
            cmd_logger = [
                "ros2", "run", "rover_experiments_pkg", "metrics_logger_e1_2.py",
                "--ros-args",
                "-p", 'experiment_name:="Demo_Run"',
                "-p", f'bt_variant:="{args.variant}"',
                "-p", f'scenario:="{args.scenario}"',
                "-p", f'map_difficulty:="{args.map}"',
                "-p", f"run_id:={args.run_id}",
                "-p", f'waypoint_config:="{args.wp}"',
                "-p", f'bt_executable:="{bt_exec}"',
                "-p", "use_sim_time:=true",
                "-p", "max_mission_time:=900.0",
            ]
            p_logger = subprocess.Popen(cmd_logger, env=sim_env, preexec_fn=os.setsid)
            launched_procs.append(p_logger)
            p_logger.wait()
            print("-> Mission finished and metrics logged!")
        else:
            print("\n✅ Simulation running! Press Ctrl+C in this terminal to stop and clean up.\n")
            while True:
                time.sleep(1.0)

    except KeyboardInterrupt:
        print("\n-> Interrupt received from user (Ctrl+C).")
    except RuntimeError as exc:
        print(f"\n[ERROR] {exc}")
    finally:
        for p in launched_procs:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(2.0)
        cleanup_processes()


if __name__ == "__main__":
    main()
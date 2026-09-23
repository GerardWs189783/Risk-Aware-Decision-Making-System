#!/usr/bin/env python3
import os
import glob
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
# Point to Experiment 2 trajectory data
CSV_DIRECTORY = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_2/trajectory")

# We only analyze Medium (R2) and Hard (R3) for the ablation study
MAP_DIFFICULTIES = ["medium", "hard"]
SCENARIOS = ["ideal", "gauss", "storm"]
WAYPOINT_SETS = ["wp1", "wp2", "wp3"]

# Define the Ablation Variants and their formal titles for the plot headers
VARIANTS = {
    "no_platt": "No Platt Scaling",
    "no_pcol": "No P(Col)",
    "no_severity": "No Severity Index"
}

# Dictionary containing all waypoint coordinates
WAYPOINTS_CONFIG = {
    "wp1": {
        "Start": (-21.0, 17.0),
        "WP1": (-8.0, 15.0),
        "WP2": (-7.0, -2.0)
    },
    "wp2": {
        "Start": (-11.0, -12.5),
        "WP1": (5.2, 6.5),
        "WP2": (5.0, 16.0)
    },
    "wp3": {
        "Start": (-13.0, -9.0),
        "WP1": (-5.0, 13.0),
        "WP2": (-3.0, 20.0)
    }
}

def extract_data(filepath):
    """Custom parser to sort rows based on their leading label."""
    path_x, path_y = [], []
    gt_rocks, confirmed_rocks = [], []
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: 
                continue
                
            parts = line.split(',')
            if len(parts) >= 3:
                label = parts[0].strip().lower()
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    
                    if label == 'path':
                        path_x.append(x)
                        path_y.append(y)
                    elif label == 'gt_rock':
                        gt_rocks.append((x, y))
                    elif 'confirmed' in label: 
                        confirmed_rocks.append((x, y))
                except ValueError:
                    continue

    return path_x, path_y, gt_rocks, confirmed_rocks

def main():
    # Dedicated output directory for individual ablation paths
    output_dir = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/ablation_individual_paths")
    os.makedirs(output_dir, exist_ok=True)
    
    print("Generating individual ablation trajectory plots for Medium & Hard maps...")

    for map_diff in MAP_DIFFICULTIES:
        map_idx = "2" if map_diff == "medium" else "3"
        
        for var_key, var_title in VARIANTS.items():
            for wp_set in WAYPOINT_SETS:
                wp_num = wp_set[-1]
                
                for scenario in SCENARIOS:
                    # Grab all 5 runs for this specific combination
                    search_pattern = os.path.join(CSV_DIRECTORY, f"*{var_key}_{scenario}_{map_diff}_{wp_set}*.csv")
                    csv_files = glob.glob(search_pattern)
                    
                    if not csv_files:
                        continue
                        
                    display_scenario = "Degraded" if scenario == "gauss" else scenario.capitalize()
                    
                    # Title includes the specific variant name
                    title = f"{var_title} | {display_scenario} | Path $W_{{{wp_num}}}$ | Map $R_{map_idx}$"
                    
                    fig, ax = plt.subplots(figsize=(7, 7))
                    
                    gt_plotted = False # Flag to ensure we only draw ground truth rocks once
                    
                    for file in csv_files:
                        path_x, path_y, gt_rocks, confirmed_rocks = extract_data(file)
                        
                        # 1. Plot Ground Truth Rocks (Only once per image)
                        if gt_rocks and not gt_plotted:
                            gt_x, gt_y = zip(*gt_rocks)
                            ax.scatter(gt_x, gt_y, c='lightgray', marker='o', s=150, alpha=0.4, edgecolor='gray', label='GT Rocks')
                            gt_plotted = True

                        # 2. Plot Trajectory (Semi-transparent blue so 5 overlapping paths look good)
                        if path_x and path_y:
                            ax.plot(path_x, path_y, c='blue', alpha=0.3, linewidth=2, label='Rover Path')

                        # 3. Plot Mapped Rocks
                        if confirmed_rocks:
                            conf_x, conf_y = zip(*confirmed_rocks)
                            ax.scatter(conf_x, conf_y, c='red', marker='x', s=80, alpha=0.7, zorder=4, label='Mapped Rocks')

                    print(f"  -> Saved {var_key} | {map_diff} | {scenario} | {wp_set}")

                    # 4. Plot Waypoints
                    wp_data = WAYPOINTS_CONFIG[wp_set]
                    ax.scatter(*wp_data["Start"], c='limegreen', marker='s', s=150, edgecolor='black', zorder=5, label=f'Start $S_{{{wp_num}}}$')
                    ax.scatter(*wp_data["WP1"], c='gold', marker='*', s=250, edgecolor='black', zorder=5, label=f'WP1 $W_{{{wp_num},1}}$')
                    ax.scatter(*wp_data["WP2"], c='darkorange', marker='*', s=250, edgecolor='black', zorder=5, label=f'WP2 $W_{{{wp_num},2}}$')

                    # Formatting
                    ax.set_title(title, fontsize=16, fontweight='bold')
                    ax.set_xlabel("X (meters)", fontsize=12)
                    ax.set_ylabel("Y (meters)", fontsize=12)
                    ax.grid(True, linestyle='--', alpha=0.6)
                    ax.set_aspect('equal', adjustable='box')
                    
                    # Locked map bounds so all plots have identical scale
                    ax.set_xlim(-25, 30)
                    ax.set_ylim(-15, 35)
                    
                    # Clean up the legend (merges the duplicate labels into single entries)
                    handles, labels = ax.get_legend_handles_labels()
                    by_label = dict(zip(labels, handles))
                    ax.legend(by_label.values(), by_label.keys(), loc='best', fontsize=10, framealpha=0.9)
                    
                    plt.tight_layout()
                    
                    # Save the grouped plot
                    output_filename = os.path.join(output_dir, f"path_{var_key}_{map_diff}_{scenario}_{wp_set}.png")
                    plt.savefig(output_filename, dpi=300)
                    plt.close(fig) # Close to free up memory

    print(f"\n✅ All individual ablation plots saved to:\n{output_dir}")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import os
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
# Paste the exact path of the specific run you want to investigate here:
CSV_FILE_PATH = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_4/trajectory/traj_e4_tau_2_0.85_run1.csv")

# Waypoints
# WAYPOINTS = {
#     "Start": (-11.0, -12.5),
#     "WP1": (5.2, 6.5),
#     "WP2": (5.0, 16.0)
# }
# WAYPOINTS = {
#     "Start": (-21.0, 17.0),
#     "WP1": (-8.0, 15.0),
#     "WP2": (-7.0, -2.0)
# }

WAYPOINTS = {
    "Start": (-11.0, -12.5),
    "WP1": (5.2, 6.5),
    "WP2": (5.0, 16.0)
}

def extract_data(filepath):
    """Custom parser to sort rows based on their leading label (path, gt_rock, confirmed_rock)."""
    path_x, path_y = [], []
    gt_rocks, confirmed_rocks = [], []
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: 
                continue
                
            parts = line.split(',')
            
            # We need at least 3 parts: label, x, y
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
                    elif 'confirmed' in label: # Matches 'confirmed_rock' or similar
                        confirmed_rocks.append((x, y))
                except ValueError:
                    # Skip the header row if it exists (e.g., "type,x,y")
                    continue

    return path_x, path_y, gt_rocks, confirmed_rocks

def main():
    if not os.path.exists(CSV_FILE_PATH):
        print(f"❌ ERROR: Could not find the file at:\n{CSV_FILE_PATH}")
        return

    filename = os.path.basename(CSV_FILE_PATH)
    print(f"🔍 Inspecting: {filename}")

    # Extract clean data using our label-based parser
    path_x, path_y, gt_rocks, confirmed_rocks = extract_data(CSV_FILE_PATH)
    
    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_title(f"Detailed Run Inspection\n{filename}", fontsize=16, fontweight='bold', pad=15)

    # 1. PLOT ROCKS
    if gt_rocks:
        gt_x, gt_y = zip(*gt_rocks)
        ax.scatter(gt_x, gt_y, c='lightgray', marker='o', s=600, alpha=0.5, edgecolor='gray', label='Ground Truth Rocks')

    if confirmed_rocks:
        conf_x, conf_y = zip(*confirmed_rocks)
        ax.scatter(conf_x, conf_y, c='red', marker='x', s=100, linewidths=2, zorder=4, label='Rover Mapped Rocks')

    # 2. PLOT THE TRAJECTORY
    if path_x and path_y:
        ax.plot(path_x, path_y, c='blue', alpha=0.7, linewidth=1.5, marker='.', markersize=3, zorder=3, label='Rover Trajectory')

    # 3. PLOT WAYPOINTS
    ax.scatter(*WAYPOINTS["Start"], c='limegreen', marker='s', s=200, edgecolor='black', zorder=5, label='Start (-13, -9)')
    ax.scatter(*WAYPOINTS["WP1"], c='gold', marker='*', s=350, edgecolor='black', zorder=5, label='WP1 (-5, 13)')
    ax.scatter(*WAYPOINTS["WP2"], c='darkorange', marker='*', s=350, edgecolor='black', zorder=5, label='WP2 (-3, 20)')

    # Formatting
    ax.set_xlabel("X coordinate (meters)", fontsize=12, fontweight='bold')
    ax.set_ylabel("Y coordinate (meters)", fontsize=12, fontweight='bold')
    ax.grid(True, linestyle='--', alpha=0.7)
    ax.set_aspect('equal', adjustable='box')
    
    ax.legend(loc='best', fontsize=10, framealpha=0.9)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
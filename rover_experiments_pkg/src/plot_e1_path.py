#!/usr/bin/env python3
import os
import glob
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
CSV_DIRECTORY = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_1/trajectory")
WAYPOINT_SET = "wp2"
VARIANT = "uncertainty_aware"

# Define the grid structure
MAPS = ["easy","medium", "hard"]         # Rows
SCENARIOS = ["ideal", "gauss", "storm"]   # Columns

#Define Waypoints for plotting
WAYPOINTS = {
    "Start": (-11.0, -12.5),
    "WP1": (5.2, 6.5),
    "WP2": (5.0, 16.0)
}
# WAYPOINTS = {
#     "Start": (-21.0, 17.0),
#     "WP1": (-8.0, 15.0),
#     "WP2": (-7.0, -2.0)
# }

# WAYPOINTS = {
#     "Start": (-13.0, -9.0),
#     "WP1": (-5.0, 13.0),
#     "WP2": (-3.0, 20.0)
# }



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
                    elif 'confirmed' in label: 
                        confirmed_rocks.append((x, y))
                except ValueError:
                    # Skip the header row if it exists
                    continue

    return path_x, path_y, gt_rocks, confirmed_rocks

def main():
    search_pattern = os.path.join(CSV_DIRECTORY, f"*{VARIANT}*{WAYPOINT_SET}*.csv")
    csv_files = glob.glob(search_pattern)
    
    if not csv_files:
        print(f"No CSV files found in {CSV_DIRECTORY} matching *{WAYPOINT_SET}*.csv")
        return

    print(f"Found {len(csv_files)} files. Generating plots...")

    fig, axes = plt.subplots(nrows=len(MAPS), ncols=len(SCENARIOS), figsize=(18, 18), sharex=True, sharey=True)
    fig.suptitle(f"Rover Path Analysis | {VARIANT.replace('_', ' ').title()} | {WAYPOINT_SET.upper()}", fontsize=22, fontweight='bold', y=0.95)

    ax_dict = {m: {s: axes[i][j] for j, s in enumerate(SCENARIOS)} for i, m in enumerate(MAPS)}

    for file in csv_files:
        # 1. EXTRACT METADATA FROM FILENAME
        basename = os.path.basename(file).lower()
        
        # Search the filename directly for the scenario and map keywords
        scenario = next((s for s in SCENARIOS if s in basename), None)
        map_diff = next((m for m in MAPS if m in basename), None)
        
        if not scenario or not map_diff:
            continue # Skip if it doesn't perfectly match our grid
            
        ax = ax_dict[map_diff][scenario]
        
        # 2. EXTRACT DATA USING CUSTOM PARSER
        path_x, path_y, gt_rocks, confirmed_rocks = extract_data(file)
        
        # 3. PLOT THE TRAJECTORY
        if path_x and path_y:
            # Check if 'Rover Path' is already in the legend to avoid duplicates
            label_path = 'Rover Path' if 'Rover Path' not in [line.get_label() for line in ax.lines] else ""
            ax.plot(path_x, path_y, c='blue', alpha=0.5, linewidth=2, label=label_path)

        # 4. PLOT ROCKS
        if gt_rocks:
            gt_x, gt_y = zip(*gt_rocks)
            label_gt = 'GT Rocks' if 'GT Rocks' not in [c.get_label() for c in ax.collections] else ""
            ax.scatter(gt_x, gt_y, c='lightgray', marker='o', s=150, alpha=0.1, label=label_gt)

        if confirmed_rocks:
            conf_x, conf_y = zip(*confirmed_rocks)
            label_mapped = 'Mapped Rocks' if 'Mapped Rocks' not in [c.get_label() for c in ax.collections] else ""
            ax.scatter(conf_x, conf_y, c='red', marker='x', s=80, label=label_mapped)

    # Formatting the subplots
    for i, map_name in enumerate(MAPS):
        for j, scenario in enumerate(SCENARIOS):
            ax = axes[i][j]
            
            # --- PLOT WAYPOINTS ON EVERY SUBPLOT ---
            ax.scatter(*WAYPOINTS["Start"], c='limegreen', marker='s', s=120, edgecolor='black', zorder=5, label='Start' if (i==0 and j==0) else "")
            ax.scatter(*WAYPOINTS["WP1"], c='gold', marker='*', s=250, edgecolor='black', zorder=5, label='WP1' if (i==0 and j==0) else "")
            ax.scatter(*WAYPOINTS["WP2"], c='darkorange', marker='*', s=250, edgecolor='black', zorder=5, label='WP2' if (i==0 and j==0) else "")

            if i == 0:
                ax.set_title(f"Scenario: {scenario.upper()}", fontsize=16, fontweight='bold')
            if j == 0:
                ax.set_ylabel(f"Map: {map_name.upper()}\nY (meters)", fontsize=14, fontweight='bold')
            if i == len(MAPS) - 1:
                ax.set_xlabel("X (meters)", fontsize=14)
                
            ax.grid(True, linestyle='--', alpha=0.6)
            ax.set_aspect('equal', adjustable='box')
            
            # Draw the legend only on the first plot
            if i == 0 and j == 0:
                ax.legend(loc='upper left', bbox_to_anchor=(1.0, 1.0))

    plt.tight_layout(rect=[0, 0.03, 1, 0.93]) 
    
    # Save the file correctly using expanduser
    output_filename = os.path.expanduser(f"~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/path_analysis_{VARIANT}_{WAYPOINT_SET}.png")
    
    # Ensure the directory actually exists before trying to save the plot
    os.makedirs(os.path.dirname(output_filename), exist_ok=True)
    
    plt.savefig(output_filename, dpi=300)
    print(f"✅ Plot saved successfully as: {output_filename}")

if __name__ == "__main__":
    main()
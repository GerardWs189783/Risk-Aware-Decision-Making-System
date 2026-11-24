#!/usr/bin/env python3
import os
import glob
import matplotlib.pyplot as plt

# --- 1. GLOBAL SETTINGS & FONT ---
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 12
plt.rcParams['axes.labelsize'] = 14
plt.rcParams['axes.titlesize'] = 16
plt.rcParams['xtick.labelsize'] = 12
plt.rcParams['ytick.labelsize'] = 12
plt.rcParams['legend.fontsize'] = 12

# --- 2. CONFIGURATION ---
CSV_DIRECTORY = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_2/trajectory")
OUTPUT_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/e2_grids")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# E2 Ablation specifics
MAPS = ["medium", "hard"]
SCENARIOS = ["ideal", "gauss", "storm"]
WAYPOINT_SETS = ["wp1", "wp2"]

VARIANTS = {
    "no_platt": "No Platt Scaling",
    "no_pcol": "No P(collision)",
    "no_severity": "No Collision Severity Index"
}

WAYPOINTS_CONFIG = {
    "wp1": {"Start": (-21.0, 17.0), "WP1": (-8.0, 15.0), "WP2": (-7.0, -2.0)},
    "wp2": {"Start": (-11.0, -12.5), "WP1": (5.2, 6.5), "WP2": (5.0, 16.0)}
}

def extract_data(filepath):
    path_x, path_y, gt_rocks, confirmed_rocks = [], [], [], []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line: continue
            parts = line.split(',')
            if len(parts) >= 3:
                label = parts[0].strip().lower()
                try:
                    x, y = float(parts[1]), float(parts[2])
                    if label == 'path':
                        path_x.append(x); path_y.append(y)
                    elif label == 'gt_rock':
                        gt_rocks.append((x, y))
                    elif 'confirmed' in label: 
                        confirmed_rocks.append((x, y))
                except ValueError:
                    continue
    return path_x, path_y, gt_rocks, confirmed_rocks

def main():
    print("Generating Map-based 2x3 Grid Plots for E2 Ablation...")

    for var_key, var_title in VARIANTS.items():
        for map_diff in MAPS:
            map_idx = "2" if map_diff == "medium" else "3"
            
            # Create a 2x3 figure
            fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(15, 10), sharex=True, sharey=True)
            fig.suptitle(f"Ablation: {var_title} | Map $R_{{{map_idx}}}$", fontsize=22, fontweight='bold', y=0.97)
            
            for row, wp_set in enumerate(WAYPOINT_SETS):
                wp_num = wp_set[-1]
                
                for col, scenario in enumerate(SCENARIOS):
                    ax = axes[row][col]
                    
                    search_pattern = os.path.join(CSV_DIRECTORY, f"*{var_key}_{scenario}_{map_diff}_{wp_set}*.csv")
                    csv_files = glob.glob(search_pattern)
                    
                    gt_plotted = False
                    
                    for file in csv_files:
                        path_x, path_y, gt_rocks, confirmed_rocks = extract_data(file)
                        
                        if gt_rocks and not gt_plotted:
                            gt_x, gt_y = zip(*gt_rocks)
                            ax.scatter(gt_x, gt_y, c='lightgray', marker='o', s=150, alpha=0.5, edgecolor='gray', label='GT Rocks')
                            gt_plotted = True

                        if path_x and path_y:
                            label_path = 'Rover Path' if 'Rover Path' not in [line.get_label() for line in ax.lines] else ""
                            ax.plot(path_x, path_y, c='blue', alpha=0.3, linewidth=2, label=label_path)

                        if confirmed_rocks:
                            conf_x, conf_y = zip(*confirmed_rocks)
                            label_mapped = 'Mapped Rocks' if 'Mapped Rocks' not in [c.get_label() for c in ax.collections] else ""
                            ax.scatter(conf_x, conf_y, c='red', marker='x', s=60, label=label_mapped)

                    # Plot Waypoints
                    wp_data = WAYPOINTS_CONFIG[wp_set]
                    ax.scatter(*wp_data["Start"], c='limegreen', marker='s', s=120, edgecolor='black', zorder=5, label=f'Start $S_{{{wp_num}}}$')
                    ax.scatter(*wp_data["WP1"], c='gold', marker='*', s=200, edgecolor='black', zorder=5, label=f'WP1 $W_{{{wp_num},1}}$')
                    ax.scatter(*wp_data["WP2"], c='darkorange', marker='*', s=200, edgecolor='black', zorder=5, label=f'WP2 $W_{{{wp_num},2}}$')

                    ax.grid(True, linestyle='--', alpha=0.6)
                    ax.set_aspect('equal', adjustable='box')
                    
                    # Set Top Row Titles (Scenarios)
                    if row == 0:
                        display_scenario = "Degraded" if scenario == "gauss" else scenario.capitalize()
                        ax.set_title(display_scenario, fontsize=18, fontweight='bold')
                    
                    # Set Left Column Y-Labels (Waypoints instead of Maps)
                    if col == 0:
                        ax.set_ylabel(f"Path $W_{{{wp_num}}}$\nY (meters)", fontweight='bold')
                        
                    # Set Bottom Row X-Labels
                    if row == 1:
                        ax.set_xlabel("X (meters)", fontweight='bold')

                    # Draw Legend for EVERY row on the far right column
                    if col == 2:
                        handles, labels = ax.get_legend_handles_labels()
                        by_label = dict(zip(labels, handles))
                        ax.legend(by_label.values(), by_label.keys(), loc='center left', bbox_to_anchor=(1.02, 0.5), borderaxespad=0.)

            plt.subplots_adjust(wspace=0.05, hspace=0.05)
            
            output_filename = os.path.join(OUTPUT_DIR, f"grid_2x3_{var_key}_{map_diff}.png")
            plt.savefig(output_filename, dpi=300, bbox_inches='tight')
            plt.close() 
            print(f"  -> Saved: {output_filename}")

    print("✅ All 6 E2 Map-based grids generated successfully!")

if __name__ == "__main__":
    main()
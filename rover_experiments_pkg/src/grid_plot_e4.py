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
# Point to Experiment 4 trajectory data
CSV_DIRECTORY = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_4/trajectory")
OUTPUT_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/e4_grids")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# E4 is locked to these conditions
MAP_DIFF = "medium"
SCENARIO = "gauss"
WP_SET = "wp2"

# Parameter groups exactly as tested
PARAM_GROUPS = {
    "tau_1": [0.2, 0.4],
    "tau_2": [0.55, 0.65, 0.85],
    "var_track": [0.1, 0.25, 0.5],
    "d_safe": [0.5, 1.0, 2.0],
    "t_react": [0.3, 0.8, 1.2]
}

# Formal LaTeX labels for the titles
PARAM_LABELS = {
    "tau_1": r"$\tau_1$",
    "tau_2": r"$\tau_2$",
    "var_track": r"$\sigma_{\text{track}}^2$",
    "d_safe": r"$d_{\text{safe}}$",
    "t_react": r"$t_{\text{react}}$"
}

WAYPOINTS = {"Start": (-11.0, -12.5), "WP1": (5.2, 6.5), "WP2": (5.0, 16.0)}

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
    print(f"Generating 1D Grid Plots for E4 Sensitivity Analysis...")

    for param_name, values in PARAM_GROUPS.items():
        n_cols = len(values)
        
        # Dynamically size the figure based on how many values we tested (e.g., 2 cols vs 3 cols)
        fig, axes = plt.subplots(nrows=1, ncols=n_cols, figsize=(6 * n_cols, 6), sharey=True)
        
        # If there's only 1 row, axes might not be a 2D array, so we ensure it's iterable
        if n_cols == 1: axes = [axes]
            
        param_label = PARAM_LABELS[param_name]
        fig.suptitle(f"Sensitivity Analysis: {param_label} | Map $R_2$ | Degraded Conditions", fontsize=22, fontweight='bold', y=0.98)
        
        for col, val in enumerate(values):
            ax = axes[col]
            
            # Look for the specific parameter and value in the filename
            search_pattern = os.path.join(CSV_DIRECTORY, f"*{param_name}_{val}*.csv")
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
            ax.scatter(*WAYPOINTS["Start"], c='limegreen', marker='s', s=120, edgecolor='black', zorder=5, label='Start $S_2$')
            ax.scatter(*WAYPOINTS["WP1"], c='gold', marker='*', s=200, edgecolor='black', zorder=5, label='WP1 $W_{2,1}$')
            ax.scatter(*WAYPOINTS["WP2"], c='darkorange', marker='*', s=200, edgecolor='black', zorder=5, label='WP2 $W_{2,2}$')

            ax.grid(True, linestyle='--', alpha=0.6)
            ax.set_aspect('equal', adjustable='box')
            
            # Set Subplot Title to the specific parameter value tested
            ax.set_title(f"{param_label} = {val}", fontsize=18, fontweight='bold')
            ax.set_xlabel("X (meters)", fontweight='bold')
            
            if col == 0:
                ax.set_ylabel("Y (meters)", fontweight='bold')

            # Draw Legend on the far right column
            if col == n_cols - 1:
                handles, labels = ax.get_legend_handles_labels()
                by_label = dict(zip(labels, handles))
                ax.legend(by_label.values(), by_label.keys(), loc='center left', bbox_to_anchor=(1.02, 0.5), borderaxespad=0.)

        plt.subplots_adjust(wspace=0.05)
        
        output_filename = os.path.join(OUTPUT_DIR, f"grid_1x{n_cols}_{param_name}.png")
        plt.savefig(output_filename, dpi=300, bbox_inches='tight')
        plt.close() 
        print(f"  -> Saved: {output_filename}")

    print("✅ All E4 sensitivity grids generated successfully!")

if __name__ == "__main__":
    main()
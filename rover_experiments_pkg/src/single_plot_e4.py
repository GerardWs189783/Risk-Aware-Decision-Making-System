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
plt.rcParams['legend.fontsize'] = 11

# =====================================================================
# --- 2. USER CONFIGURATION: CHOOSE PARAMETER AND VALUE HERE ---
# Options: "tau_1", "tau_2", "var_track", "d_safe", "t_react"
SELECTED_PARAMETER = "t_react" 
SELECTED_VALUE = 1.2  # Enter the specific value you want to plot
# =====================================================================

# Directories
TRAJ_DIR_E4 = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_4/trajectory")
OUTPUT_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/e4_single_plots")
os.makedirs(OUTPUT_DIR, exist_ok=True)

PARAM_LABELS = {
    "tau_1": r"$\tau_1$", "tau_2": r"$\tau_2$", "var_track": r"$\sigma_{\text{track}}^2$",
    "d_safe": r"$d_{\text{safe}}$", "t_react": r"$t_{\text{react}}$"
}

# Waypoints (E4 is locked to Map R2, WP2)
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
    if SELECTED_PARAMETER not in PARAM_LABELS:
        print(f"❌ Error: '{SELECTED_PARAMETER}' is not a valid parameter.")
        return

    print(f"Generating isolated plot for {SELECTED_PARAMETER} = {SELECTED_VALUE}...")

    fig, ax = plt.subplots(figsize=(8, 8))
    latex_symbol = PARAM_LABELS[SELECTED_PARAMETER]
    
    ax.set_title(f"Sensitivity Analysis: {latex_symbol} = {SELECTED_VALUE}", fontsize=18, fontweight='bold')
    
    gt_plotted = False

    # --- PLOT SINGLE VARIATION (E4) ---
    search_pattern = os.path.join(TRAJ_DIR_E4, f"*{SELECTED_PARAMETER}_{SELECTED_VALUE}*.csv")
    csv_files = glob.glob(search_pattern)
    
    var_label = f"Rover Path ({latex_symbol} = {SELECTED_VALUE})"
    
    if not csv_files:
        print(f"⚠️ Warning: No CSV files found for {SELECTED_PARAMETER} = {SELECTED_VALUE}")
        
    for file in csv_files:
        path_x, path_y, gt_rocks, confirmed_rocks = extract_data(file)
        
        # Plot GT Rocks once
        if gt_rocks and not gt_plotted:
            gt_x, gt_y = zip(*gt_rocks)
            ax.scatter(gt_x, gt_y, c='lightgray', marker='o', s=150, alpha=0.5, edgecolor='gray', label='GT Rocks')
            gt_plotted = True
            
        # Plot Path
        if path_x and path_y:
            label = var_label if var_label not in [line.get_label() for line in ax.lines] else ""
            ax.plot(path_x, path_y, c='blue', alpha=0.5, linewidth=2, label=label)
            
        # Plot Mapped Rocks
        if confirmed_rocks:
            conf_x, conf_y = zip(*confirmed_rocks)
            label_mapped = 'Mapped Rocks' if 'Mapped Rocks' not in [c.get_label() for c in ax.collections] else ""
            ax.scatter(conf_x, conf_y, c='red', marker='x', s=60, label=label_mapped)

    # --- PLOT WAYPOINTS ---
    ax.scatter(*WAYPOINTS["Start"], c='limegreen', marker='s', s=120, edgecolor='black', zorder=5, label='Start $S_2$')
    ax.scatter(*WAYPOINTS["WP1"], c='gold', marker='*', s=200, edgecolor='black', zorder=5, label='WP1 $W_{2,1}$')
    ax.scatter(*WAYPOINTS["WP2"], c='darkorange', marker='*', s=200, edgecolor='black', zorder=5, label='WP2 $W_{2,2}$')

    # --- FORMATTING ---
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel("X (meters)", fontweight='bold')
    ax.set_ylabel("Y (meters)", fontweight='bold')
    
    # Lock bounds for direct visual comparison across different plots
    ax.set_xlim(-25, 30)
    ax.set_ylim(-15, 35)

    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='upper left', framealpha=0.9)

    plt.tight_layout()
    output_filename = os.path.join(OUTPUT_DIR, f"isolated_plot_{SELECTED_PARAMETER}_{SELECTED_VALUE}.png")
    plt.savefig(output_filename, dpi=300)
    print(f"✅ Saved plot to: {output_filename}")

if __name__ == "__main__":
    main()
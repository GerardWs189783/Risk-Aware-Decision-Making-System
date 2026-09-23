#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# --- CONFIGURATION ---
CSV_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_3")
OUTPUT_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/experiment_3_plots")
os.makedirs(OUTPUT_DIR, exist_ok=True)

csv_files = glob.glob(os.path.join(CSV_DIR, "e3_math_*.csv"))

# Color mapping for BT states
STATE_COLORS = {
    'MONITORING': '#d4edda',  # Light Green
    'PLANNING': '#fff3cd',    # Light Yellow
    'DANGER': '#f8d7da'       # Light Red
}

def plot_dashboard(df, scenario_name):
    """Creates the stacked 3-pane Anatomy of Risk dashboard against Time."""
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(f"Expected Risk components plots | {scenario_name.capitalize()} Conditions", fontsize=21, fontweight='bold', y=0.95)
    
    # 1. TOP PANE: Perception
    ax1.plot(df['Sim_Time'], df['Confidence'], label='Raw Confidence ($s_i$)', color='gray', alpha=0.6, linestyle='--')
    ax1.plot(df['Sim_Time'], df['P_Obstacle'], label='Calibrated P(Obstacle)', color='blue', linewidth=2)
    ax1.set_ylabel("Probability", fontsize=15, fontweight='bold')
    ax1.set_ylim(-0.05, 1.05)
    ax1.legend(loc='upper left')
    ax1.grid(True, linestyle='--', alpha=0.5)

    # 2. MIDDLE PANE: Spatial & Kinematic
    ax2.plot(df['Sim_Time'], df['K_Collision'], label='Collision Kernel ($K_{col}$)', color='purple', linewidth=2)
    ax2.plot(df['Sim_Time'], df['C_Severity'], label='Severity Index ($C_i$)', color='darkorange', linewidth=2)
    ax2.set_ylabel("Factor Value", fontsize=15, fontweight='bold')
    ax2.set_ylim(-0.05, 1.05)
    ax2.legend(loc='upper left')
    ax2.grid(True, linestyle='--', alpha=0.5)

    # 3. BOTTOM PANE: Total Risk & BT States
    ax3.plot(df['Sim_Time'], df['Total_Risk'], label='Total Expected Risk ($R_i$)', color='red', linewidth=2.5)
    ax3.axhline(y=0.30, color='darkorange', linestyle='--', alpha=0.8, label=r'Planning Threshold ($\tau_1$)')
    ax3.axhline(y=0.75, color='darkred', linestyle='--', alpha=0.8, label=r'Danger Threshold ($\tau_2$)')
    ax3.set_ylabel("Risk ($R_i$)", fontsize=15, fontweight='bold')
    ax3.set_xlabel("Elapsed Time (seconds)", fontsize=19, fontweight='bold')
    ax3.set_ylim(-0.05, 1.05)
    
    # EXACT LIMITS: Lock X-axis from 0 to the exact final time value of this run
    max_time = df['Sim_Time'].max()
    ax3.set_xlim(0, max_time)
    
    ax3.grid(True, linestyle='--', alpha=0.5)

    # Add background spans for BT States
    times = df['Sim_Time'].values
    states = df['BT_State'].values
    start_idx = 0
    
    for i in range(1, len(states)):
        if states[i] != states[start_idx]:
            state = states[start_idx]
            color = STATE_COLORS.get(state, 'white')
            ax3.axvspan(times[start_idx], times[i], color=color, alpha=0.7, lw=0)
            start_idx = i
            
    # Final span to the end
    if start_idx < len(states):
        color = STATE_COLORS.get(states[start_idx], 'white')
        ax3.axvspan(times[start_idx], times[-1], color=color, alpha=0.7, lw=0)

    # Custom legend for background colors
    handles, labels = ax3.get_legend_handles_labels()
    handles.extend([mpatches.Patch(color=c, alpha=0.7, label=s) for s, c in STATE_COLORS.items()])
    ax3.legend(handles=handles, loc='upper left', fontsize=9, ncol=2)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(os.path.join(OUTPUT_DIR, f"dashboard_time_{scenario_name}.png"), dpi=300)
    plt.close()

def plot_3d_variable(df, y_col, y_label, file_suffix, scenario_name):
    """Creates a 3D surface plot mapping Distance and a given Y-variable to Total Risk."""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    x_data = df['Distance_m']
    y_data = df[y_col]
    z_data = df['Total_Risk']
    
    try:
        # First attempt: Try to plot a continuous 3D surface normally
        surf = ax.plot_trisurf(x_data, y_data, z_data, 
                               cmap='jet', edgecolor='none', alpha=0.85, linewidth=0, antialiased=True)
        is_surface = True
    except Exception as e:
        print(f"  -> ⚠️ Triangulation failed for {y_col}. Applying micro-jitter to fix the surface...")
        try:
            # Second attempt: Apply a microscopic, invisible jitter to X and Y to break the perfect line
            x_jitter = x_data + np.random.normal(0, 1e-5, size=len(x_data))
            y_jitter = y_data + np.random.normal(0, 1e-5, size=len(y_data))
            
            surf = ax.plot_trisurf(x_jitter, y_jitter, z_data, 
                                   cmap='jet', edgecolor='none', alpha=0.85, linewidth=0, antialiased=True)
            is_surface = True
        except Exception as e2:
            # Absolute fallback to Scatter if it somehow still fails
            print(f"  -> ❌ Jitter failed. Falling back to 3D Scatter.")
            surf = ax.scatter(x_data, y_data, z_data, 
                              c=z_data, cmap='jet', s=40, edgecolor='k', alpha=0.8, vmin=0, vmax=1)
            is_surface = False

    # Invert the X axis so it looks like the rover is approaching the rock
    ax.invert_xaxis()
    
    plot_type = "Surface" if is_surface else "Scatter"
    ax.set_title(f"3D Risk {plot_type} ({y_label}) | {scenario_name.capitalize()} Conditions", fontsize=14, fontweight='bold')
    ax.set_xlabel("Distance to Obstacle (m)", fontweight='bold')
    ax.set_ylabel(y_label, fontweight='bold')
    ax.set_zlabel("Total Expected Risk", fontweight='bold')
    
    # Force Z-axis to be 0 to 1 for consistency
    ax.set_zlim(0, 1.05)
    
    # Add a colorbar for clarity
    cbar = plt.colorbar(surf, ax=ax, shrink=0.5, aspect=10)
    cbar.set_label("Total Expected Risk ($R_i$)", fontweight='bold')
    
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, f"3d_{file_suffix}_{scenario_name}.png"), dpi=300)
    plt.close(fig)

def plot_platt_scaling(all_data_df):
    """Plots the Sigmoid calibration curve using data from all scenarios combined."""
    fig, ax = plt.subplots(figsize=(7, 6))
    
    # Sort by confidence so the line plots smoothly
    all_data_df = all_data_df.sort_values(by='Confidence')
    
    ax.scatter(all_data_df['Confidence'], all_data_df['P_Obstacle'], color='blue', s=20, alpha=0.5, label='E3 Measurements')
    ax.plot(all_data_df['Confidence'], all_data_df['P_Obstacle'], color='red', linewidth=2, label='Fitted Sigmoid (Platt)')
    
    ax.set_title("Platt Scaling: Confidence to Probability Calibration", fontsize=14, fontweight='bold')
    ax.set_xlabel("Raw Confidence Score ($s_i$)", fontweight='bold')
    ax.set_ylabel("Calibrated Probability ($P(Obstacle)$)", fontweight='bold')
    ax.grid(True, linestyle='--', alpha=0.5)
    ax.legend(loc='lower right')
    
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "platt_scaling_curve.png"), dpi=300)
    plt.close()

def main():
    if not csv_files:
        print(f"❌ No CSV files found in {CSV_DIR}")
        return

    print("Generating E3 Advanced Visualizations...")
    
    all_dfs = []

    for file in csv_files:
        scenario_name = os.path.basename(file).replace("e3_math_", "").replace(".csv", "")
        print(f"  -> Processing {scenario_name}...")
        
        df = pd.read_csv(file)
        if df.empty:
            continue
            
        # --- Normalize Time to Start at 0 ---
        df['Sim_Time'] = df['Sim_Time'] - df['Sim_Time'].iloc[0]
        
        all_dfs.append(df)
        
        # 1. Generate Dashboard (Time-based)
        plot_dashboard(df, scenario_name)
        
        # 2. Generate 3D Plots (Distance vs. P_Obstacle and Distance vs. K_Collision)

    # 3. Generate Platt Scaling Curve
    if all_dfs:
        combined_df = pd.concat(all_dfs, ignore_index=True)
        plot_platt_scaling(combined_df)

    print(f"\n✅ All E3 plots saved successfully to:\n{OUTPUT_DIR}")

if __name__ == "__main__":
    main()
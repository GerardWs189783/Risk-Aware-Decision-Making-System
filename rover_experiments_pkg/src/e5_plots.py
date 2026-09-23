#!/usr/bin/env python3
import os
import glob
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# --- 1. GLOBAL SETTINGS & FONT ---
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.size'] = 12
plt.rcParams['axes.titlesize'] = 14
plt.rcParams['axes.labelsize'] = 14
plt.rcParams['legend.fontsize'] = 11

# --- 2. CONFIGURATION ---
CSV_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_5")
OUTPUT_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/e5_explainability")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Restored original color mapping for BT states
STATE_COLORS = {
    'MONITORING': '#d4edda',  # Light Green
    'PLANNING': '#fff3cd',    # Light Yellow
    'DANGER': '#f8d7da'       # Light Red
}

def plot_explainability_trace(df, scenario_name):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
    fig.suptitle(f"Explainability Trace | Map $R_2$, Path $W_2$, {scenario_name.capitalize()} conditions", fontsize=18, fontweight='bold', y=0.96)
    
    # 1. TOP PANE: Perception & Calibration
    ax1.plot(df['Sim_Time'], df['Confidence'], label=r'Raw Confidence ($s_i$)', color='gray', alpha=0.6, linestyle='--', linewidth=2, zorder=3)
    ax1.plot(df['Sim_Time'], df['P_Obstacle'], label=r'Calibrated $P_{\text{obstacle}}$', color='blue', linewidth=2.5, zorder=3)
    ax1.set_ylabel("Probability", fontweight='bold')
    ax1.set_ylim(-0.05, 1.05)
    ax1.legend(loc='upper right')

    # 2. MIDDLE PANE: Spatial Relevance & Severity
    ax2.plot(df['Sim_Time'], df['K_Collision'], label=r'Collision Proximity Kernel ($K_{\text{col}}$)', color='purple', linewidth=2.5, zorder=3)
    ax2.plot(df['Sim_Time'], df['C_Severity'], label=r'Collision Severity Index ($C_i$)', color='darkorange', linewidth=2.5, zorder=3)
    ax2.set_ylabel("Factor Value", fontweight='bold')
    ax2.set_ylim(-0.05, 1.05)
    ax2.legend(loc='upper right')

    # 3. BOTTOM PANE: Total Risk & BT States
    ax3.plot(df['Sim_Time'], df['Total_Risk'], label=r'Collision Expected Risk ($R^*$)', color='red', linewidth=3, zorder=3)
    ax3.axhline(y=0.30, color='darkorange', linestyle='--', alpha=0.8, linewidth=2, label=r'$\tau_1$ (Plan)', zorder=2)
    ax3.axhline(y=0.75, color='darkred', linestyle='--', alpha=0.8, linewidth=2, label=r'$\tau_2$ (Danger)', zorder=2)
    ax3.set_ylabel("Expected Risk", fontweight='bold')
    ax3.set_xlabel("Mission Elapsed Time (s)", fontweight='bold')
    ax3.set_ylim(-0.05, 1.05)
    ax3.legend(loc='upper right')
    
    max_time = df['Sim_Time'].max()
    ax3.set_xlim(0, max_time)

    # --- ADD BT STATE BACKGROUND SPANS TO ALL 3 AXES ---
    times = df['Sim_Time'].values
    states = df['BT_State'].values
    start_idx = 0
    
    for i in range(1, len(states)):
        if states[i] != states[start_idx]:
            state = states[start_idx]
            color = STATE_COLORS.get(state, 'white')
            
            t_start = times[start_idx]
            t_end = times[i]
            
            # Artificial widening for ultra-brief states to ensure visibility
            if state in ['PLANNING', 'DANGER'] and (t_end - t_start) < 2.0:
                t_end = t_start + 2.0 
                
            for ax in [ax1, ax2, ax3]:
                ax.axvspan(t_start, t_end, color=color, alpha=0.9, lw=0, zorder=0)
                
            start_idx = i
            
    # Final span to the end
    if start_idx < len(states):
        state = states[start_idx]
        color = STATE_COLORS.get(state, 'white')
        t_start = times[start_idx]
        t_end = times[-1]
        
        if state in ['PLANNING', 'DANGER'] and (t_end - t_start) < 2.0:
            t_end = t_start + 2.0
            
        for ax in [ax1, ax2, ax3]:
            ax.axvspan(t_start, t_end, color=color, alpha=0.9, lw=0, zorder=0)

    # Formatting Grids & Legends
    for ax in [ax1, ax2, ax3]:
        ax.grid(True, linestyle='--', alpha=0.5, zorder=1)

    handles, labels = ax3.get_legend_handles_labels()
    handles.extend([mpatches.Patch(color=c, alpha=0.9, label=s.capitalize()) for s, c in STATE_COLORS.items()])
    ax3.legend(handles=handles, loc='upper right', ncol=2)

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    
    output_path = os.path.join(OUTPUT_DIR, f"e5_trace_{scenario_name}.png")
    plt.savefig(output_path, dpi=300)
    plt.close()
    print(f"  -> Saved plot to: {output_path}")

def main():
    csv_files = glob.glob(os.path.join(CSV_DIR, "*.csv"))
    if not csv_files:
        print(f"❌ No CSV files found in {CSV_DIR}")
        return

    print("Generating E5 Explainability Plots...")

    for file in csv_files:
        scenario_name = os.path.basename(file).replace("e5_trace_R2_W2_", "").replace(".csv", "")
        
        df = pd.read_csv(file)
        if df.empty:
            continue
            
        # Normalize time so the plot always starts nicely at t=0
        df['Sim_Time'] = df['Sim_Time'] - df['Sim_Time'].iloc[0]
        
        plot_explainability_trace(df, scenario_name)

    print("\n✅ All E5 trace plots generated successfully!")

if __name__ == "__main__":
    main()
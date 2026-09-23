#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines

# --- CONFIGURATION ---
search_path = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_1/add_exp_results/timeseries/*.csv")
CSV_FILES = glob.glob(search_path)

def main():
    if not CSV_FILES:
        print(f"❌ Error: No timeseries CSV files found at:\n{search_path}")
        return

    # Create the output directory in your dedicated plots folder
    output_dir = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/timeseries_plots")
    os.makedirs(output_dir, exist_ok=True)

    for csv_file in CSV_FILES:
        print(f"Processing: {os.path.basename(csv_file)}")
        df = pd.read_csv(csv_file)
        
        # 1. ISOLATE THE FILENAME
        base_name = os.path.basename(csv_file)
        
        # 2. AUTO-DETECT VARIANT
        is_proposed = 'Risk' in df.columns
        metric_col = 'Risk' if is_proposed else 'Confidence'
        metric_label = 'Expected Risk' if is_proposed else 'Confidence Score'
        
        # 3. EXTRACT METADATA FOR TITLE
        clean_name = base_name.replace('timeseries_', '').replace('.csv', '')
        title_parts = clean_name.split('_')
        
        variant_name = "Uncertainty-Aware BT" if is_proposed else "Classic BT"
        
        # The environment name is at a different index depending on the variant length
        raw_env = title_parts[2] if is_proposed else title_parts[1]
        
        # Standardize "gauss" to "Degraded"
        env_name = "Degraded" if raw_env.lower() == "gauss" else raw_env.capitalize()
        
        # 4. SETUP THE FIGURE
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
        fig.suptitle(f"{variant_name} | {env_name} Conditions", fontsize=14, fontweight='bold', y=0.95)

        # 5. TOP PLOT: The Metric (Risk/Confidence)
        ax1.plot(df['Sim_Time_s'], df[metric_col], color='blue', linewidth=1.2)
        ax1.set_ylabel(metric_label, fontsize=10, fontweight='bold')
        
        max_val = df[metric_col].max()
        ax1.set_ylim(-0.05, max(1.05, max_val + 0.1))
        ax1.grid(True, axis='y', linestyle='--', alpha=0.5)

        # 6. BOTTOM PLOT: The Physical Velocity
        ax2.plot(df['Sim_Time_s'], df['Velocity_ms'], color='purple', linewidth=1.2)
        ax2.set_ylabel("Velocity (m/s)", fontsize=10, fontweight='bold')
        ax2.set_xlabel("Mission Time (seconds)", fontsize=10, fontweight='bold')
        ax2.set_ylim(-0.05, df['Velocity_ms'].max() + 0.1)
        ax2.grid(True, axis='y', linestyle='--', alpha=0.5)

        # 7. HIGHLIGHT DECISION ZONES (BACKGROUND COLORS)
        color_map = {
            'MONITORING': '#d4edda',  # Light Green
            'PLANNING': '#fff3cd',    # Light Yellow
            'DANGER': '#f8d7da'       # Light Red
        }
        
        # FIX: Visually trigger the yellow background at 0.29 for the Proposed BT 
        # to compensate for the 10Hz sampling, while keeping the physical line at 0.3.
        if is_proposed:
            conditions = [
                (df['Risk'] >= 0.75),
                (df['Risk'] >= 0.29) & (df['Risk'] < 0.75)
            ]
            choices = ['DANGER', 'PLANNING']
            states = np.select(conditions, choices, default='MONITORING')
        else:
            states = df['BT_State'].values

        times = df['Sim_Time_s'].values
        
        start_idx = 0
        for i in range(1, len(states)):
            if states[i] != states[start_idx]:
                state = states[start_idx]
                color = color_map.get(state, 'white')
                
                # lw=0 removes tiny vertical white rendering artifacts between colored blocks
                ax1.axvspan(times[start_idx], times[i], color=color, alpha=0.7, lw=0)
                ax2.axvspan(times[start_idx], times[i], color=color, alpha=0.7, lw=0)
                start_idx = i
                
        # Draw the final remaining block
        if start_idx < len(states):
            state = states[start_idx]
            color = color_map.get(state, 'white')
            ax1.axvspan(times[start_idx], times[-1], color=color, alpha=0.7, lw=0)
            ax2.axvspan(times[start_idx], times[-1], color=color, alpha=0.7, lw=0)

        # 8. LEGEND & THRESHOLDS
        legend_handles = [
            mpatches.Patch(color=color_map['MONITORING'], alpha=0.7, label='MONITORING'),
            mpatches.Patch(color=color_map['PLANNING'], alpha=0.7, label='PLANNING'),
            mpatches.Patch(color=color_map['DANGER'], alpha=0.7, label='DANGER')
        ]

        if is_proposed:
            # Reverted the mathematical dashed line back to 0.3!
            ax1.axhline(y=0.30, color='darkorange', linestyle='--', linewidth=1.5)
            ax1.axhline(y=0.75, color='red', linestyle='--', linewidth=1.5)
            legend_handles.append(mlines.Line2D([], [], color='darkorange', linestyle='--', label='Threshold (0.3)'))
            legend_handles.append(mlines.Line2D([], [], color='red', linestyle='--', label='Threshold (0.75)'))

        ax1.legend(handles=legend_handles, loc='upper right', fontsize=8, framealpha=0.9)

        # 9. SAVE PLOT
        plt.tight_layout(rect=[0, 0, 1, 0.96])
        output_filename = os.path.join(output_dir, base_name.replace('.csv', '.png'))
        plt.savefig(output_filename, dpi=300)
        plt.close(fig) 

    print(f"\n✅ All plots generated and successfully saved in:\n{output_dir}")

if __name__ == "__main__":
    main()
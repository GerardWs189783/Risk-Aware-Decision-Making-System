#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
BASE_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_1")
CSV_DIR = os.path.join(BASE_DIR, "metrics_results")
TRAJ_DIR = os.path.join(BASE_DIR, "trajectory")

# Dedicated output directory for these master plots
OUTPUT_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/consolidated_bars")
os.makedirs(OUTPUT_DIR, exist_ok=True)

def calculate_trajectory_distance(row):
    """Calculates the total distance driven by reading the corresponding trajectory CSV."""
    traj_filename = f"traj_{row['Variant']}_{row['Scenario']}_{row['Map_Difficulty']}_{row['Waypoints']}_run{row['Run_ID']}.csv"
    traj_path = os.path.join(TRAJ_DIR, traj_filename)
    
    if not os.path.exists(traj_path):
        return 0.0
        
    try:
        df_traj = pd.read_csv(traj_path)
        path_data = df_traj[df_traj['type'] == 'path']
        if len(path_data) < 2:
            return 0.0
            
        x = path_data['x'].values
        y = path_data['y'].values
        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
        return np.sum(distances)
    except Exception:
        return 0.0

def aggregate_all_data(df):
    """Aggregates all data across waypoints, grouping by Map, Scenario, and Variant."""
    records = []
    maps = ['easy', 'medium', 'hard']
    map_labels = {'easy': 'R_1', 'medium': 'R_2', 'hard': 'R_3'}
    
    scenarios = ['ideal', 'gauss', 'storm']
    scen_labels = {'ideal': 'Ideal', 'gauss': 'Degraded', 'storm': 'Storm'}
    
    for m in maps:
        for s in scenarios:
            for v in ['classic', 'uncertainty_aware']:
                subset = df[(df['Map_Difficulty'] == m) & (df['Scenario'] == s) & (df['Variant'] == v)]
                if subset.empty:
                    continue
                    
                # 1. Mission Success Rate (%)
                success = subset['Success'].mean() * 100
                
                # 2. Collision Rate
                dist = subset['Distance_m'].sum()
                cols = subset['Collisions'].sum()
                col_rate = (cols / dist) if dist > 0 else 0.0
                
                # 3. Unsafe States
                unsafe = subset['Unsafe_States'].mean()
                
                # 4. Tick time
                tick = subset['Mean_Tick_ms'].mean()
                
                records.append({
                    'Map': map_labels[m],
                    'Scenario': scen_labels[s],
                    'Variant': v,
                    'Success_Rate': success,
                    'Collision_Rate': col_rate,
                    'Unsafe_States': unsafe,
                    'Tick_Time': tick
                })
                
    return pd.DataFrame(records)

def plot_consolidated_metric(agg_df, metric_col, ylabel, title, filename_slug):
    """Generates the 3-tier master plot for a specific metric."""
    fig, ax = plt.subplots(figsize=(14, 6))
    
    maps = ['R_1', 'R_2', 'R_3']
    scenarios = ['Ideal', 'Degraded', 'Storm']
    
    # Custom spacing mechanics to group bars beautifully
    current_x = 0
    bar_width = 0.4
    x_ticks = []
    x_labels = []
    
    # Define Y-axis bounds
    y_max = agg_df[metric_col].max()
    if metric_col == 'Success_Rate':
        ax.set_ylim(0, 105)
    elif y_max > 0:
        ax.set_ylim(0, y_max * 1.15) # 15% headroom for legend
        
    color_c = '#4169E1' # Royal Blue (Classic)
    color_p = '#FF8C00' # Dark Orange (Proposed)
    
    for m in maps:
        for s in scenarios:
            df_c = agg_df[(agg_df['Map'] == m) & (agg_df['Scenario'] == s) & (agg_df['Variant'] == 'classic')]
            df_p = agg_df[(agg_df['Map'] == m) & (agg_df['Scenario'] == s) & (agg_df['Variant'] == 'uncertainty_aware')]
            
            val_c = df_c[metric_col].values[0] if not df_c.empty else 0
            val_p = df_p[metric_col].values[0] if not df_p.empty else 0
            
            # Plot the paired bars
            ax.bar(current_x - bar_width/2, val_c, width=bar_width, color=color_c, edgecolor='black', zorder=3,
                   label='Classic BT' if current_x == 0 else "")
            ax.bar(current_x + bar_width/2, val_p, width=bar_width, color=color_p, edgecolor='black', zorder=3,
                   label='Proposed BT (Uncertainty-Aware)' if current_x == 0 else "")
                   
            x_ticks.append(current_x)
            x_labels.append(f"${m}$\n{s}")
            
            current_x += 1.2 # Spacing between scenarios (Ideal -> Degraded -> Storm)
            
        current_x += 1.0 # Extra spacing between map groups (R1 -> R2)
        
    # Formatting X and Y axes
    ax.set_xticks(x_ticks)
    ax.set_xticklabels(x_labels, fontsize=19)
    ax.set_ylabel(ylabel, fontsize=21, fontweight='bold')
    ax.set_title(title, fontsize=24, fontweight='bold', pad=15)
    ax.grid(axis='y', linestyle='--', alpha=0.7, zorder=0)
    ax.legend(fontsize=16, loc='upper right', framealpha=0.9)
    
    # Add visual vertical dividers between Map groups
    if len(x_ticks) >= 9:
        div_1 = (x_ticks[2] + x_ticks[3]) / 2
        div_2 = (x_ticks[5] + x_ticks[6]) / 2
        ax.axvline(div_1, color='gray', linestyle=':', linewidth=1.5, alpha=0.5, zorder=0)
        ax.axvline(div_2, color='gray', linestyle=':', linewidth=1.5, alpha=0.5, zorder=0)
    
    # Save the plot
    plt.tight_layout()
    filepath = os.path.join(OUTPUT_DIR, f"{filename_slug}.png")
    plt.savefig(filepath, dpi=300)
    plt.close()

def main():
    all_files = glob.glob(os.path.join(CSV_DIR, "*.csv"))
    if not all_files:
        print(f"No CSV files found in {CSV_DIR}")
        return
        
    print("Loading data and dynamically calculating trajectory distances...")
    df = pd.concat((pd.read_csv(f) for f in all_files), ignore_index=True)
    df['Distance_m'] = df.apply(calculate_trajectory_distance, axis=1)
    
    print("\nAggregating metrics across all maps, scenarios, and architectures...")
    agg_df = aggregate_all_data(df)
    
    metrics = {
        'Success_Rate': ('Mission Success Rate (%)', 'Aggregated Mission Success Rate', 'bar_consolidated_success'),
        'Collision_Rate': ('Collision Rate (per meter)', 'Aggregated Collision Rate', 'bar_consolidated_collision'),
        'Unsafe_States': ('Unsafe State Count (Mean per run)', 'Aggregated Unsafe State Count', 'bar_consolidated_unsafe'),
        'Tick_Time': ('Mean BT Tick Time (ms)', 'Aggregated BT Tick Time', 'bar_consolidated_tick')
    }
    
    print(f"\n==================================================")
    print(f"GENERATING 4 MASTER BAR CHARTS")
    print(f"==================================================\n")
    
    for metric_col, (ylabel, title, slug) in metrics.items():
        print(f"-> Generating {title}...")
        plot_consolidated_metric(agg_df, metric_col, ylabel, title, slug)
        
    print(f"\n✅ All 4 master plots successfully saved in:\n{OUTPUT_DIR}")

if __name__ == "__main__":
    main()
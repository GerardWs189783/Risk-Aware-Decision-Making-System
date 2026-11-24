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
MAP_DIFFICULTY = "medium"

# Output directory for the bar charts
OUTPUT_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/bar_charts")
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

def aggregate_metrics(df, variant_name):
    """Aggregates metrics for a specific variant into a clean DataFrame for plotting."""
    df_var = df[df['Variant'] == variant_name].copy()
    
    scenarios = ['ideal', 'gauss', 'storm']
    waypoints = ['wp1', 'wp2', 'wp3']
    
    records = []
    
    for sc in scenarios:
        for wp in waypoints:
            subset = df_var[(df_var['Scenario'] == sc) & (df_var['Waypoints'] == wp)]
            if subset.empty:
                continue
                
            # 1. Mission Success Rate (%)
            success_rate = subset['Success'].mean() * 100
            
            # 2. Collision Rate
            total_collisions = subset['Collisions'].sum()
            total_distance_all = subset['Distance_m'].sum()
            col_rate = (total_collisions / total_distance_all) if total_distance_all > 0 else 0.0
            
            # 3. Unsafe State Count
            unsafe_mean = subset['Unsafe_States'].mean()
            
            # 4. Tick Time (ms)
            tick_mean = subset['Mean_Tick_ms'].mean()
            
            # Formatting Labels
            sc_label = "Degraded" if sc == "gauss" else sc.capitalize()
            wp_label = f"W{wp[-1]}"
            
            records.append({
                'Scenario': sc_label,
                'Path': wp_label,
                'Success_Rate': success_rate,
                'Collision_Rate': col_rate,
                'Unsafe_States': unsafe_mean,
                'Tick_Time': tick_mean
            })
            
    return pd.DataFrame(records)

def plot_grouped_bar(agg_df, metric_col, ylabel, title, filename_suffix, y_max):
    """Generates and saves a grouped bar chart with a fixed Y-axis limit."""
    if agg_df.empty:
        return
        
    # Pivot the data so Scenarios are rows and Paths are columns
    pivot_df = agg_df.pivot(index='Scenario', columns='Path', values=metric_col)
    
    # Ensure correct order on the X-axis
    scenario_order = ['Ideal', 'Degraded', 'Storm']
    existing_scenarios = [s for s in scenario_order if s in pivot_df.index]
    pivot_df = pivot_df.reindex(existing_scenarios)
    
    # Plotting
    ax = pivot_df.plot(kind='bar', figsize=(8, 6), edgecolor='black', zorder=3, colormap='viridis')
    
    # Set the fixed Y-axis limit for 1:1 visual comparison
    plt.ylim(0, y_max)
    
    plt.title(title, fontsize=14, fontweight='bold', pad=15)
    plt.ylabel(ylabel, fontsize=20, fontweight='bold')
    plt.xlabel("Environmental Conditions", fontsize=20, fontweight='bold')
    plt.xticks(rotation=0)
    plt.grid(axis='y', linestyle='--', alpha=0.7, zorder=0)
    
    # Format Legend
    plt.legend(title='Waypoint Path', title_fontsize=10, fontsize=10, loc='best')
    
    plt.tight_layout()
    
    # Save Figure
    filepath = os.path.join(OUTPUT_DIR, f"{filename_suffix}.png")
    plt.savefig(filepath, dpi=300)
    plt.close()

def main():
    all_files = glob.glob(os.path.join(CSV_DIR, "*.csv"))
    if not all_files:
        print(f"No CSV files found in {CSV_DIR}")
        return
        
    df = pd.concat((pd.read_csv(f) for f in all_files), ignore_index=True)
    
    print("Calculating distances from trajectory files...")
    df['Distance_m'] = df.apply(calculate_trajectory_distance, axis=1)
    
    # Filter for the target map difficulty
    df_map = df[df['Map_Difficulty'] == MAP_DIFFICULTY]
    
    print(f"\n==================================================")
    print(f"GENERATING ALIGNED BAR CHARTS FOR MAP: {MAP_DIFFICULTY.upper()}")
    print(f"==================================================\n")
    
    variants = {
        'classic': 'Classic BT',
        'uncertainty_aware': 'Proposed BT (Uncertainty-Aware)'
    }
    
    metrics = {
        'Success_Rate': ('Mission Success Rate (%)', 'mission_success'),
        'Collision_Rate': ('Collision Rate (per meter)', 'collision_rate'),
        'Unsafe_States': ('Unsafe State Count (Mean per run)', 'unsafe_states'),
        'Tick_Time': ('Mean BT Tick Time (ms)', 'tick_time')
    }
    
    # 1. Pre-calculate the aggregated dataframes for BOTH variants
    agg_dfs = {}
    for var_key in variants.keys():
        agg_dfs[var_key] = aggregate_metrics(df_map, var_key)
        
    # 2. Iterate through metrics, find the global maximum, and plot both
    for metric_key, (ylabel, file_slug) in metrics.items():
        
        # Find the absolute maximum across both Classic and Proposed for this specific metric
        global_max = 0
        for var_key in variants.keys():
            if not agg_dfs[var_key].empty:
                max_val = agg_dfs[var_key][metric_key].max()
                global_max = max(global_max, max_val)
                
        # Determine a clean y-axis limit
        if metric_key == 'Success_Rate':
            y_max = 105.0 # Fixed for percentages
        elif global_max == 0:
            y_max = 1.0   # Fallback if both are perfectly 0
        else:
            y_max = global_max * 1.15 # Add 15% visual headroom for the legend
            
        print(f"Plotting {metric_key} with aligned Y-max: {y_max:.2f}")
            
        # Plot both variants using the synchronized y_max
        for var_key, var_title in variants.items():
            title = f"{var_title} - {ylabel.split('(')[0].strip()}"
            filename = f"bar_{file_slug}_{var_key}_{MAP_DIFFICULTY}"
            
            plot_grouped_bar(agg_dfs[var_key], metric_key, ylabel, title, filename, y_max)
            
    print(f"\n✅ All 8 aligned bar charts generated and saved in:\n{OUTPUT_DIR}")

if __name__ == "__main__":
    main()
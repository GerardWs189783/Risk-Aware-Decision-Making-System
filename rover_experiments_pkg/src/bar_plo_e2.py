#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# --- CONFIGURATION ---
# Base directories for both experiments
EXP1_BASE = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_1")
EXP2_BASE = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_2")

# Dedicated output directory for ablation study plots
OUTPUT_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/ablation_bars")
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Map exact CSV variant names to their formal titles
VARIANTS = {
    'uncertainty_aware': 'Full Proposed BT',
    'no_platt': 'No Platt scaling',
    'no_pcol': 'No Probability of Collision',
    'no_severity': 'No Severity Index'
}

def calculate_trajectory_distance(row):
    """Calculates total distance, routing to the correct experiment folder."""
    # Determine which folder to look in based on the source tag we add during loading
    base_dir = EXP1_BASE if row['Exp_Source'] == 'exp1' else EXP2_BASE
    traj_path = os.path.join(base_dir, "trajectory", f"traj_{row['Variant']}_{row['Scenario']}_{row['Map_Difficulty']}_{row['Waypoints']}_run{row['Run_ID']}.csv")
    
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

def load_and_merge_data():
    """Loads and filters data from both experiment folders."""
    # 1. Load Experiment 1 (Only Uncertainty Aware, only Medium/Hard)
    exp1_files = glob.glob(os.path.join(EXP1_BASE, "metrics_results", "*.csv"))
    if exp1_files:
        df_exp1 = pd.concat((pd.read_csv(f) for f in exp1_files), ignore_index=True)
        df_exp1['Exp_Source'] = 'exp1'
        df_exp1 = df_exp1[(df_exp1['Variant'] == 'uncertainty_aware') & 
                          (df_exp1['Map_Difficulty'].isin(['medium', 'hard']))]
    else:
        df_exp1 = pd.DataFrame()

    # 2. Load Experiment 2 (Only Ablation variants, only Medium/Hard)
    exp2_files = glob.glob(os.path.join(EXP2_BASE, "metrics_results", "*.csv"))
    if exp2_files:
        df_exp2 = pd.concat((pd.read_csv(f) for f in exp2_files), ignore_index=True)
        df_exp2['Exp_Source'] = 'exp2'
        ablation_variants = ['no_platt', 'no_pcol', 'no_severity']
        df_exp2 = df_exp2[(df_exp2['Variant'].isin(ablation_variants)) & 
                          (df_exp2['Map_Difficulty'].isin(['medium', 'hard']))]
    else:
        df_exp2 = pd.DataFrame()

    # Combine them
    if df_exp1.empty and df_exp2.empty:
        return pd.DataFrame()
        
    return pd.concat([df_exp1, df_exp2], ignore_index=True)

def aggregate_ablation_data(df):
    """Aggregates data grouping by Map, Scenario, and Variant."""
    records = []
    
    # We strictly only care about medium and hard for this comparison
    existing_maps = ['medium', 'hard']
    map_labels = {'medium': 'R_2', 'hard': 'R_3'}
    
    scenarios = ['ideal', 'gauss', 'storm']
    scen_labels = {'ideal': 'Ideal', 'gauss': 'Degraded', 'storm': 'Storm'}
    
    for m in existing_maps:
        for s in scenarios:
            for v_key in VARIANTS.keys():
                subset = df[(df['Map_Difficulty'] == m) & (df['Scenario'] == s) & (df['Variant'] == v_key)]
                
                if subset.empty:
                    success, col_rate, unsafe, tick = 0.0, 0.0, 0.0, 0.0
                else:
                    success = subset['Success'].mean() * 100
                    
                    dist = subset['Distance_m'].sum()
                    cols = subset['Collisions'].sum()
                    col_rate = (cols / dist) if dist > 0 else 0.0
                    
                    unsafe = subset['Unsafe_States'].mean()
                    tick = subset['Mean_Tick_ms'].mean()
                
                records.append({
                    'Map': map_labels[m],
                    'Scenario': scen_labels[s],
                    'Variant': v_key,
                    'Success_Rate': success,
                    'Collision_Rate': col_rate,
                    'Unsafe_States': unsafe,
                    'Tick_Time': tick
                })
                
    return pd.DataFrame(records), existing_maps

def plot_ablation_metric(agg_df, existing_maps, metric_col, ylabel, title, filename_slug):
    """Generates the master plot for the ablation study (2 Maps only)."""
    fig, ax = plt.subplots(figsize=(12, 7)) # Slightly narrower since we dropped R1
    
    map_labels = {'medium': 'R_2', 'hard': 'R_3'}
    scenarios = ['Ideal', 'Degraded', 'Storm']
    
    bar_width = 0.20
    offsets = [-0.3, -0.1, 0.1, 0.3] 
    
    colors = {
        'uncertainty_aware': '#FF8C00',  # Dark Orange (Baseline)
        'no_platt': '#4169E1',           # Royal Blue
        'no_pcol': '#32CD32',            # Lime Green
        'no_severity': '#DC143C'         # Crimson
    }
    
    current_x = 0
    x_ticks = []
    x_labels = []
    
    # Y-axis bounds (Adding 25% headroom so the 2-row legend fits at the top)
    y_max = agg_df[metric_col].max()
    if metric_col == 'Success_Rate':
        ax.set_ylim(0, 130) # Room for legend above 100%
        ax.set_yticks([0, 20, 40, 60, 80, 100])
    elif y_max > 0:
        ax.set_ylim(0, y_max * 1.25) 
        
    for m in existing_maps:
        for s in scenarios:
            m_label = map_labels[m]
            
            for idx, (v_key, v_title) in enumerate(VARIANTS.items()):
                df_v = agg_df[(agg_df['Map'] == m_label) & (agg_df['Scenario'] == s) & (agg_df['Variant'] == v_key)]
                val = df_v[metric_col].values[0] if not df_v.empty else 0
                
                # Plot the bar
                ax.bar(current_x + offsets[idx], val, width=bar_width, color=colors[v_key], 
                       edgecolor='black', zorder=3, label=v_title if current_x == 0 else "")
                   
            x_ticks.append(current_x)
            x_labels.append(f"${m_label}$\n{s}")
            
            current_x += 1.5 
            
        current_x += 1.0 
        
    # Formatting
    ax.set_xticks(x_ticks)
    ax.set_xticklabels(x_labels, fontsize=19)
    ax.set_ylabel(ylabel, fontsize=21, fontweight='bold')
    ax.set_title(title, fontsize=24, fontweight='bold', pad=20)
    ax.grid(axis='y', linestyle='--', alpha=0.7, zorder=0)
    
    # 2-Column legend placed top center
    ax.legend(fontsize=11, loc='upper center', bbox_to_anchor=(0.5, 1.0), ncol=2, framealpha=0.95)
    
    # Add vertical divider between R2 and R3
    if len(x_ticks) == 6:
        div_x = (x_ticks[2] + x_ticks[3]) / 2
        ax.axvline(div_x, color='gray', linestyle=':', linewidth=2, alpha=0.6, zorder=0)
    
    plt.tight_layout()
    filepath = os.path.join(OUTPUT_DIR, f"{filename_slug}.png")
    plt.savefig(filepath, dpi=300)
    plt.close()

def main():
    print("Loading and merging data from Experiment 1 & Experiment 2...")
    df = load_and_merge_data()
    
    if df.empty:
        print("❌ Error: No matching CSV files found in the target directories.")
        return
        
    print("Calculating distances from trajectory files across both experiments...")
    df['Distance_m'] = df.apply(calculate_trajectory_distance, axis=1)
    
    print("\nAggregating ablation metrics for Medium and Hard maps...")
    agg_df, existing_maps = aggregate_ablation_data(df)
    
    metrics = {
        'Success_Rate': ('Mission Success Rate (%)', 'Ablation Study: Mission Success Rate', 'ablation_bar_success'),
        'Collision_Rate': ('Collision Rate (per meter)', 'Ablation Study: Collision Rate', 'ablation_bar_collision'),
        'Unsafe_States': ('Unsafe State Count (Mean per run)', 'Ablation Study: Unsafe State Count', 'ablation_bar_unsafe'),
        'Tick_Time': ('Mean BT Tick Time (ms)', 'Ablation Study: BT Tick Time', 'ablation_bar_tick')
    }
    
    print(f"\n==================================================")
    print(f"GENERATING 4 ABLATION BAR CHARTS (R2 & R3)")
    print(f"==================================================\n")
    
    for metric_col, (ylabel, title, slug) in metrics.items():
        print(f"-> Generating {title}...")
        plot_ablation_metric(agg_df, existing_maps, metric_col, ylabel, title, slug)
        
    print(f"\n✅ All 4 ablation plots successfully saved in:\n{OUTPUT_DIR}")

if __name__ == "__main__":
    main()
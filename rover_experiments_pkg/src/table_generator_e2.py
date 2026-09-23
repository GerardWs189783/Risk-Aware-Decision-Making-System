#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np

# --- CONFIGURATION ---
# UPDATE THIS: Point to your ablation study folder
BASE_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_2") 
CSV_DIR = os.path.join(BASE_DIR, "metrics_results")
TRAJ_DIR = os.path.join(BASE_DIR, "trajectory")
MAP_DIFFICULTY = "medium"

# UPDATE THIS: Map your exact CSV variant names to the formal names for the LaTeX thesis
VARIANTS = {
    'no_platt':'No Platt scaling calibration',
    'no_pcol':'No Probability of Collision',
    'no_severity':'No Severity Index'
}

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

def generate_latex_table(df, variant_key, variant_title):
    """Generates a professional LaTeX table string for a specific BT variant."""
    df_var = df[df['Variant'] == variant_key].copy()
    
    if df_var.empty:
        return f"% No data found for ablation variant: {variant_key}\n"

    scenarios = ['ideal', 'gauss', 'storm']
    waypoints = ['wp1', 'wp2', 'wp3']
    
    map_labels = {"easy": "R_1", "medium": "R_2", "hard": "R_3"}
    r_label = map_labels.get(MAP_DIFFICULTY, MAP_DIFFICULTY)
    
    latex = []
    latex.append(f"% === {variant_title.upper()} TABLE ===")
    latex.append(r"\begin{table}[H]")
    latex.append(r"\centering")
    latex.append(f"\\caption{{Aggregated metrics for {variant_title} on ${r_label}$ rock set map.}}")
    latex.append(f"\\label{{tab:ablation_{variant_key}_{MAP_DIFFICULTY}}}")
    latex.append(r"\resizebox{\textwidth}{!}{")
    latex.append(r"\begin{tabular}{ll c cc cc cccc c}")
    latex.append(r"\toprule")
    latex.append(r"\textbf{Scenario} & \textbf{Path} & \textbf{Success} & \textbf{Time (s)} & \textbf{Dist. (m)} & \textbf{Collisions} & \textbf{Coll. Rate} & \textbf{Unsafe} & \textbf{FP} & \textbf{FN} & \textbf{Tick (ms)} & \textbf{CPU (\%)} \\")
    latex.append(r"\midrule")
    
    for sc in scenarios:
        for idx, wp in enumerate(waypoints):
            subset = df_var[(df_var['Scenario'] == sc) & (df_var['Waypoints'] == wp)]
            if subset.empty:
                continue
                
            success_rate = subset['Success'].mean() * 100
            
            # --- TIME AND DISTANCE (Successful Runs Only) ---
            succ_subset = subset[subset['Success'] == 1]
            if not succ_subset.empty:
                time_mean = succ_subset['Time_s'].mean()
                time_std = succ_subset['Time_s'].std()
                time_str = f"{time_mean:.1f} $\\pm$ {time_std:.1f}" if pd.notna(time_std) and time_std > 0 else f"{time_mean:.1f}"
                
                dist_mean = succ_subset['Distance_m'].mean()
                dist_std = succ_subset['Distance_m'].std()
                dist_str = f"{dist_mean:.1f} $\\pm$ {dist_std:.1f}" if pd.notna(dist_std) and dist_std > 0 else f"{dist_mean:.1f}"
            else:
                time_str = "N/A"
                dist_str = "N/A"
                
            # --- SAFETY METRICS (All Runs) ---
            total_collisions = subset['Collisions'].sum()
            total_distance_all = subset['Distance_m'].sum()
            col_mean = subset['Collisions'].mean()
            col_rate = (total_collisions / total_distance_all) if total_distance_all > 0 else 0.0
                
            unsafe_mean = subset['Unsafe_States'].mean()
            fp_mean = subset['False_Pos'].mean()
            fn_mean = subset['False_Neg'].mean()
            tick_mean = subset['Mean_Tick_ms'].mean()
            cpu_mean = subset['CPU_Pct'].mean()
            
            # --- LABELS ---
            sc_label = "Degraded" if sc == "gauss" else sc.capitalize()
            sc_label = sc_label if idx == 0 else ""
            
            wp_num = wp[-1] 
            path_label = f"$S_{wp_num}, W_{wp_num}$"
            
            row = f"{sc_label} & {path_label} & {success_rate:.0f}\\% & {time_str} & {dist_str} & {col_mean:.1f} & {col_rate:.3f} & {unsafe_mean:.1f} & {fp_mean:.1f} & {fn_mean:.1f} & {tick_mean:.2f} & {cpu_mean:.1f} \\\\"
            latex.append(row)
            
        latex.append(r"\midrule") 
        
    latex.pop() # Remove last midrule
    latex.append(r"\bottomrule")
    latex.append(r"\end{tabular}}")
    latex.append(r"\end{table}")
    latex.append("\n")
    
    return "\n".join(latex)

def main():
    all_files = glob.glob(os.path.join(CSV_DIR, "*.csv"))
    if not all_files:
        print(f"No CSV files found in {CSV_DIR}")
        return
        
    df = pd.concat((pd.read_csv(f) for f in all_files), ignore_index=True)
    
    print("Calculating distances from trajectory files for ablation study...")
    df['Distance_m'] = df.apply(calculate_trajectory_distance, axis=1)
    
    df_map = df[df['Map_Difficulty'] == MAP_DIFFICULTY]
    
    print(f"\n==================================================")
    print(f"GENERATING LATEX TABLES FOR MAP: {MAP_DIFFICULTY.upper()}")
    print(f"==================================================\n")
    
    for var_key, var_title in VARIANTS.items():
        print(generate_latex_table(df_map, var_key, var_title))

if __name__ == "__main__":
    main()
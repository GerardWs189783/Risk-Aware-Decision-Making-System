#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np
import scipy.stats as st

# --- CONFIGURATION ---
BASE_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_1")
CSV_DIR = os.path.join(BASE_DIR, "metrics_results")
TRAJ_DIR = os.path.join(BASE_DIR, "trajectory")
MAP_DIFFICULTY = "hard"

def calculate_trajectory_distance(row):
    """Calculates the total distance driven by reading the corresponding trajectory CSV."""
    traj_filename = f"traj_{row['Variant']}_{row['Scenario']}_{row['Map_Difficulty']}_{row['Waypoints']}_run{row['Run_ID']}.csv"
    traj_path = os.path.join(TRAJ_DIR, traj_filename)
    
    if not os.path.exists(traj_path):
        return 0.0
        
    try:
        df_traj = pd.read_csv(traj_path)
        path_data = df_traj[df_traj['type'] == 'path']
        if len(path_data) < 2: return 0.0
        x, y = path_data['x'].values, path_data['y'].values
        return np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    except Exception:
        return 0.0

def get_stats(data_series, confidence=0.95):
    """Calculates mean, std, and 95% CI for a pandas Series."""
    n = len(data_series)
    if n == 0:
        return 0.0, 0.0, 0.0, 0.0
    
    mean = np.mean(data_series)
    std = np.std(data_series, ddof=1) if n > 1 else 0.0
    
    if n < 2 or std == 0.0:
        return mean, std, mean, mean
        
    # t-distribution for small sample sizes
    se = std / np.sqrt(n)
    h = se * st.t.ppf((1 + confidence) / 2., n-1)
    
    return mean, std, mean - h, mean + h

def generate_main_table(df, variant_name):
    """Generates the standard reading table with Mean/Std and k/n success rate."""
    df_var = df[df['Variant'] == variant_name].copy()
    if df_var.empty: return f"% No data found for variant: {variant_name}\n"

    scenarios = ['ideal', 'gauss', 'storm']
    waypoints = ['wp1', 'wp2', 'wp3']
    map_labels = {"easy": "R_1", "medium": "R_2", "hard": "R_3"}
    r_label = map_labels.get(MAP_DIFFICULTY, MAP_DIFFICULTY)
    variant_title = "Uncertainty Aware" if variant_name == "uncertainty_aware" else "Classic"
    
    latex = [
        f"% === {variant_title.upper()} BT MAIN TABLE ===",
        r"\begin{table}[H]", r"\centering",
        f"\\caption{{Performance metrics for {variant_title} architecture on ${r_label}$ map.}}",
        f"\\label{{tab:e1_{variant_name}_{MAP_DIFFICULTY}}}",
        r"\resizebox{\textwidth}{!}{",
        r"\begin{tabular}{ll c cc cc cccc c}", r"\toprule",
        r"\textbf{Scenario} & \textbf{Path} & \textbf{Success} & \textbf{Time (s)} & \textbf{Dist. (m)} & \textbf{Collisions} & \textbf{Coll. Rate} & \textbf{Unsafe} & \textbf{FP} & \textbf{FN} & \textbf{Tick (ms)} & \textbf{CPU (\%)} \\",
        r"\midrule"
    ]
    
    for sc in scenarios:
        for idx, wp in enumerate(waypoints):
            subset = df_var[(df_var['Scenario'] == sc) & (df_var['Waypoints'] == wp)]
            if subset.empty: continue
            
            n_runs = len(subset)
            n_success = subset['Success'].sum()
            success_str = f"{n_success}/{n_runs} ({(n_success/n_runs)*100:.0f}\\%)"
            
            # Use ONLY successful runs for time/dist
            succ_subset = subset[subset['Success'] == 1]
            if not succ_subset.empty:
                t_mean = succ_subset['Time_s'].mean()
                t_std = succ_subset['Time_s'].std()
                t_str = f"{t_mean:.1f} $\\pm$ {t_std:.1f}" if pd.notna(t_std) and t_std > 0 else f"{t_mean:.1f}"
                
                d_mean = succ_subset['Distance_m'].mean()
                d_std = succ_subset['Distance_m'].std()
                d_str = f"{d_mean:.1f} $\\pm$ {d_std:.1f}" if pd.notna(d_std) and d_std > 0 else f"{d_mean:.1f}"
            else:
                t_str = "N/A"
                d_str = "N/A"
            
            total_collisions = subset['Collisions'].sum()
            total_distance_all = subset['Distance_m'].sum()
            col_rate = (total_collisions / total_distance_all) if total_distance_all > 0 else 0.0
            
            col_mean = subset['Collisions'].mean()
            uns_mean = subset['Unsafe_States'].mean()
            fp_mean = subset['False_Pos'].mean()
            fn_mean = subset['False_Neg'].mean()
            tick_mean = subset['Mean_Tick_ms'].mean()
            cpu_mean = subset['CPU_Pct'].mean()
            
            sc_label = "Degraded" if sc == "gauss" else sc.capitalize()
            sc_label = sc_label if idx == 0 else ""
            path_label = f"$S_{wp[-1]}, W_{wp[-1]}$"
            
            row = f"{sc_label} & {path_label} & {success_str} & {t_str} & {d_str} & {col_mean:.1f} & {col_rate:.3f} & {uns_mean:.1f} & {fp_mean:.1f} & {fn_mean:.1f} & {tick_mean:.2f} & {cpu_mean:.1f} \\\\"
            latex.append(row)
        latex.append(r"\midrule") 
        
    latex.pop()
    latex.extend([r"\bottomrule", r"\end{tabular}}", r"\end{table}", "\n"])
    return "\n".join(latex)

def generate_statistical_comparison_table(df):
    """Generates a dedicated table comparing 95% CIs between architectures."""
    scenarios = ['ideal', 'gauss', 'storm']
    waypoints = ['wp1', 'wp2', 'wp3']
    map_labels = {"easy": "R_1", "medium": "R_2", "hard": "R_3"}
    r_label = map_labels.get(MAP_DIFFICULTY, MAP_DIFFICULTY)
    
    latex = [
        f"% === STATISTICAL COMPARISON TABLE ({r_label}) ===",
        r"\begin{table}[H]", r"\centering",
        f"\\caption{{95\\% Confidence Intervals for key metrics on map ${r_label}$, evaluating $n=5$ runs per configuration.}}",
        f"\\label{{tab:e1_stats_{MAP_DIFFICULTY}}}",
        r"\resizebox{\textwidth}{!}{",
        r"\begin{tabular}{ll l cccccc}", r"\toprule",
        r"\textbf{Scenario} & \textbf{Path} & \textbf{Algorithm} & \textbf{Time 95\% CI} & \textbf{Distance 95\% CI} & \textbf{Collisions 95\% CI} & \textbf{Unsafe 95\% CI} & \textbf{FP 95\% CI} & \textbf{FN 95\% CI} \\",
        r"\midrule"
    ]
    
    for sc in scenarios:
        for idx, wp in enumerate(waypoints):
            sc_label = "Degraded" if sc == "gauss" else sc.capitalize()
            sc_label = sc_label if idx == 0 else ""
            path_label = f"$S_{wp[-1]}, W_{wp[-1]}$"
            
            for alg_key, alg_name in [('classic', 'Classic'), ('uncertainty_aware', 'Proposed')]:
                subset = df[(df['Scenario'] == sc) & (df['Waypoints'] == wp) & (df['Variant'] == alg_key)]
                if subset.empty: continue
                
                # Use ONLY successful runs for Time & Distance CIs
                succ_subset = subset[subset['Success'] == 1]
                if not succ_subset.empty:
                    _, _, t_l, t_u = get_stats(succ_subset['Time_s'])
                    _, _, d_l, d_u = get_stats(succ_subset['Distance_m'])
                    # Clip lower bounds to 0
                    t_l, d_l = max(0, t_l), max(0, d_l)
                    t_str = f"[{t_l:.1f}, {t_u:.1f}]"
                    d_str = f"[{d_l:.1f}, {d_u:.1f}]"
                else:
                    t_str = "N/A"
                    d_str = "N/A"
                
                # Absolute Collisions
                _, _, col_l, col_u = get_stats(subset['Collisions'])
                
                # Unsafe, FP, FN
                _, _, uns_l, uns_u = get_stats(subset['Unsafe_States'])
                _, _, fp_l, fp_u = get_stats(subset['False_Pos'])
                _, _, fn_l, fn_u = get_stats(subset['False_Neg'])
                
                # Clip all non-negative metric lower bounds to 0
                col_l, uns_l, fp_l, fn_l = max(0, col_l), max(0, uns_l), max(0, fp_l), max(0, fn_l)
                
                row = f"{sc_label} & {path_label} & {alg_name} & {t_str} & {d_str} & [{col_l:.1f}, {col_u:.1f}] & [{uns_l:.1f}, {uns_u:.1f}] & [{fp_l:.1f}, {fp_u:.1f}] & [{fn_l:.1f}, {fn_u:.1f}] \\\\"
                latex.append(row)
                sc_label = "" # Clear scenario label for the 'Proposed' row
                path_label = "" # Clear path label for the 'Proposed' row
                
            latex.append(r"\midrule")
            
    latex.pop()
    latex.extend([r"\bottomrule", r"\end{tabular}}", r"\end{table}", "\n"])
    return "\n".join(latex)

def main():
    all_files = glob.glob(os.path.join(CSV_DIR, "*.csv"))
    if not all_files:
        print(f"No CSV files found in {CSV_DIR}")
        return
        
    df = pd.concat((pd.read_csv(f) for f in all_files), ignore_index=True)
    
    print("Calculating distances from trajectory files...")
    df['Distance_m'] = df.apply(calculate_trajectory_distance, axis=1)
    
    df_map = df[df['Map_Difficulty'] == MAP_DIFFICULTY]
    
    print("\n==================================================")
    print(f"GENERATING LATEX TABLES FOR MAP: {MAP_DIFFICULTY.upper()}")
    print("==================================================\n")
    
    print(generate_main_table(df_map, 'classic'))
    print(generate_main_table(df_map, 'uncertainty_aware'))
    print(generate_statistical_comparison_table(df_map))

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import os
import glob
import pandas as pd
import numpy as np

# --- CONFIGURATION ---
BASE_DIR_E4 = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_4")
CSV_DIR_E4 = os.path.join(BASE_DIR_E4, "metrics_results")
TRAJ_DIR_E4 = os.path.join(BASE_DIR_E4, "trajectory")

# Baseline E1 Configuration
CSV_DIR_E1 = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_1/metrics_results")
TRAJ_DIR_E1 = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/experiment_data/experiment_1/trajectory")

PARAM_LABELS = {
    "tau_1": r"$\tau_1$",
    "tau_2": r"$\tau_2$",
    "var_track": r"$\sigma_{\text{track}}^2$",
    "d_safe": r"$d_{\text{safe}}$",
    "t_react": r"$t_{\text{react}}$"
}

# The default baseline values for each parameter
DEFAULT_VALUES = {
    "tau_1": 0.3,
    "tau_2": 0.75,
    "var_track": 0.02,
    "d_safe": 1.5,
    "t_react": 0.5
}

def calculate_e4_distance(row):
    """Calculates the distance driven for E4 runs."""
    param = row.get('Tested_Param', '')
    val = row.get('Param_Value', '')
    run_id = row.get('Run_ID', '')
    
    search_pattern = os.path.join(TRAJ_DIR_E4, f"*{param}_{val}*run{run_id}.csv")
    matches = glob.glob(search_pattern)
    
    if not matches: return 0.0
        
    try:
        df_traj = pd.read_csv(matches[0])
        path_data = df_traj[df_traj['type'] == 'path']
        if len(path_data) < 2: return 0.0
        x, y = path_data['x'].values, path_data['y'].values
        return np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    except Exception:
        return 0.0

def calculate_e1_distance(row):
    """Calculates the distance driven for E1 Baseline runs."""
    run_id = row.get('Run_ID', '')
    search_pattern = os.path.join(TRAJ_DIR_E1, f"*uncertainty_aware_gauss_medium_wp2*run{run_id}*.csv")
    matches = glob.glob(search_pattern)
    
    if not matches: return 0.0
        
    try:
        df_traj = pd.read_csv(matches[0])
        path_data = df_traj[df_traj['type'] == 'path']
        if len(path_data) < 2: return 0.0
        x, y = path_data['x'].values, path_data['y'].values
        return np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))
    except Exception:
        return 0.0

def get_metrics_row(subset, row_label):
    """Helper function to aggregate metrics and return a formatted LaTeX row."""
    success_rate = subset['Success'].mean() * 100
    
    succ_subset = subset[subset['Success'] == 1]
    if not succ_subset.empty:
        time_mean = succ_subset['Time_s'].mean()
        time_std = succ_subset['Time_s'].std()
        time_str = f"{time_mean:.1f} $\\pm$ {time_std:.1f}" if pd.notna(time_std) and time_std > 0 else f"{time_mean:.1f}"
        
        dist_mean = succ_subset['Distance_m'].mean()
        dist_std = succ_subset['Distance_m'].std()
        dist_str = f"{dist_mean:.1f} $\\pm$ {dist_std:.1f}" if pd.notna(dist_std) and dist_std > 0 else f"{dist_mean:.1f}"
    else:
        time_str, dist_str = "N/A", "N/A"
        
    total_collisions = subset['Collisions'].sum()
    total_distance_all = subset['Distance_m'].sum()
    
    col_mean = subset['Collisions'].mean()
    col_rate = (total_collisions / total_distance_all) if total_distance_all > 0 else 0.0
        
    unsafe_mean = subset['Unsafe_States'].mean()
    fp_mean = subset['False_Pos'].mean()
    fn_mean = subset['False_Neg'].mean()
    tick_mean = subset['Mean_Tick_ms'].mean()
    
    return f"{row_label} & {success_rate:.0f}\\% & {time_str} & {dist_str} & {col_mean:.1f} & {col_rate:.3f} & {unsafe_mean:.1f} & {fp_mean:.1f} & {fn_mean:.1f} & {tick_mean:.2f} \\\\"

def load_baseline():
    """Loads and prepares the E1 Baseline Data."""
    pattern = os.path.join(CSV_DIR_E1, "metrics_results_uncertainty_aware_gauss_medium_wp2_run*.csv")
    files = glob.glob(pattern)
    if not files:
        print("⚠️ Warning: Could not find E1 Baseline CSVs.")
        return None
        
    df_e1 = pd.concat((pd.read_csv(f) for f in files), ignore_index=True)
    df_e1['Distance_m'] = df_e1.apply(calculate_e1_distance, axis=1)
    return df_e1

def generate_e4_latex_table(df_e4, param_name, df_baseline):
    """Generates a professional LaTeX table string for a specific E4 parameter."""
    df_var = df_e4[df_e4['Tested_Param'] == param_name].copy()
    
    if df_var.empty:
        return f"% No data found for parameter: {param_name}\n"
        
    df_var = df_var.sort_values(by='Param_Value')
    unique_values = df_var['Param_Value'].unique()
    
    latex_symbol = PARAM_LABELS.get(param_name, param_name)
    default_val = DEFAULT_VALUES.get(param_name, "Unknown")
    
    latex = []
    latex.append(f"% === {param_name.upper()} SENSITIVITY TABLE ===")
    latex.append(r"\begin{table}[H]")
    latex.append(r"\centering")
    latex.append(f"\\caption{{Sensitivity analysis metrics for {latex_symbol} (Map $R_2$, Degraded conditions, Path $W_2$).}}")
    latex.append(f"\\label{{tab:e4_sens_{param_name}}}")
    latex.append(r"\resizebox{\textwidth}{!}{")
    
    latex.append(r"\begin{tabular}{c | c cc cc cccc}")
    latex.append(r"\toprule")
    latex.append(f"\\textbf{{{latex_symbol} Value}} & \\textbf{{Success}} & \\textbf{{Time (s)}} & \\textbf{{Dist. (m)}} & \\textbf{{Collisions}} & \\textbf{{Coll. Rate}} & \\textbf{{Unsafe}} & \\textbf{{FP}} & \\textbf{{FN}} & \\textbf{{Tick (ms)}} \\\\")
    latex.append(r"\midrule")
    
    # 1. ADD BASELINE ROW FIRST
    if df_baseline is not None and not df_baseline.empty:
        baseline_row = get_metrics_row(df_baseline, f"\\textbf{{Baseline ({default_val})}}")
        latex.append(baseline_row)
        latex.append(r"\midrule") # Visual separator between baseline and tests
    
    # 2. ADD EXPERIMENTAL E4 ROWS
    for val in unique_values:
        subset = df_var[df_var['Param_Value'] == val]
        row_str = get_metrics_row(subset, str(val))
        latex.append(row_str)
        
    latex.append(r"\bottomrule")
    latex.append(r"\end{tabular}}")
    latex.append(r"\end{table}")
    latex.append("\n")
    
    return "\n".join(latex)

def main():
    print("Loading E1 Baseline data...")
    df_baseline = load_baseline()
    
    all_files = glob.glob(os.path.join(CSV_DIR_E4, "*.csv"))
    if not all_files:
        print(f"No CSV files found in {CSV_DIR_E4}")
        return
        
    df_e4 = pd.concat((pd.read_csv(f) for f in all_files), ignore_index=True)
    
    print("Calculating distances from E4 trajectory files...")
    df_e4['Distance_m'] = df_e4.apply(calculate_e4_distance, axis=1)
    
    print("\n==================================================")
    print("GENERATING LATEX TABLES FOR E4 SENSITIVITY ANALYSIS")
    print("==================================================\n")
    
    for param in PARAM_LABELS.keys():
        print(generate_e4_latex_table(df_e4, param, df_baseline))

if __name__ == "__main__":
    main()
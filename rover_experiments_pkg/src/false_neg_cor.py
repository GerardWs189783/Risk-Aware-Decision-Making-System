#!/usr/bin/env python3
import os
import glob
import math
import csv
import pandas as pd

# --- CONFIGURATION ---
FOV_ANGLE_DEG = 80.0  # Camera horizontal Field of View
MAX_RANGE = 8.0       # Maximum reliable detection range in meters
MATCH_TOL = 1.5       # Tolerance for matching detection to GT

def get_heading(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

def is_in_fov(rx, ry, heading, tx, ty):
    """Checks if target (tx, ty) is within the FOV of robot at (rx, ry) with 'heading'"""
    dx = tx - rx
    dy = ty - ry
    dist = math.hypot(dx, dy)
    
    if dist > MAX_RANGE or dist < 0.1:
        return False
        
    angle_to_target = math.atan2(dy, dx)
    relative_angle = (angle_to_target - heading + math.pi) % (2 * math.pi) - math.pi
    
    fov_rad = math.radians(FOV_ANGLE_DEG)
    if abs(relative_angle) <= (fov_rad / 2.0):
        return True
    return False

def process_all_experiments():
    base_path = os.path.expanduser('~/ros2_ws/src/rover_experiments_pkg/experiment_data')
    
    search_pattern = os.path.join(base_path, '**', 'traj_*.csv')
    traj_files = glob.glob(search_pattern, recursive=True)
    
    if not traj_files:
        print(f"❌ No trajectory files found in {base_path}")
        return

    print(f"🔍 Found {len(traj_files)} trajectory files. Processing E1, E2, and E4...")
    
    for t_file in traj_files:
        filename = os.path.basename(t_file)
        dir_name = os.path.dirname(t_file)
        parent_dir = os.path.dirname(dir_name)
        
        # 1. Filter out E3 and E5 based on folder path
        if not any(exp in parent_dir for exp in ['experiment_1', 'experiment_2', 'experiment_4']):
            continue

        # 2. Handle the specific E4 filename formatting vs E1/E2
        if "traj_e4_" in filename:
            metrics_filename = filename.replace("traj_e4_", "metrics_e4_")
        else:
            metrics_filename = filename.replace("traj_", "metrics_results_")
            
        m_file = os.path.join(parent_dir, 'metrics_results', metrics_filename)
        
        if not os.path.exists(m_file):
            print(f"⚠️ Metrics file missing for {filename}: Expected {m_file}")
            continue
                
        # 3. Determine Initial Heading
        # Default to wp1 (0.0). Override to 1.57 if wp2/wp3 is in filename OR if it's E4.
        current_heading = 0.0
        if "_wp2_" in filename or "_wp3_" in filename or "experiment_4" in parent_dir:
            current_heading = 1.57
                
        # 4. Read Trajectory Data
        path_pts, gt_rocks, conf_rocks = [], [], []
        with open(t_file, 'r') as f:
            reader = csv.reader(f)
            next(reader) 
            for row in reader:
                if len(row) < 3: continue
                rtype, x, y = row[0], float(row[1]), float(row[2])
                if rtype == 'path': path_pts.append((x, y))
                elif rtype == 'gt_rock': gt_rocks.append((x, y))
                elif rtype == 'confirmed_rock': conf_rocks.append((x, y))
                    
        if not path_pts or not gt_rocks:
            continue
                
        # 5. Sweep FOV to Find Observable GT Rocks
        observable_gt_rocks = set()
        
        for i in range(1, len(path_pts)):
            prev_x, prev_y = path_pts[i-1]
            curr_x, curr_y = path_pts[i]
            
            # Update heading only if moved sufficiently
            dist_moved = math.hypot(curr_x - prev_x, curr_y - prev_y)
            if dist_moved > 0.05:
                current_heading = get_heading(prev_x, prev_y, curr_x, curr_y)
                
            # Check GT rocks against FOV
            for rock_idx, (gx, gy) in enumerate(gt_rocks):
                if rock_idx not in observable_gt_rocks:
                    if is_in_fov(curr_x, curr_y, current_heading, gx, gy):
                        observable_gt_rocks.add(rock_idx)
                        
        # 6. Calculate True False Negatives
        matched_observable = 0
        for obs_idx in observable_gt_rocks:
            gx, gy = gt_rocks[obs_idx]
            
            matched = False
            for cx, cy in conf_rocks:
                if math.hypot(gx - cx, gy - cy) <= MATCH_TOL:
                    matched = True
                    break
            if matched:
                matched_observable += 1
                
        new_false_negatives = len(observable_gt_rocks) - matched_observable
        
        # 7. Safely Update ONLY the False_Neg column
        df = pd.read_csv(m_file)
        if 'False_Neg' in df.columns:
            old_fn = df.at[0, 'False_Neg']
            df.at[0, 'False_Neg'] = new_false_negatives
            df.to_csv(m_file, index=False)
            exp_label = "E4" if "experiment_4" in parent_dir else "E1/E2"
            print(f"✅ [{exp_label}] {filename} | Old FN: {old_fn} ➡️ Obs GT: {len(observable_gt_rocks)} ➡️ New FN: {new_false_negatives}")

if __name__ == "__main__":
    process_all_experiments()
    print("\n🚀 Post-processing complete. All E1, E2, and E4 FNRR metrics updated!")
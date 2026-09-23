#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import os

# --- CONFIGURATION ---
OUTPUT_DIR = os.path.expanduser("~/ros2_ws/src/rover_experiments_pkg/plots_and_figs/experiment_3_plots")
os.makedirs(OUTPUT_DIR, exist_ok=True)

plt.rcParams['axes.labelsize'] = 14

def plot_perception_severity():
    """
    Figure X: Perception-severity relationship
    R_i = P_obstacle * K_col * C_i
    Assuming K_col = 1.0 (Obstacle is directly on the path)
    """
    # Create a grid of P and C_i values from 0 to 1
    P_vals = np.linspace(0, 1, 100)
    C_vals = np.linspace(0, 1, 100)
    P, C = np.meshgrid(P_vals, C_vals)
    
    # Calculate Risk (K_col = 1.0)
    K_col = 1.0
    R = P * K_col * C
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    surf = ax.plot_surface(P, C, R, cmap='jet', edgecolor='none', alpha=0.9, antialiased=True)
    
    ax.set_title("Perception-Severity Relationship ($K_{col} = 1$)", fontsize=16, fontweight='bold')
    ax.set_xlabel("Probability of Obstacle ($P(\t{obstacle})$)", fontweight='bold')
    ax.set_ylabel("Collision Severity Index ($C_i$)", fontweight='bold')
    ax.set_zlabel("Expected Risk ($R_i$)", fontweight='bold')
    ax.set_zlim(0, 1.0)
    
    cbar = plt.colorbar(surf, ax=ax, shrink=0.5, aspect=10)
    cbar.set_label("Expected Risk ($R_i$)", fontweight='bold')
    
    ax.view_init(elev=25, azim=-120)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "parametric_P_vs_C.png"), dpi=300)
    plt.close(fig)
    print("✅ Saved: parametric_P_vs_C.png")

def plot_perception_spatial():
    """
    Figure Y: Perception-spatial relationship
    R_i = P_obstacle * K_col * C_i
    Assuming C_i = 0.5 (Moderate constant speed/distance)
    """
    # Create a grid of P (0 to 1) and d_path (0 to 1.5 meters)
    P_vals = np.linspace(0, 1, 100)
    d_vals = np.linspace(0, 1.5, 100)
    P, d = np.meshgrid(P_vals, d_vals)
    
    # Calculate Gaussian K_col
    # Using a representative standard deviation for the track variance (e.g., sigma = 0.3 meters)
    sigma_track = 0.3 
    K_col = np.exp(-(d**2) / (2 * sigma_track**2))
    
    # Calculate Risk (C_i = 0.5)
    C_i = 0.5
    R = P * K_col * C_i
    
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    surf = ax.plot_surface(P, d, R, cmap='jet', edgecolor='none', alpha=0.9, antialiased=True)
    
    ax.set_title("Perception-Spatial Relationship ($C_i = 0.5$)", fontsize=16, fontweight='bold')
    ax.set_xlabel("Probability of Obstacle ($P(\t{obstacle})$)", fontweight='bold')
    ax.set_ylabel("Path-Obstacle Distance ($\Delta y_i$) [m]", fontweight='bold')
    ax.set_zlabel("Expected Risk ($R_i$)", fontweight='bold')
    ax.set_zlim(0, 0.5) # Max risk here is 0.5 because C_i is capped at 0.5
    
    cbar = plt.colorbar(surf, ax=ax, shrink=0.5, aspect=10)
    cbar.set_label("Total Expected Risk ($R_i$)", fontweight='bold')
    
    # Adjust viewing angle for best perspective on the Gaussian curve
    ax.view_init(elev=25, azim=156)
    plt.tight_layout()
    plt.savefig(os.path.join(OUTPUT_DIR, "parametric_P_vs_dpath.png"), dpi=300)
    plt.close(fig)
    print("✅ Saved: parametric_P_vs_dpath.png")

if __name__ == "__main__":
    print("Generating Theoretical Parametric Plots...")
    #plot_perception_severity()
    plot_perception_spatial()
#!/usr/bin/env python3
import matplotlib.pyplot as plt

def main():
    # Hardest difficulty rocks (17 total)
    hard_rocks = [
        [0.85, 6.88], [-11.98, 7.91], [11.97, 8.31], [25.95, 30.55], [10.0, 13.69],
        [-6.56, 5.8], [-5.0, 19.5], [-6.3, -7.96], [-5.0, 0.5], [-4.20, 6.99],
        [8.2, 2.5], [-13.0, 14.5], [17.0, 19.0], [0.6, 17.69], [-2.0, 22.69],
        [9.0, 11.5], [-12.0, 11.5]
    ]

    # Mission Paths dictionary
    paths = {
        1: {
            'color': 'crimson',
            'points': {'S': [-21.0, 17.0], 'W_1': [-8.0, 15.0], 'W_2': [-7.0, -2.0]}
        },
        2: {
            'color': 'dodgerblue',
            'points': {'S': [-11.0, -12.5], 'W_1': [5.2, 6.5], 'W_2': [5.0, 16.0]}
        },
        3: {
            'color': 'forestgreen',
            'points': {'S': [-13.0, -9.0], 'W_1': [-5.0, 13.0], 'W_2': [-3.0, 20.0]}
        }
    }

    fig, ax = plt.subplots(figsize=(12, 12))
    ax.set_title("Mission Layout Overview", fontsize=28, fontweight='bold', pad=15)

    # 1. Plot the Rocks
    r_x, r_y = zip(*hard_rocks)
    ax.scatter(r_x, r_y, c='dimgray', marker='o', s=400, edgecolor='black', alpha=0.5, label='Obstacles')

    # 2. Plot the Waypoints & Starting Points
    for path_num, data in paths.items():
        color = data['color']
        pts = data['points']
        
        # Extract coordinates
        sx, sy = pts['S']
        w1x, w1y = pts['W_1']
        w2x, w2y = pts['W_2']
        
        # Plot Connecting Lines (Dashed for visual clarity)
        ax.plot([sx, w1x, w2x], [sy, w1y, w2y], c=color, linestyle='--', linewidth=2, alpha=0.6)

        # Plot Start (Square)
        ax.scatter(sx, sy, c=color, marker='s', s=200, edgecolor='black', zorder=5, label=f'Path {path_num} Points')
        # Plot Waypoints (Circles)
        ax.scatter([w1x, w2x], [w1y, w2y], c=color, marker='o', s=200, edgecolor='black', zorder=5)

        # Add Text Labels Offset slightly so they don't cover the markers
        offset = 0.8
        ax.text(sx + offset, sy + offset, f'S{path_num}', fontsize=12, fontweight='bold', color=color, 
                bbox=dict(facecolor='white', edgecolor='none', alpha=0.7, pad=1))
        ax.text(w1x + offset, w1y + offset, f'W{path_num}_1', fontsize=12, fontweight='bold', color=color, 
                bbox=dict(facecolor='white', edgecolor='none', alpha=0.7, pad=1))
        ax.text(w2x + offset, w2y + offset, f'W{path_num}_2', fontsize=12, fontweight='bold', color=color, 
                bbox=dict(facecolor='white', edgecolor='none', alpha=0.7, pad=1))

    # Formatting
    ax.set_xlabel("X coordinate (meters)", fontsize=25)
    ax.set_ylabel("Y coordinate (meters)", fontsize=25)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.set_aspect('equal', adjustable='box')
    
    # Clean up the legend so it only shows 1 entry per path, plus the rocks
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc='best', fontsize=24, framealpha=0.9)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
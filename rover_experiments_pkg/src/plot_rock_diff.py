#!/usr/bin/env python3
import matplotlib.pyplot as plt

def main():
    # Ground Truth Rocks from YAML
    rocks = {
        'Easy': [
            [0.85, 6.88], [-11.98, 7.91], [11.97, 8.31], [25.95, 30.55], [10.0, 13.69]
        ],
        'Medium': [
            [0.85, 6.88], [-11.98, 7.91], [11.97, 8.31], [25.95, 30.55], [10.0, 13.69],
            [-6.56, 5.8], [-5.0, 19.5], [-6.3, -7.96], [-5.0, 0.5]
        ],
        'Hard': [
            [0.85, 6.88], [-11.98, 7.91], [11.97, 8.31], [25.95, 30.55], [10.0, 13.69],
            [-6.56, 5.8], [-5.0, 19.5], [-6.3, -7.96], [-5.0, 0.5], [-4.20, 6.99],
            [8.2, 2.5], [-13.0, 14.5], [17.0, 19.0], [0.6, 17.69], [-2.0, 22.69],
            [9.0, 11.5], [-12.0, 11.5]
        ]
    }


    for difficulty, coords in rocks.items():
        match difficulty:
            case "Easy":
                numb = 1
            case "Medium":
                numb = 2
            case "Hard":
                numb = 3
            case _:
                numb = 0
        ig, ax = plt.subplots(figsize=(8, 8))
        ig.canvas.manager.set_window_title(f'Map: {difficulty}')

        if coords:
            x_vals, y_vals = zip(*coords)
            ax.scatter(x_vals, y_vals, c='dimgray', marker='o', s=150, edgecolor='black', alpha=0.7, label='Obstacles (Rocks)')        

        ax.set_title(f"{difficulty} Rock Placement Map (R{numb})", fontsize=16, fontweight='bold')
        ax.set_xlabel("X (meters)", fontsize=12)
        ax.set_ylabel("Y (meters)", fontsize=12)
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.set_aspect('equal', adjustable='box')
        
        # Lock axes limits to the largest possible spread so all plots scale equally
        ax.set_xlim(-25, 30)
        ax.set_ylim(-15, 35)

        ax.legend(loc='upper left')
        plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    main()
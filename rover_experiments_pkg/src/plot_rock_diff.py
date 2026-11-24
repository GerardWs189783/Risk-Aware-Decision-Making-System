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

    # Create a 2x2 grid. figsize=(12, 12) gives a great square aspect ratio for an A4 page
    fig, axes = plt.subplots(2, 2, figsize=(12, 12))
    fig.canvas.manager.set_window_title('Map Difficulties')

    # Flatten the 2x2 axes matrix into a 1D list so we can loop through it easily
    axs = axes.flatten()

    for i, (difficulty, coords) in enumerate(rocks.items()):
        ax = axs[i]
        numb = i + 1

        if coords:
            x_vals, y_vals = zip(*coords)
            ax.scatter(x_vals, y_vals, c='dimgray', marker='o', s=150, edgecolor='black', alpha=0.7, label='Obstacles (Rocks)')        

        ax.set_title(rf"{difficulty} Rock Placement Map ($R_{numb}$)", fontsize=18, fontweight='bold')
        ax.grid(True, linestyle='--', alpha=0.6)
        ax.set_aspect('equal', adjustable='box')
        
        # Lock axes limits to the largest possible spread so all plots scale equally
        ax.set_xlim(-25, 30)
        ax.set_ylim(-15, 35)

        # Increase the size of the axis numbers to match the larger labels
        ax.tick_params(axis='both', which='major', labelsize=14)

        # Smart Axis Labeling for a 2x2 Grid
        if i == 0: # Top-Left (Easy)
            ax.set_ylabel("Y (meters)", fontsize=16)
            ax.tick_params(labelbottom=False) # Hide X numbers since they share with the bottom
        elif i == 1: # Top-Right (Medium)
            ax.tick_params(labelleft=False) # Hide Y numbers
            ax.set_xlabel("X (meters)", fontsize=16) # Give it an X label since there is no plot below it
        elif i == 2: # Bottom-Left (Hard)
            ax.set_xlabel("X (meters)", fontsize=16)
            ax.set_ylabel("Y (meters)", fontsize=16)

        # Add the legend with an increased font size
        ax.legend(loc='upper left', fontsize=15)

    # Handle the empty 4th subplot (Bottom-Right)
    ax_empty = axs[3]
    ax_empty.axis('off') # completely hides the grid, box, and ticks

    # w_pad=0.1 pulls the columns very close together. h_pad=2.0 keeps comfortable vertical spacing.
    plt.tight_layout(w_pad=0.1, h_pad=2.0)
    plt.show()

if __name__ == "__main__":
    main()
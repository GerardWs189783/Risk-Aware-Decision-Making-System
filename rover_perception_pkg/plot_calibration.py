import numpy as np
import matplotlib.pyplot as plt

# 1. Your Learned Parameters
A = 1.68825
B = 0.02879

# 2. Generate X values (Raw Confidence)
# We use 0.01 to 0.99 to avoid math errors with log(0)
s = np.linspace(0.01, 0.99, 500)

# 3. Calculate Logits (L)
L = np.log(s / (1.0 - s))

# 4. Calculate Calibrated Probability (Y values)
P_obstacle = 1.0 / (1.0 + np.exp(-(A * L + B)))

# 5. Set up the plot for an Academic Paper
plt.figure(figsize=(8, 6), dpi=300) # High DPI for crisp printing

# Plot the ideal "Perfect Calibration" reference line
plt.plot(s, s, linestyle='--', color='gray', label='Perfect Calibration ($P_{obs} = s_i$)')

# Plot your actual Platt Scaling curve
plt.plot(s, P_obstacle, color='blue', linewidth=2.5, 
         label=rf'Platt Scaling ($A={A:.3f}, B={B:.3f}$)')

# Add grid, labels, and title
plt.grid(True, linestyle=':', alpha=0.7)
plt.title('Logistic Sigmoid Function obtained from Platt Scaling calibration method', fontsize=14, pad=15)
plt.xlabel('Raw YOLO Confidence Score ($s_i$)', fontsize=12)
plt.ylabel('Calibrated Probability ($P_{obstacle}$)', fontsize=12)

# Set axis limits strictly from 0 to 1
plt.xlim(0, 1)
plt.ylim(0, 1)

# Add Legend
plt.legend(loc='upper left', fontsize=11)

# 6. Save the plot (PDF is best for LaTeX, PNG is good for quick viewing)
plt.tight_layout()
plt.savefig('calibration_curve.pdf', format='pdf')
plt.savefig('calibration_curve.png', format='png', dpi=300)

print("Plots saved successfully as 'calibration_curve.pdf' and 'calibration_curve.png'!")

# Show the plot on your screen
plt.show()
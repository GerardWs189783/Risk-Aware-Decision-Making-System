import numpy as np
import matplotlib.pyplot as plt

delta_y = np.linspace(-3.0, 3.0, 500)

def conditional_probability(dy, sigma):
    return np.exp(-(dy**2) / (2 * sigma**2))

prob_low  = conditional_probability(delta_y, 0.6)
prob_mid  = conditional_probability(delta_y, 1.0)
prob_high = conditional_probability(delta_y, 1.3)

plt.figure(figsize=(8, 5))

plt.plot(delta_y, prob_low, label=r'$\sigma_{track} = 0.6$ m (Low Variance)', color='green', linewidth=2.5)
plt.plot(delta_y, prob_mid, label=r'$\sigma_{track} = 1.0$ m (Middle Variance)', color='blue', linewidth=2.5)
plt.plot(delta_y, prob_high, label=r'$\sigma_{track} = 1.3$ m (High Variance)', color='red', linewidth=2.5)

plt.fill_between(delta_y, prob_mid, alpha=0.1, color='blue')
plt.fill_between(delta_y, prob_low, alpha=0.1, color='green')
plt.fill_between(delta_y, prob_high, alpha=0.1, color='red')


plt.title('Conditional Collision Probability', fontsize=20, pad=15)
plt.xlabel(r'Lateral Cross-Track Distance $\Delta y_i$ [meters]', fontsize=15)
plt.ylabel(r'$P(\text{collision}_i \mid \text{obstacle}_i)$', fontsize=15)

plt.xlim([-3.0, 3.0])
plt.ylim([0, 1.05])

plt.grid(True, linestyle='--', alpha=0.6)
plt.legend(fontsize=13, loc='upper right', framealpha=0.9)

plt.tight_layout()
plt.savefig('collision_probability_plot.pdf', format='pdf', bbox_inches='tight')
plt.show()
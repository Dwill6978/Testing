import matplotlib.pyplot as plt
import numpy as np

# Example data for plotting
x = np.linspace(0, 10, 100)
y1 = np.sin(x)
y2 = np.cos(x)
y3 = np.sin(2 * x)
y4 = np.exp(-0.1 * x) * np.sin(3 * x)

# Create a 2x3 grid of subplots (total 6)
fig, axs = plt.subplots(2, 3, figsize=(10, 6))
fig.suptitle("4 Subplots and 2 Numeric Displays", fontsize=16)

# --- 4 subplots ---
axs[0, 0].plot(x, y1)
axs[0, 0].set_title("sin(x)")

axs[0, 1].plot(x, y2)
axs[0, 1].set_title("cos(x)")

axs[1, 0].plot(x, y3)
axs[1, 0].set_title("sin(2x)")

axs[1, 1].plot(x, y4)
axs[1, 1].set_title("damped sin(3x)")

# --- 2 number displays ---
value1 = np.mean(y1)
value2 = np.max(y4)

axs[0, 2].axis("off")
axs[1, 2].axis("off")

axs[0, 2].text(0.5, 0.5, f"Mean(sin) = {value1:.3f}", 
               ha="center", va="center", fontsize=14, color="blue")

axs[1, 2].text(0.5, 0.5, f"Max(damped) = {value2:.3f}", 
               ha="center", va="center", fontsize=14, color="red")

plt.tight_layout(rect=[0, 0, 1, 0.96])
plt.show()

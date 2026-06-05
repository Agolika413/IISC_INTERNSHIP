import numpy as np
import matplotlib.pyplot as plt
import run_task1

t = np.linspace(0, 70, 700)
refs = np.array([run_task1.ref(tt)[0][2] for tt in t])

plt.plot(t, np.rad2deg(refs), 'k--')
plt.savefig('ref_plot.png')
print("Saved ref_plot.png")

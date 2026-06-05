import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import square

T_SIM = 70.0
dt = 0.002
N = int(T_SIM/dt)
ta = np.linspace(0, T_SIM, N+1)
td = np.zeros(N+1)
freq = np.pi / 10.0
P_AMP = np.deg2rad(4.0)

for k, t in enumerate(ta):
    td[k] = P_AMP * square(freq * t)

plt.plot(ta, np.rad2deg(td))
plt.savefig('test_square.png')
print("Saved test_square.png")

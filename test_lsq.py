import numpy as np
from scipy.optimize import lsq_linear

Ld = 0.23
c = 7.5e-8 / 2.8e-6
Ba = np.array([
    [0, 0, Ld, Ld, 0, 0, -Ld, -Ld],
    [-Ld, -Ld, 0, 0, Ld, Ld, 0, 0],
    [c, -c, -c, c, c, -c, -c, c],
    [1, 1, 1, 1, 1, 1, 1, 1]
])

Ba_reorder = np.array([
    Ba[3, :],
    Ba[0, :],
    Ba[1, :],
    Ba[2, :]
])

L = np.ones(8)
#L[0]=0
#L[4]=0.6

B_eff = Ba_reorder @ np.diag(L)
nu_d = np.array([35.0, 0.0, 1.0, 0.0])

res = lsq_linear(B_eff, nu_d, bounds=(0.0, 4.5))
print('res.x:', res.x)
print('u_actual:', L * res.x)

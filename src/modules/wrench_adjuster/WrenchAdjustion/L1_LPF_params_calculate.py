import math
import numpy as np
import sympy as smp
from casadi import (DM, SX, cross, dot, fabs, horzcat, inv, norm_2, sqrt,
                    vertcat, is_equal)

def np_inv(m):

    return np.array(inv(DM(m)))

def exp_ATs(A, Ts):

    if np.size(A, 0) != np.size(A, 1):
        print("A is badly defined")
        return
    else:
        dim = np.size(A, 0)

    s = smp.symbols('s')
    t = smp.symbols('t')

    S = s * smp.eye(dim)

    G_s = (S - A)**-1
    G_t = smp.inverse_laplace_transform(G_s, s, t)

    return np.array(G_t.subs(t, Ts), float)

def css2dss(A, B, Ts):

    if np.size(A, 0) != np.size(A, 1):
        print("A is badly defined")
        return
    else:
        dim = np.size(A, 0)

    s = smp.symbols('s')
    t = smp.symbols('t')

    S = s * smp.eye(dim)

    G_s = (S - A)**-1
    G_t = smp.inverse_laplace_transform(G_s, s, t)
    G = G_t.subs(t, Ts)

    H = smp.integrate(G_t, (t, 0, Ts)) @ B

    return np.array(G, float), np.array(H, float)


def get_lpf(omega, Ts):

    dim = np.size(omega)

    A = np.diag(-omega)
    B = np.eye(dim)
    C = np.diag(omega)
    D = np.zeros([dim, dim])

    G, H = css2dss(A, B, Ts)
    return G, H, C, D


# A = params.A
# lpf_omega = params.lpf_omega
# Ts = params.l1_Ts
a = -10
A = np.diag([a, a, a, a, a, a]) # TODO: need further tunning, -10 with 1 works on real hardware
w = 1
lpf_omega = np.array([w, w, w, w, w, w])
Ts = 1/250      # to use with cpp node l1 and mpc rate should be set to 1/100 == ekf_Ts
G, H, C, D = get_lpf(lpf_omega, Ts)

# l1 estimator matrix 6 * 6
E = np_inv(exp_ATs(A, Ts) - np.eye(np.size(A, 0))) @ A @ exp_ATs(A, Ts)

# C 就是omega向量的对角阵
print("C:")
print(C)
# D 就是全零的
print("D:")
print(D)

print("E:")
print(E)

print("G:")
print(G)
print("H:")
print(H)

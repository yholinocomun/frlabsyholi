import numpy as np
from copy import copy

cos = np.cos
sin = np.sin
pi = np.pi


def dh(d, theta, a, alpha):
    """
    Matriz de transformacion homogenea segun la convencion estandar de
    Denavit-Hartenberg (Spong / Craig "distal"):

        T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)

    Entradas (escalares):
        d, theta, a, alpha
    Salida:
        T : np.ndarray 4x4
    """
    ct = cos(theta)
    st = sin(theta)
    ca = cos(alpha)
    sa = sin(alpha)

    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0.0,      sa,       ca,      d],
        [0.0,     0.0,      0.0,    1.0]
    ])
    return T


def fkine_ur5(q):
    """
    Cinematica directa del UR5.
    q : vector numpy [q1, q2, q3, q4, q5, q6] en radianes.
    Retorna T : matriz 4x4 del efector final respecto a la base.

    Tabla DH (Universal Robots, valores en metros y radianes):
        i |  theta  |    d     |    a     |  alpha
        --|---------|----------|----------|--------
        1 |   q1    |  0.08920 |   0      |  pi/2
        2 |   q2    |  0       | -0.425   |  0
        3 |   q3    |  0       | -0.392   |  0
        4 |   q4    |  0.1093  |   0      |  pi/2
        5 |   q5    |  0.09475 |   0      | -pi/2
        6 |   q6    |  0.0825  |   0      |  0
    """
    T1 = dh(0.089159, q[0],  0.000,  pi / 2)
    T2 = dh(0.00000, q[1], -0.425,  0.0)
    T3 = dh(0.00000, q[2], -0.392,  0.0)
    T4 = dh(0.10915, q[3],  0.000,  pi / 2)
    T5 = dh(0.09465, q[4],  0.000, -pi / 2)
    T6 = dh(0.08230, q[5],  0.000,  0.0)

    T = T1 @ T2 @ T3 @ T4 @ T5 @ T6
    return T


# ---- Las funciones rot2quat y TF2xyzquat se mantienen sin cambios ----

def rot2quat(R):
    dEpsilon = 1e-6
    quat = 4 * [0., ]
    quat[0] = 0.5 * np.sqrt(R[0, 0] + R[1, 1] + R[2, 2] + 1.0)
    if (np.fabs(R[0, 0] - R[1, 1] - R[2, 2] + 1.0) < dEpsilon):
        quat[1] = 0.0
    else:
        quat[1] = 0.5 * np.sign(R[2, 1] - R[1, 2]) * np.sqrt(R[0, 0] - R[1, 1] - R[2, 2] + 1.0)
    if (np.fabs(R[1, 1] - R[2, 2] - R[0, 0] + 1.0) < dEpsilon):
        quat[2] = 0.0
    else:
        quat[2] = 0.5 * np.sign(R[0, 2] - R[2, 0]) * np.sqrt(R[1, 1] - R[2, 2] - R[0, 0] + 1.0)
    if (np.fabs(R[2, 2] - R[0, 0] - R[1, 1] + 1.0) < dEpsilon):
        quat[3] = 0.0
    else:
        quat[3] = 0.5 * np.sign(R[1, 0] - R[0, 1]) * np.sqrt(R[2, 2] - R[0, 0] - R[1, 1] + 1.0)
    return np.array(quat)


def TF2xyzquat(T):
    quat = rot2quat(T[0:3, 0:3])
    res = [T[0, 3], T[1, 3], T[2, 3], quat[0], quat[1], quat[2], quat[3]]
    return np.array(res)
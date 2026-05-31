import numpy as np
from copy import copy

cos=np.cos; sin=np.sin; pi=np.pi


def dh(d, theta, a, alpha):
  """
  Calcular la matriz de transformacion homogenea asociada con los parametros
  de Denavit-Hartenberg.
  Los valores d, theta, a, alpha son escalares.
  """
  # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
  sth = np.sin(theta)
  cth = np.cos(theta)
  sa  = np.sin(alpha)
  ca  = np.cos(alpha)
  T = np.array([[cth, -ca*sth,  sa*sth, a*cth],
                [sth,  ca*cth, -sa*cth, a*sth],
                [0.0,      sa,      ca,     d],
                [0.0,     0.0,     0.0,   1.0]])
  return T
    

def fkine_ur5(q):
  """
  Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
  q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
  """
  """  
      q : vector numpy [q1, q2, q3, q4, q5, q6] en radianes.
    Retorna T : matriz 4x4 del efector final respecto a la base.

    Tabla DH (Universal Robots UR5e, valores en metros y radianes):
        i |  theta  |    d     |    a      |  alpha
        --|---------|----------|-----------|--------
        1 |   q1    |  0.1625  |   0       |  pi/2
        2 |   q2    |  0       | -0.425    |  0
        3 |   q3    |  0       | -0.3922   |  0
        4 |   q4    |  0.1333  |   0       |  pi/2
        5 |   q5    |  0.0997  |   0       | -pi/2
        6 |   q6    |  0.0996  |   0       |  0
    """





  # Longitudes (en metros)
  # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
  T1 = dh( 0.1625,        q[0],     0, pi/2)
  T2 = dh(      0, q[1]+2*pi/2, -0.425,    0)
  T3 = dh(      0,        q[2], -0.3922,    0)
  T4 = dh( 0.1333, q[3]+2*pi/2,     0, pi/2)
  T5 = dh(0.0997,     q[4]+pi,     0, -pi/2)
  T6 = dh( 0.0996,        q[5],     0,    0)
  # Efector final con respecto a la base
  T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
  return T


def jacobian_position(q, delta=0.0001):
 """
 Jacobiano analitico para la posicion de un brazo robotico de n grados de libertad.
 Retorna una matriz de 3xn y toma como entrada el vector de configuracion articular
 q=[q1, q2, q3, ..., qn]
 """
 # Determinar la cantidad de articulaciones
 n = q.size
 # Crear una matriz 3xn
 J = np.zeros((3, n))
 # Calcular la transformacion homogenea inicial (usando q)
 T = fkine_ur5(q)
 # Posicion inicial del efector final (columna 4 de la matriz T)
 x = T[0:3, 3]

 # Iteracion para la derivada de cada articulacion (columna)
 for i in range(n):
  # Copiar la configuracion articular inicial
  dq = copy(q)
  # Incrementar la articulacion i-esima usando un delta,
  # usar la copia de configuración inicial
  dq[i] = dq[i] + delta
  # Transformacion homogenea luego del incremento (q+delta)
  T_dq = fkine_ur5(dq)
  # Posicion del efector final luego del incremento
  x_dq = T_dq[0:3, 3]
  # Aproximacion del Jacobiano de posicion usando diferencias finitas
  J[:, i] = (x_dq - x) / delta

 return J



def ikine(xdes, q0):
 """
 Calcular la cinematica inversa de un brazo robotico numericamente a partir
 de la configuracion articular inicial de q0. Emplear el metodo de newton.
 """
 epsilon  = 0.001
 max_iter = 1000
 delta    = 0.00001

 q  = copy(q0)
 for i in range(max_iter):
  # Main loop
  # Calcular la cinematica directa con la configuracion actual
  T = fkine_ur5(q)
  # Posicion actual del efector final
  x = T[0:3, 3]
  # Error entre la posicion deseada y la actual
  e = xdes - x
  # Criterio de convergencia: si el error es muy pequeño, terminar
  if np.linalg.norm(e) < epsilon:
   print(f"Newton convergio en {i} iteraciones")
   break
  # Calcular el Jacobiano de posicion en la configuracion actual
  J = jacobian_position(q, delta)
  # Actualizacion de Newton: q_{k+1} = q_k + J^+ * e
  q = q + np.dot(np.linalg.pinv(J), e)

 return q


def ik_gradient(xdes, q0):
 """
 Calcular la cinematica inversa de un brazo robotico numericamente a partir
 de la configuracion articular inicial de q0. Emplear el metodo gradiente.
 """
 epsilon  = 0.001
 max_iter = 1000
 delta    = 0.00001
 alpha    = 0.1

 q  = copy(q0)
 for i in range(max_iter):
  # Main loop
  # Calcular la cinematica directa con la configuracion actual
  T = fkine_ur5(q)
  # Posicion actual del efector final
  x = T[0:3, 3]
  # Error entre la posicion deseada y la actual
  e = xdes - x
  # Criterio de convergencia
  if np.linalg.norm(e) < epsilon:
   print(f"Gradiente convergio en {i} iteraciones")
   break
  # Calcular el Jacobiano de posicion en la configuracion actual
  J = jacobian_position(q, delta)
  # Actualizacion del gradiente: q_{k+1} = q_k + alpha * J^T * e
  q = q + alpha * np.dot(J.T, e)

 return q


def rot2quat(R):
  """
  Convertir una matriz de rotacion en un cuaternion

  Entrada:
   R -- Matriz de rotacion
  Salida:
   Q -- Cuaternion [ew, ex, ey, ez]

  """
  dEpsilon = 1e-6
  quat = 4*[0.,]

  quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)
  if ( np.fabs(R[0,0]-R[1,1]-R[2,2]+1.0) < dEpsilon ):
    quat[1] = 0.0
  else:
    quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(R[0,0]-R[1,1]-R[2,2]+1.0)
  if ( np.fabs(R[1,1]-R[2,2]-R[0,0]+1.0) < dEpsilon ):
    quat[2] = 0.0
  else:
    quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(R[1,1]-R[2,2]-R[0,0]+1.0)
  if ( np.fabs(R[2,2]-R[0,0]-R[1,1]+1.0) < dEpsilon ):
    quat[3] = 0.0
  else:
    quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(R[2,2]-R[0,0]-R[1,1]+1.0)

  return np.array(quat)


def TF2xyzquat(T):
  """
  Convert a homogeneous transformation matrix into the a vector containing the
  pose of the robot.
  
  Input:
   T -- A homogeneous transformation
  Output:
   X -- A pose vector in the format [x y z ew ex ey ez], donde la first part
        is Cartesian coordinates and the last part is a quaternion
  """
  quat = rot2quat(T[0:3,0:3])
  res = [T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]]
  return np.array(res)

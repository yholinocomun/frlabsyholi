#!/usr/bin/env python3
"""
Actividad 2.2 - Cuadrado en el plano XY
El efector final del UR5 traza un cuadrado de 8 cm de lado
en un plano horizontal (z constante) usando cinematica inversa.
"""
import rclpy
import numpy as np
from copy import copy
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from lab5functions import ikine, fkine_ur5


def main():

  rclpy.init()
  node = rclpy.create_node('square_ur5')

  # Cliente de la accion de trayectoria en Gazebo
  ac = SimpleActionClient(
    node,
    '/joint_trajectory_controller/follow_joint_trajectory',
    FollowJointTrajectory
  )
  ac.wait_for_server()

  # Nombres de las articulaciones del UR5
  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # ── Geometria del cuadrado ────────────────────────────────────────────────
  L  = 0.08        # lado del cuadrado en metros (8 cm)
  h  = L / 2.0     # mitad del lado
  z  = 0.40        # altura conveniente en metros (plano paralelo al piso)
  cx = 0.50        # centro del cuadrado en X
  cy = 0.00        # centro del cuadrado en Y

  # Esquinas del cuadrado recorridas en sentido antihorario
  # Punto 5 = Punto 1 para cerrar el cuadrado
  cartesian_waypoints = [
    np.array([cx - h, cy - h, z]),   # esquina 1 (frente-izquierda)
    np.array([cx + h, cy - h, z]),   # esquina 2 (frente-derecha)
    np.array([cx + h, cy + h, z]),   # esquina 3 (atras-derecha)
    np.array([cx - h, cy + h, z]),   # esquina 4 (atras-izquierda)
    np.array([cx - h, cy - h, z]),   # cierre al origen del cuadrado
  ]

  # ── Calcular IK para cada esquina ────────────────────────────────────────
  # Semilla inicial (pose "ready" del robot)
  q0 = np.array([0.0, -1.0, 1.7, -1.0, -1.57, 0.0])

  print('Calculando cinematica inversa para cada esquina del cuadrado...')
  print(f'Cuadrado: lado={L*100:.0f}cm, centro=[{cx},{cy},{z}]m')
  print('-' * 60)

  joint_configs = []
  q_prev = copy(q0)

  for i, xdes in enumerate(cartesian_waypoints):
    # Usar la solucion anterior como semilla: garantiza continuidad articular
    q = ikine(xdes, q_prev)
    joint_configs.append(q.tolist())

    T   = fkine_ur5(q)
    err = np.linalg.norm(T[0:3, 3] - xdes)
    etiqueta = 'cierre' if i == 4 else f'esquina {i+1}'
    print(f'  {etiqueta}: xdes={np.round(xdes,3)} | alcanzada={np.round(T[0:3,3],3)} | error={err:.5f}m')

    q_prev = copy(q)   # semilla para la siguiente iteracion

  print('-' * 60)

  # ── Construir la trayectoria con todos los puntos ────────────────────────
  goal = FollowJointTrajectory.Goal()
  traj = JointTrajectory()
  traj.joint_names = joint_names

  # Tiempos de cada waypoint:
  #   - 5 s para llegar a la primera esquina desde la pose actual
  #   - 3 s por cada lado del cuadrado (8 cm a velocidad moderada)
  t_inicio  = 5
  t_por_lado = 3

  for i, q in enumerate(joint_configs):
    point = JointTrajectoryPoint()
    point.positions = q
    t_total = t_inicio + i * t_por_lado
    point.time_from_start = Duration(sec=t_total, nanosec=0)
    traj.points.append(point)

  goal.trajectory = traj

  # ── Enviar y esperar resultado ────────────────────────────────────────────
  print('Enviando trayectoria a Gazebo...')
  print(f'Duracion total estimada: {t_inicio + len(cartesian_waypoints)*t_por_lado} segundos')
  result = ac.send_goal(goal)
  print('Resultado:', result)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

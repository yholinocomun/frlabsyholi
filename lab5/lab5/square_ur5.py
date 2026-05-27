#!/usr/bin/env python3
"""
Actividad 2.2 - Cuadrado en el plano XY
El efector final del UR5 traza un cuadrado de 8 cm de lado
en un plano horizontal (z constante) usando cinematica inversa.
"""
import rclpy
from rclpy.action import ActionClient
import numpy as np
from copy import copy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from lab5functions import ikine, fkine_ur5


def send_goal_and_wait(node, goal):
  """Envia un goal de trayectoria y bloquea hasta que el robot llegue."""
  client = ActionClient(
    node,
    FollowJointTrajectory,
    '/joint_trajectory_controller/follow_joint_trajectory'
  )

  if not client.wait_for_server(timeout_sec=5.0):
    print('ERROR: Servidor de accion no disponible. Verifica que Gazebo este corriendo.')
    return None

  future_goal = client.send_goal_async(goal)
  rclpy.spin_until_future_complete(node, future_goal)
  goal_handle = future_goal.result()

  if not goal_handle.accepted:
    print('ERROR: Goal rechazado por el controlador.')
    return None

  print('Goal aceptado. Ejecutando trayectoria...')

  future_result = goal_handle.get_result_async()
  rclpy.spin_until_future_complete(node, future_result)
  return future_result.result().result


def main():

  rclpy.init()
  node = rclpy.create_node('square_ur5')

  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # ── Geometria del cuadrado ────────────────────────────────────────────────
  L  = 0.08        # lado del cuadrado en metros (8 cm)
  h  = L / 2.0     # mitad del lado (4 cm)
  z  = 0.40        # altura en metros (plano XY paralelo al piso)
  cx = 0.50        # centro del cuadrado en X
  cy = 0.00        # centro del cuadrado en Y

  # Las 4 esquinas del cuadrado + cierre (sentido antihorario)
  cartesian_waypoints = [
    np.array([cx - h, cy - h, z]),   # esquina 1
    np.array([cx + h, cy - h, z]),   # esquina 2
    np.array([cx + h, cy + h, z]),   # esquina 3
    np.array([cx - h, cy + h, z]),   # esquina 4
    np.array([cx - h, cy - h, z]),   # cierre
  ]

  # ── Calcular IK para cada esquina ────────────────────────────────────────
  q0 = np.array([0.0, -1.0, 1.7, -1.0, -1.57, 0.0])

  print(f'Cuadrado: lado={L*100:.0f}cm | centro=[{cx}, {cy}, {z}]m')
  print('-' * 60)

  joint_configs = []
  q_prev = copy(q0)

  for i, xdes in enumerate(cartesian_waypoints):
    # Usar solucion anterior como semilla para garantizar continuidad articular
    q   = ikine(xdes, q_prev)
    T   = fkine_ur5(q)
    err = np.linalg.norm(T[0:3, 3] - xdes)
    etiqueta = 'cierre  ' if i == 4 else f'esquina {i+1}'
    print(f'  {etiqueta}: xdes={np.round(xdes,3)} | error={err:.5f}m')
    joint_configs.append(q.tolist())
    q_prev = copy(q)

  print('-' * 60)

  # ── Construir trayectoria multi-punto (un solo goal = movimiento continuo) ─
  goal = FollowJointTrajectory.Goal()
  traj = JointTrajectory()
  traj.joint_names = joint_names

  # 5 s para llegar a la primera esquina + 3 s por cada lado
  t_inicio   = 5
  t_por_lado = 3

  for i, q in enumerate(joint_configs):
    point = JointTrajectoryPoint()
    point.positions       = q
    point.time_from_start = Duration(sec=t_inicio + i * t_por_lado, nanosec=0)
    traj.points.append(point)

  goal.trajectory = traj

  # ── Enviar y esperar resultado ────────────────────────────────────────────
  duracion = t_inicio + (len(cartesian_waypoints) - 1) * t_por_lado
  print(f'Enviando trayectoria... duracion total: {duracion}s')
  result = send_goal_and_wait(node, goal)
  print('Resultado:', result)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

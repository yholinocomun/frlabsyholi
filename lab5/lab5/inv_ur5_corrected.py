#!/usr/bin/env python3
"""
square_gazebo_corrected.py
VERSION CORREGIDA: Robot en posicion extendida correcta, dibuja cuadrado de 8cm
"""
import rclpy
import numpy as np
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lab5functions import ikine, fkine_ur5


def main():

  rclpy.init()
  node = rclpy.create_node('square_gazebo_corrected')

  client = SimpleActionClient(node,
                              FollowJointTrajectory,
                              '/joint_trajectory_controller/follow_joint_trajectory')

  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # =========================================================================
  # PARAMETROS DEL CUADRADO
  # =========================================================================
  L  = 0.08   # Lado (8 cm)
  x0 = 0.4    # Centro X
  y0 = 0.3    # Centro Y (movido a la derecha)
  z0 = 0.35   # Altura Z (mas baja para mejor alcance)

  print("=" * 70)
  print("SQUARE_GAZEBO_CORRECTED - Dibujar cuadrado de 8cm")
  print("=" * 70)
  print(f"Parametros:")
  print(f"  Lado: {L*100} cm")
  print(f"  Centro: ({x0}, {y0}, {z0})")
  print()

  # 4 esquinas del cuadrado
  P1 = np.array([x0 - L/2, y0 - L/2, z0])
  P2 = np.array([x0 + L/2, y0 - L/2, z0])
  P3 = np.array([x0 + L/2, y0 + L/2, z0])
  P4 = np.array([x0 - L/2, y0 + L/2, z0])

  corners = [P1, P2, P3, P4, P1]

  print(f"Esquinas:")
  for i, P in enumerate([P1, P2, P3, P4]):
    print(f"  P{i+1}: {np.round(P, 3)}")
  print()

  # =========================================================================
  # CONFIGURACION INICIAL CORREGIDA
  # =========================================================================
  # ¡ESTA ES LA POSICION EXTENDIDA CORRECTA!


  q0 = np.array([0.0, -1.57, 1.57, 0.0, 1.57, 0.0])

  T0 = fkine_ur5(q0)
  print(f"Configuracion inicial (POSICION EXTENDIDA):")
  print(f"  q0 = {np.round(q0, 4)}")
  print(f"  Posicion: {np.round(T0[0:3, 3], 3)}")
  print()

  # =========================================================================
  # CALCULAR IK PARA CADA ESQUINA
  # =========================================================================
  print("=" * 70)
  print("CALCULANDO TRAYECTORIA")
  print("=" * 70)
  print()

  q_list = []
  q_seed = q0

  for i, P in enumerate(corners):
    print(f"Esquina {i+1}: {np.round(P, 3)}")
    q = ikine(P, q_seed)
    T = fkine_ur5(q)
    err = np.linalg.norm(P - T[0:3, 3])
    print(f"  Error: {err:.6f} m | q: {np.round(q, 3)}")

    q_list.append(q.tolist())
    q_seed = q

  print()

  # =========================================================================
  # CONSTRUIR Y ENVIAR TRAYECTORIA
  # =========================================================================
  print("=" * 70)
  print("ENVIANDO TRAYECTORIA")
  print("=" * 70)

  goal = FollowJointTrajectory.Goal()
  traj = JointTrajectory()
  traj.joint_names = joint_names

  dt = 3  # segundos entre puntos

  for i, q in enumerate(q_list):
    point = JointTrajectoryPoint()
    point.positions = q
    point.time_from_start.sec = (i + 1) * dt
    point.time_from_start.nanosec = 0
    traj.points.append(point)

  goal.trajectory = traj

  result = client.send_goal(goal)
  print(f"Resultado: {result}")

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
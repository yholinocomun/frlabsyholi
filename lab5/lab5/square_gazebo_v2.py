#!/usr/bin/env python3
"""
square_gazebo_v2.py
Versión mejorada: dibuja un cuadrado de 8cm en el plano XY
Con mejor diagnostico de orientacion y mejor q0 inicial
"""
import rclpy
import numpy as np
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lab5functions import ikine, fkine_ur5


def main():

  # Iniciar nodo
  rclpy.init()
  node = rclpy.create_node('square_gazebo_v2')

  # Cliente de accion
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
  y0 = 0.0    # Centro Y
  z0 = 0.4    # Altura Z (plano XY paralelo al piso)

  print("=" * 70)
  print("SQUARE_GAZEBO_V2 - Dibujar un cuadrado de 8cm en el plano XY")
  print("=" * 70)
  print(f"Parametros:")
  print(f"  Lado del cuadrado: {L*100} cm")
  print(f"  Centro: ({x0}, {y0}, {z0})")
  print()

  # 4 esquinas del cuadrado
  P1 = np.array([x0 - L/2, y0 - L/2, z0])
  P2 = np.array([x0 + L/2, y0 - L/2, z0])
  P3 = np.array([x0 + L/2, y0 + L/2, z0])
  P4 = np.array([x0 - L/2, y0 + L/2, z0])

  corners = [P1, P2, P3, P4, P1]

  print(f"Esquinas del cuadrado:")
  print(f"  P1 (inf-izq): {np.round(P1, 3)}")
  print(f"  P2 (inf-der): {np.round(P2, 3)}")
  print(f"  P3 (sup-der): {np.round(P3, 3)}")
  print(f"  P4 (sup-izq): {np.round(P4, 3)}")
  print()

  # =========================================================================
  # CONFIGURACION INICIAL
  # =========================================================================
  # q0 elegida para mejor convergencia
  q0 = np.array([0.0, -0.5, 0.5, 0.0, 0.0, 0.0])

  T0 = fkine_ur5(q0)
  print(f"Configuracion inicial q0:")
  print(f"  q = {np.round(q0, 3)}")
  print(f"  Posicion efector: {np.round(T0[0:3, 3], 4)}")
  print(f"  Eje Z efector:    {np.round(T0[0:3, 2], 3)}")
  print()

  # =========================================================================
  # CALCULAR CINEMATICA INVERSA PARA CADA ESQUINA
  # =========================================================================
  print("=" * 70)
  print("CALCULANDO TRAYECTORIA (IK para cada esquina)")
  print("=" * 70)

  q_list = []
  q_seed = q0

  for i, P in enumerate(corners):
    print(f"\nEsquina {i+1}:")
    print(f"  Posicion deseada: {np.round(P, 4)}")

    q = ikine(P, q_seed)
    T = fkine_ur5(q)
    x_actual = T[0:3, 3]
    z_axis = T[0:3, 2]
    error = np.linalg.norm(P - x_actual)

    print(f"  Posicion obtenida: {np.round(x_actual, 4)}")
    print(f"  Error de posicion: {error:.6f} m")
    print(f"  Configuracion q:   {np.round(q, 3)}")
    print(f"  Eje Z efector:     {np.round(z_axis, 3)}")

    q_list.append(q.tolist())
    q_seed = q  # Usar solucion anterior como semilla

  # =========================================================================
  # CONSTRUIR Y ENVIAR TRAYECTORIA
  # =========================================================================
  print("\n" + "=" * 70)
  print("ENVIANDO TRAYECTORIA AL ROBOT")
  print("=" * 70)

  goal = FollowJointTrajectory.Goal()
  traj = JointTrajectory()
  traj.joint_names = joint_names

  dt = 3  # Tiempo entre puntos (segundos)

  for i, q in enumerate(q_list):
    point = JointTrajectoryPoint()
    point.positions = q
    point.time_from_start.sec = (i + 1) * dt
    point.time_from_start.nanosec = 0
    traj.points.append(point)
    print(f"Punto {i+1}: t={point.time_from_start.sec}s, q={np.round(q, 2)}")

  goal.trajectory = traj

  print("\nEnviando al controlador...")
  result = client.send_goal(goal)
  print(f"Resultado: {result}")

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
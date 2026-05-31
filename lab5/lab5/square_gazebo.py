#!/usr/bin/env python3
from frlabsyholi.lab5.lab5 import command_gazebo
import rclpy
import numpy as np
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from lab5functions import ikine, fkine_ur5


def main():

  # Iniciar un nodo
  rclpy.init()
  node = rclpy.create_node('square_gazebo')

  # Declara la accion del tipo cliente
  client = SimpleActionClient(node,
                              FollowJointTrajectory,
                              '/joint_trajectory_controller/follow_joint_trajectory')

  # Nombres de las articulaciones del UR5
  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # ---------------------------------------------------------------
  # Definicion del cuadrado en el plano XY
  # ---------------------------------------------------------------
  L  = 0.08   # Lado del cuadrado (8 cm)
  x0 = 0.40    # Coordenada X del centro del cuadrado
  y0 = 0.0    # Coordenada Y del centro del cuadrado
  z0 = 0.4    # Altura del plano XY (paralelo al piso)

  # 4 esquinas del cuadrado (sentido antihorario vista desde arriba)
  P1 = np.array([x0 - L/2, y0 - L/2, z0])
  P2 = np.array([x0 + L/2, y0 - L/2, z0])
  P3 = np.array([x0 + L/2, y0 + L/2, z0])
  P4 = np.array([x0 - L/2, y0 + L/2, z0])

  # Lista de esquinas (cierra el cuadrado regresando a P1)
  corners = [P1, P2, P3, P4, P1]

  # Configuracion articular inicial con orientacion correcta
  # (eje Z del efector apuntando hacia abajo)
  q0 = np.array([0.0, -1.0, 1.0, 0.0, 0.0, 0.0])

  # ---------------------------------------------------------------
  # Calcular la configuracion articular para cada esquina
  # ---------------------------------------------------------------
  print("Verificando orientacion inicial q0:")
  T0 = fkine_ur5(q0)
  print(f"  q0 = {np.round(q0, 3)}")
  print(f"  Posicion: {np.round(T0[0:3, 3], 3)}")
  print(f"  Eje Z del efector: {np.round(T0[0:3, 2], 3)}")
  print()

  print("Calculando cinematica inversa para cada esquina del cuadrado...")
  q_list = []
  q_seed = q0
  for i, P in enumerate(corners):
    q = ikine(P, q_seed)
    T = fkine_ur5(q)
    err = np.linalg.norm(P - T[0:3, 3])
    print(f"Esquina {i+1}: P = {np.round(P,3)} | error = {err:.6f}")
    print(f"           Eje Z: {np.round(T[0:3, 2], 3)}")
    q_list.append(q.tolist())
    # Usar la solucion actual como semilla para la siguiente (mejor convergencia)
    q_seed = q

  # ---------------------------------------------------------------
  # Construir la trayectoria
  # ---------------------------------------------------------------
  goal = FollowJointTrajectory.Goal()
  traj = JointTrajectory()
  traj.joint_names = joint_names

  # Tiempo entre puntos (segundos)
  dt = 3

  for i, q in enumerate(q_list):
    point = JointTrajectoryPoint()
    point.positions = q
    point.time_from_start.sec = (i + 1) * dt
    point.time_from_start.nanosec = 0
    traj.points.append(point)

  goal.trajectory = traj

  # Enviar el objetivo y esperar por el resultado
  print("\nEnviando trayectoria al robot...")
  result = client.send_goal(goal)
  print("Resultado:", result)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
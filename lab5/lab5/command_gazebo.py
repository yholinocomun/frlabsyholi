#!/usr/bin/env python3
import rclpy
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():

  # Iniciar un nodo
  rclpy.init()
  node = rclpy.create_node('command_gazebo')

  # Declara la acción del tipo cliente
  client = SimpleActionClient(node,
                              FollowJointTrajectory,
                              '/joint_trajectory_controller/follow_joint_trajectory')

  # Declara las variables del brazo robotico
  # Lista de nombres de las articulaciones del UR5
  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
  # Lista de valores de la configuracion del robot (posicion deseada)
  Q0 = [0.0, -1.57, 1.57, 0.0, 1.57, 0.0]

  # Definir el tipo de mensaje a utilizar
  goal = FollowJointTrajectory.Goal()
  traj = JointTrajectory()
  point = JointTrajectoryPoint()

  # Definir los nombres de las articulaciones de traj.joint_names
  traj.joint_names = joint_names

  # Definir la posicion inicial de point
  point.positions = Q0

  # Definir el tiempo para llegar al point (5 segundos)
  point.time_from_start.sec = 5
  point.time_from_start.nanosec = 0

  # Agregar el punto a la trayectoria
  traj.points.append(point)
  goal.trajectory = traj

  # Enviar el objetivo y esperar por el resultado
  result = client.send_goal(goal)

  # Imprimir el resultado
  print("Resultado:", result)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
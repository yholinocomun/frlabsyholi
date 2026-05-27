#!/usr/bin/env python3
import rclpy
import numpy as np
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def main():

  # Iniciar un nodo
  rclpy.init()
  node = rclpy.create_node('command_gazebo')

  # Declara la accion del tipo cliente
  ac = SimpleActionClient(
    node,
    '/joint_trajectory_controller/follow_joint_trajectory',
    FollowJointTrajectory
  )
  ac.wait_for_server()

  # Nombres de las articulaciones del UR5
  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # Configuracion articular deseada (pose conveniente "ready")
  Q0 = [0.0, -1.0, 1.7, -1.0, -1.57, 0.0]

  # Definir el tipo de mensaje a utilizar
  goal  = FollowJointTrajectory.Goal()
  traj  = JointTrajectory()
  point = JointTrajectoryPoint()

  # Asignar los nombres de las articulaciones a la trayectoria
  traj.joint_names = joint_names

  # Posicion articular del punto de trayectoria
  point.positions = Q0

  # Tiempo para llegar al punto (5 segundos desde el inicio)
  point.time_from_start = Duration(sec=5, nanosec=0)

  # Agregar el punto a la trayectoria
  traj.points.append(point)
  goal.trajectory = traj

  # Enviar el objetivo y esperar por el resultado
  result = ac.send_goal(goal)

  # Imprimir el resultado
  print('Resultado:', result)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

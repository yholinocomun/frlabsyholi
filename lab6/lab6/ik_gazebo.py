#!/usr/bin/env python3
# Actividad 2.2: Cinematica inversa de POSICION y ORIENTACION usando kinpy,
# enviando el resultado a SIMULACION en Gazebo.
#
# Combina "test_kinpy" (resolver la IK a partir del URDF) con la idea de
# "command_gazebo" del laboratorio 5 (enviar la configuracion articular al
# controlador de trayectoria mediante una accion FollowJointTrajectory).

import os
import rclpy
import numpy as np
import kinpy as kp
from scipy.spatial.transform import Rotation as R
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def _urdf_path():
  here = os.path.dirname(os.path.abspath(__file__))
  for c in [os.path.join(here, '..', 'urdf', 'ur5_robot.urdf'),
            os.path.join(here, '..', '..', 'urdf', 'ur5_robot.urdf')]:
    if os.path.exists(c):
      return c
  return "../urdf/ur5_robot.urdf"


def main():
  rclpy.init()
  node = rclpy.create_node('ik_gazebo')

  # Cliente de la accion del controlador de trayectoria del UR5
  client = SimpleActionClient(node,
                              FollowJointTrajectory,
                              '/joint_trajectory_controller/follow_joint_trajectory')

  # Cadena cinematica a partir del URDF
  chain = kp.build_serial_chain_from_urdf(
      open(_urdf_path()).read(),
      root_link_name="base_link",
      end_link_name="ee_link")

  # ----- Pose deseada (posicion + orientacion) -----
  target_pos = np.array([0.5, 0.0, 0.5])
  desired_rpy = (0.0, np.pi / 2, 0.0)

  r = R.from_euler('xyz', desired_rpy)
  quat_xyzw = r.as_quat()
  quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
  target_transform = kp.Transform(rot=quat_wxyz, pos=target_pos)

  # Resolver la cinematica inversa
  q0 = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
  q = list(chain.inverse_kinematics(target_transform, q0))
  print("Configuracion articular obtenida (rad):", np.round(q, 4))

  # Nombres de las articulaciones del UR5
  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # Construir el objetivo de la trayectoria
  goal = FollowJointTrajectory.Goal()
  traj = JointTrajectory()
  point = JointTrajectoryPoint()

  traj.joint_names = joint_names
  point.positions = q
  point.time_from_start.sec = 5
  point.time_from_start.nanosec = 0
  traj.points.append(point)
  goal.trajectory = traj

  # Enviar el objetivo y esperar el resultado
  result = client.send_goal(goal)
  print("Resultado:", result)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

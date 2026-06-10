#!/usr/bin/env python3
# Actividad 3.1: Mover el robot en SIMULACION cambiando UNICAMENTE la
# orientacion del efector final, manteniendo su POSICION fija.
#
# - La posicion del efector se mantiene constante (no se mueve en el espacio).
# - La orientacion barre un angulo de modo que el eje del efector se mueva en
#   un plano PARALELO al plano ZY  => rotacion alrededor del eje X.
# - Restriccion de seguridad: el efector nunca debe bajar de 0.10 m sobre la
#   base del robot (se valida cada pose antes de enviarla).

import os
import rclpy
import numpy as np
import kinpy as kp
from scipy.spatial.transform import Rotation as R
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Altura minima de seguridad del efector respecto a la base (m)
Z_MIN_SEGURIDAD = 0.10


def _urdf_path():
  here = os.path.dirname(os.path.abspath(__file__))
  for c in [os.path.join(here, '..', 'urdf', 'ur5_robot.urdf'),
            os.path.join(here, '..', '..', 'urdf', 'ur5_robot.urdf')]:
    if os.path.exists(c):
      return c
  return "../urdf/ur5_robot.urdf"


def main():
  rclpy.init()
  node = rclpy.create_node('move_orientation')

  client = SimpleActionClient(node,
                              FollowJointTrajectory,
                              '/joint_trajectory_controller/follow_joint_trajectory')

  chain = kp.build_serial_chain_from_urdf(
      open(_urdf_path()).read(),
      root_link_name="base_link",
      end_link_name="ee_link")

  # ----- POSICION FIJA del efector final (no cambia durante la tarea) -----
  # Debe estar por encima de la altura de seguridad (z > 0.10 m).
  pos_fija = np.array([0.4, 0.0, 0.4])
  assert pos_fija[2] > Z_MIN_SEGURIDAD, "La posicion fija viola la altura de seguridad"

  # ----- Barrido de ORIENTACION en el plano paralelo a ZY -----
  # Rotacion alrededor del eje X (el efector se inclina dentro del plano ZY).
  angulos = np.linspace(-np.pi / 4, np.pi / 4, 9)   # de -45 a +45 grados

  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  goal = FollowJointTrajectory.Goal()
  traj = JointTrajectory()
  traj.joint_names = joint_names

  q_seed = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]
  t = 2.0          # instante del primer punto (s)
  dt_punto = 1.5   # separacion temporal entre puntos (s)

  for ang in angulos:
    # Orientacion deseada: rotacion 'ang' alrededor de X (plano ZY)
    r = R.from_euler('x', ang)
    quat_xyzw = r.as_quat()
    quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])

    # Validacion de seguridad: la posicion (fija) debe estar sobre el limite
    if pos_fija[2] < Z_MIN_SEGURIDAD:
      print("Pose descartada por seguridad (z < 0.10 m)")
      continue

    target = kp.Transform(rot=quat_wxyz, pos=pos_fija)
    q = list(chain.inverse_kinematics(target, q_seed))
    q_seed = q   # usar la solucion anterior como semilla (continuidad)

    point = JointTrajectoryPoint()
    point.positions = q
    point.time_from_start.sec = int(t)
    point.time_from_start.nanosec = int((t - int(t)) * 1e9)
    traj.points.append(point)
    t += dt_punto

  goal.trajectory = traj
  result = client.send_goal(goal)
  print("Resultado:", result)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

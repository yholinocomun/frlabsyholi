#!/usr/bin/env python3
# Actividad 2.1: Cinematica inversa de POSICION y ORIENTACION usando la
# libreria kinpy, mostrando el resultado en VISUALIZACION (RViz).
#
# Combina la idea de "test_kinpy" (resolver la IK a partir del URDF) con la de
# "test_fkine" del laboratorio 4 (publicar joint_states y un FrameMarker para
# poder ver el robot y la pose deseada en RViz).

import os
import rclpy
import threading
import numpy as np
import kinpy as kp
from scipy.spatial.transform import Rotation as R
from markers import *
from lab6functions import *
from sensor_msgs.msg import JointState


def _urdf_path():
  """Ubica el archivo URDF de forma robusta (corra desde donde corra)."""
  here = os.path.dirname(os.path.abspath(__file__))
  candidatos = [
    os.path.join(here, '..', 'urdf', 'ur5_robot.urdf'),       # desde el codigo fuente
    os.path.join(here, '..', '..', 'urdf', 'ur5_robot.urdf'),
  ]
  for c in candidatos:
    if os.path.exists(c):
      return c
  # Fallback: ruta relativa tal como en el ejemplo test_kinpy
  return "../urdf/ur5_robot.urdf"


def main():
  rclpy.init()
  node = rclpy.create_node('ik_kinpy')
  pub = node.create_publisher(JointState, 'joint_states', 10)

  thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
  thread.start()

  # Cadena cinematica del robot a partir del URDF
  chain = kp.build_serial_chain_from_urdf(
      open(_urdf_path()).read(),
      root_link_name="base_link",
      end_link_name="ee_link")

  # ----- Pose deseada (posicion + orientacion) -----
  # Posicion cartesiana deseada del efector final [x, y, z] en metros
  target_pos = np.array([0.5, 0.0, 0.5])
  # Orientacion deseada en angulos de Euler (roll, pitch, yaw) en radianes
  desired_rpy = (0.0, np.pi / 2, 0.0)

  # Convertir la orientacion a cuaternion en el formato que pide kinpy [w,x,y,z]
  r = R.from_euler('xyz', desired_rpy)
  quat_xyzw = r.as_quat()                       # [x, y, z, w]
  quat_wxyz = np.array([quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]])
  target_transform = kp.Transform(rot=quat_wxyz, pos=target_pos)

  # Estimacion inicial (semilla) para la cinematica inversa
  q0 = [0.0, -1.0, 1.0, 0.0, 0.0, 0.0]

  # Resolver la cinematica inversa (posicion + orientacion)
  q = np.array(chain.inverse_kinematics(target_transform, q0))
  print("Configuracion articular obtenida (rad):", np.round(q, 4))

  # Marcador para visualizar la pose deseada en RViz
  marker = FrameMarker(node)
  # Pose deseada como [x y z ew ex ey ez]
  xd = np.array([target_pos[0], target_pos[1], target_pos[2],
                 quat_wxyz[0], quat_wxyz[1], quat_wxyz[2], quat_wxyz[3]])
  marker.setPose(xd)

  # Nombres de las articulaciones del UR5
  jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # Mensaje JointState con la solucion de la IK
  jstate = JointState()
  jstate.header.stamp = node.get_clock().now().to_msg()
  jstate.name = jnames
  jstate.position = q.tolist()

  rate = node.create_rate(20)
  while rclpy.ok():
    jstate.header.stamp = node.get_clock().now().to_msg()
    pub.publish(jstate)
    marker.publish()
    rate.sleep()

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

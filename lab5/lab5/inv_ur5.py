#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
from markers import *
from lab5functions import *
from sensor_msgs.msg import JointState


def main():

  rclpy.init()
  node = rclpy.create_node('inv_ur5')
  pub = node.create_publisher(JointState, 'joint_states', 10)

  thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
  thread.start()

  # Marcador para visualizar la posicion deseada (verde) en RViz
  bmarker = BallMarker(node, color['GREEN'])

  # Nombres de las articulaciones del UR5
  jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # Posicion cartesiana deseada del efector final [x, y, z] (en metros)
  # IMPORTANTE: debe estar dentro del espacio de trabajo del UR5
  xdes = np.array([0.0, 0.5, 0.0])

  # Configuracion articular inicial (estimacion inicial para la IK)
  q0 = np.array([0.0, -1.0, 1.0, 0.0, 1.0, 0.0])

  # Calcular la cinematica inversa usando el metodo de Newton
  q = ikine(xdes, q0)

  # Verificacion: aplicar cinematica directa a la solucion obtenida
  T = fkine_ur5(q)
  print("Configuracion articular obtenida:")
  print(np.round(q, 4))
  print("Posicion obtenida:", np.round(T[0:3, 3], 4))
  print("Posicion deseada: ", xdes)
  print("Error (norma):    ", np.round(np.linalg.norm(xdes - T[0:3, 3]), 6))

  # Posicionar el marcador en la posicion deseada
  bmarker.xyz(xdes)

  # Mensaje de tipo JointState
  jstate = JointState()
  jstate.header.stamp = node.get_clock().now().to_msg()
  jstate.name = jnames
  jstate.position = q.tolist()

  # Tasa de publicacion (Hz)
  rate = node.create_rate(20)
  while rclpy.ok():
    jstate.header.stamp = node.get_clock().now().to_msg()
    pub.publish(jstate)
    bmarker.publish()
    rate.sleep()

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
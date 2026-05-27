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

  bmarker = BallMarker(node, color['RED'])
  marker  = FrameMarker(node)

  # Nombres de las articulaciones del UR5
  jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # Configuracion articular inicial para el algoritmo de IK
  q0 = np.array([0.0, -1.0, 1.7, -1.0, -1.57, 0.0])

  # --- Posicion cartesiana deseada del efector final [x, y, z] en metros ---
  xdes = np.array([0.5, 0.2, 0.3])

  # Calcular cinematica inversa mediante el metodo de Newton
  q = ikine(xdes, q0)

  # Verificacion: cinematica directa sobre la solucion obtenida
  T = fkine_ur5(q)
  print('Configuracion articular resultante (rad):')
  print(np.round(q, 4))
  print('Posicion deseada:   ', np.round(xdes, 4))
  print('Posicion alcanzada: ', np.round(T[0:3, 3], 4))
  print('Error de posicion:  ', np.round(np.linalg.norm(T[0:3, 3] - xdes), 6))

  # Visualizar el efector final en RViz
  bmarker.position(T)
  x0 = TF2xyzquat(T)
  marker.setPose(x0)

  # Preparar el mensaje JointState con la solucion IK
  jstate = JointState()
  jstate.header.stamp = node.get_clock().now().to_msg()
  jstate.name = jnames
  jstate.position = q.tolist()

  # Publicar a 20 Hz de forma continua
  rate = node.create_rate(20)
  while rclpy.ok():
    jstate.header.stamp = node.get_clock().now().to_msg()
    pub.publish(jstate)
    bmarker.publish()
    marker.publish()
    rate.sleep()

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

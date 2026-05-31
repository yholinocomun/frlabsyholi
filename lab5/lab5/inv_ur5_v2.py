#!/usr/bin/env python3
"""
inv_ur5_v2.py
Versión mejorada: calcula cinematica inversa y visualiza el robot en RViz
Con mejor diagnostico y validacion
"""
import rclpy
import threading
import numpy as np
from markers import *
from lab5functions import ikine, fkine_ur5
from sensor_msgs.msg import JointState


def main():

  rclpy.init()
  node = rclpy.create_node('inv_ur5_v2')
  pub = node.create_publisher(JointState, 'joint_states', 10)

  thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
  thread.start()

  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  print("=" * 70)
  print("INV_UR5_V2 - Cinematica Inversa del UR5")
  print("=" * 70)
  print()

  # =========================================================================
  # DEFINIR POSICION DESEADA
  # =========================================================================
  # Posicion cartesiana deseada del efector final (dentro del espacio de trabajo)
  xdes = np.array([0.5, 0.3, 0.4])

  print(f"Posicion deseada del efector final:")
  print(f"  x = {xdes[0]} m")
  print(f"  y = {xdes[1]} m")
  print(f"  z = {xdes[2]} m")
  print()

  # =========================================================================
  # CONFIGURACION INICIAL
  # =========================================================================
  q0 = np.array([0.0, -1.0, 1.0, 0.0, 1.0, 0.0])

  print(f"Configuracion inicial (semilla para la IK):")
  print(f"  q0 = {np.round(q0, 3)}")
  print()

  # =========================================================================
  # CALCULAR CINEMATICA INVERSA
  # =========================================================================
  print("=" * 70)
  print("CALCULANDO CINEMATICA INVERSA (Metodo de Newton)")
  print("=" * 70)
  print()

  q_solucion = ikine(xdes, q0)
  T_solucion = fkine_ur5(q_solucion)
  x_obtenida = T_solucion[0:3, 3]
  error = np.linalg.norm(xdes - x_obtenida)

  print(f"Solucion encontrada:")
  print(f"  q = {np.round(q_solucion, 4)}")
  print()

  print(f"Verificacion (FK de la solucion):")
  print(f"  Posicion deseada:  {np.round(xdes, 4)}")
  print(f"  Posicion obtenida: {np.round(x_obtenida, 4)}")
  print(f"  Error de posicion: {error:.6f} m")
  print()

  if error < 0.01:
    print("✓ CONVERGENCIA EXITOSA (error < 1 cm)")
  else:
    print("✗ ERROR GRANDE (posicion fuera del alcance o no convergio)")

  print()

  # =========================================================================
  # VISUALIZAR EN RVIZ
  # =========================================================================
  print("=" * 70)
  print("PUBLICANDO POSE DEL ROBOT EN RViz")
  print("=" * 70)
  print()

  # Mensaje JointState para publicar la pose del robot
  jstate = JointState()
  jstate.header.stamp = node.get_clock().now().to_msg()
  jstate.name = joint_names
  jstate.position = q_solucion.tolist()

  rate = node.create_rate(20)  # 20 Hz
  while rclpy.ok():
    jstate.header.stamp = node.get_clock().now().to_msg()
    pub.publish(jstate)
    rate.sleep()

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()
#!/usr/bin/env python3

import time
import rclpy
from std_msgs.msg import Float64MultiArray


def main():

  rclpy.init()

  # Declarar el nombre del nodo
  node = rclpy.create_node('JointsNode')
 
  # Definir el publicador para el tópico /forward_position_controller/commands
  publisher = node.create_publisher(
    Float64MultiArray,
    '/forward_position_controller/commands',
    10
  )
 
  # Definir una variable de tipo Float64MultiArray
  msg = Float64MultiArray()

  # Espera breve para que ROS registre el publicador
  time.sleep(1.0)

  # Primera configuración:
  # pan_joint  = -0.393
  # tilt_joint =  0.785
  msg.data = [-0.393, 0.785]

  start_time = time.time()

  # Publicar primera configuración durante 3 segundos
  while rclpy.ok() and (time.time() - start_time) < 3.0:
    publisher.publish(msg)
    node.get_logger().info(f'Primera configuración enviada: {msg.data}')
    time.sleep(0.1)

  # Segunda configuración:
  # pan_joint  =  0.0
  # tilt_joint = -0.393
  msg.data = [0.0, -0.393]

  start_time = time.time()

  # Publicar segunda configuración durante 5 segundos
  while rclpy.ok() and (time.time() - start_time) < 5.0:
    publisher.publish(msg)
    node.get_logger().info(f'Segunda configuración enviada: {msg.data}')
    time.sleep(0.1)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
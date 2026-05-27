#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


def send_goal_and_wait(node, goal):
  """Envia un goal de trayectoria y bloquea hasta que el robot llegue."""
  client = ActionClient(
    node,
    FollowJointTrajectory,
    '/joint_trajectory_controller/follow_joint_trajectory'
  )

  # Esperar a que el servidor de accion este disponible
  if not client.wait_for_server(timeout_sec=5.0):
    print('ERROR: Servidor de accion no disponible. Verifica que Gazebo este corriendo.')
    return None

  # Enviar el goal y esperar que sea aceptado
  future_goal = client.send_goal_async(goal)
  rclpy.spin_until_future_complete(node, future_goal)
  goal_handle = future_goal.result()

  if not goal_handle.accepted:
    print('ERROR: Goal rechazado por el controlador.')
    return None

  print('Goal aceptado. Esperando que el robot llegue a la posicion...')

  # Esperar el resultado final (robot llego a destino)
  future_result = goal_handle.get_result_async()
  rclpy.spin_until_future_complete(node, future_result)
  return future_result.result().result


def main():

  rclpy.init()
  node = rclpy.create_node('command_gazebo')

  # Nombres de las articulaciones del UR5
  joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

  # Configuracion articular deseada (pose "ready")
  Q0 = [0.0, -1.0, 1.7, -1.0, -1.57, 0.0]

  # Construir el mensaje de trayectoria
  goal  = FollowJointTrajectory.Goal()
  traj  = JointTrajectory()
  point = JointTrajectoryPoint()

  traj.joint_names      = joint_names
  point.positions       = Q0
  point.time_from_start = Duration(sec=5, nanosec=0)  # llegar en 5 segundos

  traj.points.append(point)
  goal.trajectory = traj

  # Enviar y esperar resultado
  result = send_goal_and_wait(node, goal)
  print('Resultado:', result)

  node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()

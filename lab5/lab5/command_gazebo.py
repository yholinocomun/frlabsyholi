#!/usr/bin/env python3
import rclpy
from simple_actions import SimpleActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def main():
    
  # Iniciar un nodo
  
  
  # Declara la acción del tipo cliente
  
    
  # Declara las variables del brazo robotico
  # Lista de nombre
  # joint_names = 
  # Lista de valores de la configuracion del robot
  # Q0 = 
 
  # Definir el tipo de mensaje a utilizar
  goal = FollowJointTrajectory.Goal()    
  traj = JointTrajectory()
  point = JointTrajectoryPoint()
  
  # Definir los nombres de las articulaciones de traj.joint_names
  
  # Definir la posicion inicial de point
  
  # Definir el tiempo para llegar al point
  
  
  # Agregar el punto a la trayectoria
  traj.points.append(point)
  goal.trajectory = traj
  
  # Enviar el objetivo y esperar por el resultado
  
  
  # Imprimir el resultado
  
  node.destroy_node()
  rclpy.shutdown()
  
  
if __name__ == '__main__':
  main()
  

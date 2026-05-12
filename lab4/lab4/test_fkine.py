#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
from markers import *
from lab4functions import *
from sensor_msgs.msg import JointState

def main():

  rclpy.init()
  node = rclpy.create_node('ForwardKinematics')
  pub = node.create_publisher(JointState, 'joint_states', 10)
  
  thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
  thread.start()
  
  #bmarker = BallMarker(node, color['GREEN'])
  marker = FrameMarker(node)
 
  # Joint names
  jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint','wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
  # Joint Configuration
  q  = np.array([0.0, -np.pi/4, 0.0, 0.0, 0.0, 0.0])
 
  # End effector with respect to the base
  T = fkine_ur5(q)
  print(np.round(T,3))
  #bmarker.position(T)
 
  x0 = TF2xyzquat(T)
  marker.setPose(x0)
 
  # Object (message) whose type is JointState
  jstate = JointState()
  # Set values to the message
  jstate.header.stamp = node.get_clock().now().to_msg()
  jstate.name = jnames
  # Add the head joint value (with value 0) to the joints
  jstate.position = q.tolist()
  
  # Loop rate (in Hz)
  rate = node.create_rate(20)
  # Continuous execution loop
  while rclpy.ok():
    # Current time (needed for ROS)
    jstate.header.stamp = node.get_clock().now().to_msg()
    # Publish the message
    pub.publish(jstate)
    #bmarker.publish()
    marker.publish()
    # Wait for the next iteration
    rate.sleep()
    
  
  node.destroy_node()
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()
  

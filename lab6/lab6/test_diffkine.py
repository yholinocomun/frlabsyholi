#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
from markers import *
from lab6functions import *
from sensor_msgs.msg import JointState

def main():
  rclpy.init()
  node = rclpy.create_node('testKinematicControlPosition')
  pub = node.create_publisher(JointState, 'joint_states', 10)
  
  thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
  thread.start()
  
  # Files for the logs
  fxcurrent = open("/home/<user>/xcurrent.txt", "w")                
  fxdesired = open("/home/<user>/xdesired.txt", "w")
  fq = open("/home/<user>/q.txt", "w")

  # Markers for the current and desired positions
  bmarker_current  = FrameMarker(node)
  bmarker_desired = FrameMarker(node)

  # Joint names
  jnames = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
 
  # Desired position
  xd = np.array([-0.45, -0.20, 0.6])
  # Initial configuration
  q0 = np.array([0.0, -1.0, 1.7, -2.2, -1.6, 0.0])

  # Resulting initial position (end effector with respect to the base link)
  T = fkine_ur5(q0)
  x0 = T[0:3,3]

  # Red marker shows the achieved position
  bmarker_current.xyz(x0)
  # Green marker shows the desired position
  bmarker_desired.xyz(xd)

  # Object (message) whose type is JointState
  jstate = JointState()
  # Set values to the message
  jstate.header.stamp = node.get_clock().now().to_msg()
  jstate.name = jnames
  # Add the head joint value (with value 0) to the joints
  jstate.position = q0.tolist()

  # Frequency (in Hz) and control period 
  freq = 50
  dt = 1.0/freq
  rate = node.create_rate(20)

  # Initial joint configuration
  q = copy(q0)
  # Main loop
  while rclpy.ok():
    
    # Current time (needed for ROS)
    jstate.header.stamp = node.get_clock().now().to_msg()
    # Kinematic control law for position (complete here)
    # -----------------------------

    # q = 
        
    # -----------------------------

        
    # Log values                                                      
    fxcurrent.write(str(x[0])+' '+str(x[1]) +' '+str(x[2])+'\n')
    fxdesired.write(str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
    fq.write(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" "+
             str(q[4])+" "+str(q[5])+"\n")
        
    # Publish the message
    jstate.position = q.tolist()
    pub.publish(jstate)
    bmarker_desired.xyz(xd)
    bmarker_current.xyz(x)
    # Wait for the next iteration
    rate.sleep()

  print('ending motion ...')
  fxcurrent.close()
  fxdesired.close()
  fq.close()

if __name__ == '__main__':
  main()

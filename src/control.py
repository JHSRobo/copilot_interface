#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
from dynamic_reconfigure.server import Server
from copilot_interface.cfg import copilotControlParamsConfig
from copilot_interface.msg import controlData

control = controlData()

def cameraCallback(joy):
  if joy.buttons[2]:
    control.camera = 1
  elif joy.buttons[3]:
    control.camera = 2
  elif joy.buttons[4]:
    control.camera = 3
  elif joy.buttons[5]:
    control.camera = 4
  control_pub.publish(control)
  

def controlCallback(config, level):
  #Toggle Thrusters and Depth Hold
  control.thruster_status = config.thrusterToggle
  control.dh_status = config.depthHoldToggle

  #Update control message with sensitivity
  control.linear_sense = config.linearSense
  control.angular_sense = config.angularSense
  control.vertical_sense = config.verticalSense
  control_pub.publish(control)

  return config

if __name__  == "__main__":
  rospy.init_node('GUI')
  joy_sub = rospy.Subscriber('joy/joy1', Joy, cameraCallback) # Subscriber to joystick 1
  control_pub = rospy.Publisher('control', controlData, queue_size=1) # Publisher to control
  server = Server(copilotControlParamsConfig, controlCallback)
  rospy.spin()

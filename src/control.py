#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
from dynamic_reconfigure.server import Server
from copilot_interface.cfg import copilotControlParamsConfig
from copilot_interface.msg import controlData

control = controlData()
current_cam = 0

def cameraCallback(joy):
  cam_select = 0
  if joy.buttons[2]:
    cam_select = 1
  elif joy.buttons[3]:
    cam_select = 2
  elif joy.buttons[4]:
    cam_select = 3
  elif joy.buttons[5]:
    cam_select = 4
  if cam_select == current_cam:
    pass
  else:
    control.camera = cam_select
    control_pub.publish(control)
  

def controlCallback(config, level):
  #Toggle Thrusters
  control.thruster_status = config.thrusterToggle

  #Update control message with sensitivity
  control.linear_sense = config.linearSense
  control.angular_sense = config.angularSense
  control.vertical_sense = config.verticalSense

  # Update GPIO status
  control.gpio_pin_3 = config.pin3
  control.gpio_pin_5 = config.pin5
  control.gpio_pin_7 = config.pin7
  control.gpio_pin_11 = config.pin11
  control.gpio_pin_13 = config.pin13

  control_pub.publish(control)
  return config

if __name__  == "__main__":
  rospy.init_node('GUI')
  joy_sub = rospy.Subscriber('joystick', Joy, cameraCallback) # Subscriber to joystick 1
  control_pub = rospy.Publisher('control', controlData, queue_size=1) # Publisher to control
  server = Server(copilotControlParamsConfig, controlCallback)
  rospy.spin()

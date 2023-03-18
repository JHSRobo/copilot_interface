#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8
from sensor_msgs.msg import Joy
from dynamic_reconfigure.server import Server
from copilot_interface.cfg import copilotControlParamsConfig
from launch_files.msg import controlData

control = controlData()
current_cam = 0

def joyCallback(joy):
  cam_select = 0
  change = False
  if joy.axes[4] != 0 or joy.axes[5] != 0:
    control.screenshot = True
    change = True

  if joy.buttons[2]:
    cam_select = 1
    change = True
  elif joy.buttons[3]:
    cam_select = 2
    change = True
  elif joy.buttons[4]:
    cam_select = 3
    change = True
  elif joy.buttons[5]:
    cam_select = 4
    change = True
  if change:
    control.camera = cam_select
    control_pub.publish(control)
    control.screenshot = False

def controlCallback(config, level):
  #Toggle Thrusters
  control.thruster_status = config.thrusterToggle

  #Update control message with sensitivity
  control.linear_sense = config.linearSense
  control.angular_sense = config.angularSense
  control.vertical_sense = config.verticalSense

  # Update GPIO status
  control.gpio_pin_23 = config.pin23
  control.gpio_pin_24 = config.pin24
  control.gpio_pin_25 = config.pin25
  control.gpio_pin_5 = config.pin5
  control.gpio_pin_19 = config.pin19
  control.gpio_pin_16 = config.pin16

  # Update coral status
  control.coral = config.coralPhotos

  control_pub.publish(control)
  return config

if __name__  == "__main__":
  rospy.init_node('GUI')
  joy_sub = rospy.Subscriber('joystick', Joy, joyCallback) # Subscriber to joystick 1
  control_pub = rospy.Publisher('control', controlData, queue_size=1) # Publisher to control
  server = Server(copilotControlParamsConfig, controlCallback)
  rospy.spin()

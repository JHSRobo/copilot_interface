#!/usr/bin/env python
import rospy
#from std_msgs.msg import Int32

from dynamic_reconfigure.server import Server
from copilot_interface.cfg import copilotControlParamsConfig

rospy.init_node('cameraSwitch')

def cameraCallback(joy):
  global camera_select
  if joy.buttons[2]:
      camera_select.publish(1)
    elif joy.buttons[3]:
      camera_select.publish(2)
    elif joy.buttons[4]:
      camera_select.publish(3)
    elif joy.buttons[5]:
      camera_select.publish(4)

def main():
  global joy_sub1, camera_select
  joy_sub1 = rospy.Subscriber('joy/joy1', Joy, cameraCallback) # Subscriber to joystick 1
  camera_select = rospy.Publisher('rov/camera_select', UInt8, queue_size=3) # Publisher to camera select

  rospy.spin()
        
if __name__  == "__main__":
  main()

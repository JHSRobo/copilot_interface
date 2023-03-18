#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8
from dynamic_reconfigure.server import Server
from copilot_interface.cfg import depthControlParamsConfig
from copilot_interface.msg import autoControlData

autoControl = autoControlData()

def autoControlCallback(config, level):
  #Toggle Depth Hold and select Target Depth
  autoControl.dh_status = config.depthHoldToggle
  autoControl.target_depth = config.targetDepth

  #Update pid scalars
  autoControl.p_scalar = config.pScalar
  autoControl.i_scalar = config.iScalar
  autoControl.d_scalar = config.dScalar
  
  #Auto Docking Enable/Disable
  autoControl.auto_dock = config.autoDock
  
  #Publish
  auto_control_pub.publish(autoControl)

  return config

if __name__  == "__main__":
  rospy.init_node('DH_GUI')
  auto_control_pub = rospy.Publisher('auto_control', autoControlData, queue_size=1) # Publisher to autoControl
  server = Server(depthControlParamsConfig, autoControlCallback)
  rospy.spin()
  

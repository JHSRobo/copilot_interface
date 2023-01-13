#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8
from dynamic_reconfigure.server import Server
from copilot_interface.cfg import depthControlParamsConfig
from copilot_interface.msg import autoControlData

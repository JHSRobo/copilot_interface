#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server
from copilot_interface.cfg import copilotControlParamsConfig
#from copilot_interface.cfg import opsInterfaceConfig

lengthProgram = False

def main():
    global pubLength
    rospy.init_node('ops_interface')
  
    pubLength = rospy.Publisher('ops/measure_toggle', Bool, queue_size=1)
    
    def opsCallback(config, level):
      global lengthProgram

      lengthProgram = config.l_scale
      pubLength.publish(lengthProgram)
      
      

    # setup dynamic reconfigure
    server = Server(copilotControlParamsConfig, opsCallback)
    
    rospy.spin()
        
if __name__  == "__main__":
    main()

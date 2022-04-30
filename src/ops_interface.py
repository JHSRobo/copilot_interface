#!/usr/bin/env python
import rospy
from std_msgs.msg import String

from dynamic_reconfigure.server import Server
from copilot_interface.cfg import opsInterfaceConfig

lengthProgram = False

#def opsCallback
    

def main():
    rospy.init_node('opsInterface')
  
    pubLength = rospy.Publisher('ops/measure_toggle', Bool, queue_size=1)
    
    def opsCallback(config, level):
      global lengthProgram

      lengthProgram = config.lengthFinder
      pubLength.publish(lengthProgram)
      
      

    # setup dynamic reconfigure
    server = Server(opsInterfaceConfig, opsCallback)
    
    rospy.spin()
        
if __name__  == "__main__":
    main()

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server
from copilot_interface.cfg import opsControlParamsConfig

lengthProgram = False
toggleLasers = False

def main():
    global pubLength, pubLasers
    rospy.init_node('ops_interface')
  
    pubLength = rospy.Publisher('ops/measure_toggle', Bool, queue_size=1)
    pubLasers = rospy.Publisher('ops/toggle_lasers', Bool, queue_size=1)
    
    def opsCallback(config, level):
      global lengthProgram, toggleLasers

      lengthProgram = config.length_finder
      toggleLasers = config.toggle_lasers
      
      pubLength.publish(lengthProgram)
      pubLasers.publish(toggleLasers)
      
      return config
      

    # setup dynamic reconfigure
    server = Server(opsControlParamsConfig, opsCallback)
    
    rospy.spin()
        
if __name__  == "__main__":
    main()

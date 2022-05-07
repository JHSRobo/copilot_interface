#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool

from dynamic_reconfigure.server import Server
from copilot_interface.cfg import opsControlParamsConfig

toggleLasers = False
lengthProgram = False
photomosaicProgram = False

def main():
    global pubLength, pubLasers, pubPhotomosaic
    rospy.init_node('ops_interface')
    
    pubLasers = rospy.Publisher('ops/toggle_lasers', Bool, queue_size=1)
    pubLength = rospy.Publisher('ops/measure_toggle', Bool, queue_size=1)
    pubPhotomosaic = rospy.Publisher('ops/photomosaic_toggle', Bool, queue_size=1)
    
    def opsCallback(config, level):
      global lengthProgram, toggleLasers, photomosaicProgram
      
      toggleLasers = config.toggle_lasers
      lengthProgram = config.length_finder
      photomosaicProgram = config.photomosaic
      
      
      pubLasers.publish(toggleLasers)
      pubLength.publish(lengthProgram)
      pubPhotomosaic.publish(photomosaicProgram)
      
      
      return config
      

    # setup dynamic reconfigure
    server = Server(opsControlParamsConfig, opsCallback)
    
    rospy.spin()
        
if __name__  == "__main__":
    main()

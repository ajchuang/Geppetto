import os
import sys
import socket
import thread
from collections import deque

# ros imports
import rospy
from std_msgs.msg import String

# main function
if __name__ == "__main__":
    
    print 'Mr.Geppetto (test) is starting.'
    
    
        pub = rospy.Publisher (type, String, queue_size=10)
        rospy.init_node ('kinect', anonymous=True)
        rate = rospy.Rate (10) # 10hz
        
        # Keep the client here
        while True:
            print 'handler started'
            line = sys.stdin.readline ().strip ()
            
            # get the new (raw) data
            pub.publish (line);
            
    
    
#!/usr/bin/python
import sys

g_testing_mode = 'False'

if (len (sys.argv) == 2):
    g_testing_mode = sys.argv[1]
else:
    g_testing_mode = 'False'
    
print '[SYS] set testing mode: ' + g_testing_mode

# imports
import os
import socket
import thread
import util
from collections import deque

if g_testing_mode == 'False':
    # ros imports
    import rospy
    from std_msgs.msg import String

# global variabls
g_tok_q     = deque ()
g_host      = None
g_port      = None

# global variables for ROS publisher
g_pub_vcmd  = None




def vcmd_server_thread ():
   
    global g_host
    global g_port
    
    server = None

    try:
        print '{}@{}'.format (g_port, g_host)
        server = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
        server.bind ((g_host, g_port))
        server.listen (10)
    except Exception, e:
        print 'failed to start Mr.Geppetto (vcmd)'
        print str(e)
        sys.exit ()
    
    # starting the server loop
    while True:
        try:
            conn, addr = server.accept ()
            print 'Connected with ' + addr[0] + ':' + str(addr[1])
            
            #thread.start_new_thread (handler, (g_pub_myo, conn, addr))
        except:
            print 'Mr.Geppetto(myo) is not working'
            server.close ()
            return


def main ():

    global g_host
    global g_port
    
    g_host, g_port, unused = util.read_conf ('vcmd') 

    if g_host == None or g_rec_host == None:
        print 'VCMD is not configured. Exit'
        return

    if g_testing_mode == 'False':
        print 'Initialize ROS nodes - vcmd'
        g_pub_myo = rospy.Publisher ('vcmd', String, queue_size=1)
        rospy.init_node ('vcmd', anonymous=True)
        rate = rospy.Rate (10) # 10hz

    # starting the server
    print 'Starting Mr.Geppetto (vcmd) server @ {}:{}'.format(g_host, g_port)
    vcmd_server_thread ()

# main function
if __name__ == "__main__":
    main ()

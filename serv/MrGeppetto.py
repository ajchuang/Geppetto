import os
import sys
import socket
import thread
from collections import deque

# ros imports
import rospy
from std_msgs.msg import String

# constants
N_REC_SIZE = 6

# global variabls
g_tok_q = deque ()
g_host  = socket.gethostbyname (socket.gethostname ())
g_myo_port     = 4001
g_kinect_port   = 4002

# global variables for ROS publisher
g_pub_kinect    = None
g_pub_myo       = None

# using ros function
def send_ros (pub, list):
    
    sent =        list[0] + ' ' + list[1] + ' ' + list[2] + ' ' + list[3]  + ' ' + list[4]  + ' ' + list[5]  + ' ' + list[6] + ' '
    sent = sent + list[7] + ' ' + list[8] + ' ' + list[9] + ' ' + list[10] + ' ' + list[11] + ' ' + list[12] + ' ' + list[13] + ' '
    
    print 'sendin {}'.format (sent)
    pub.publish (sent);

def parse_input (data, pub):

    # add the new input to the token queue
    g_tok_q.extend (data.strip().split (' '));
    
    while len(g_tok_q) > N_REC_SIZE:

        print 'test 0'

        # check 'Go' tag
        tag = g_tok_q.popleft ()
         
        if tag != 'GO':
            print '{}'.format (tag)
            continue
        
        data = []
        
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        
        # send via ROS functions
        send_ros (pub, data)

def handler (pub, conn, addr):
    
    try:
        print 'handler started'
        
        # Keep the client here
        while True:
            # get the new (raw) data
            new_data = conn.recv (1024)
            parse_input (new_data, pub)
            
    except rospy.ROSInterruptException:
        print 'ros interrupt exception'
        pass
            
def myo_server_thread ():
    
    server = None

    try:
        server = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
        server.bind ((g_host, g_myo_port))
        server.listen (10)
    except:
        print 'failed to start Mr.Geppetto'
        sys.exit ()
    
    # starting the server loop
    while True:
        try:
            conn, addr = server.accept ()
            print 'Connected with ' + addr[0] + ':' + str(addr[1])
            
            thread.start_new_thread (handler, (g_pub_myo, conn, addr))
        except:
            print 'Mr.Geppetto(myo) is not working'
            server.close ()
            return

def kinect_server_thread ():
    
    server = None

    try:
        server = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
        server.bind ((g_host, g_kinect_port))
        server.listen (10)
    except:
        print 'failed to start Mr.Geppetto'
        sys.exit ()
    
    # starting the server loop
    while True:
        try:
            conn, addr = server.accept ()
            print 'Connected with ' + addr[0] + ':' + str(addr[1])
            
            thread.start_new_thread (handler, (g_pub_kinect, conn, addr))
        except:
            print 'Mr.Geppetto(kinect) is not working'
            server.close ()
            return

# main function
if __name__ == "__main__":
    
    # Param processing
    argc = len (sys.argv) 
    
    if argc == 3:
        g_myo_port      = int (sys.argv[1])
        g_kinect_port   = int (sys.argv[2])
    elif argc == 1:
        # use default
        pass
    else:
        print 'incorrect params'
        sys.exit ()
        
    # init ros nodes
    print 'Initialize ROS nodes - kinect'
    g_pub_kinect = rospy.Publisher ('kinect', String, queue_size=10)
    rospy.init_node ('kinect', anonymous=True)
    rate = rospy.Rate (10) # 10hz
    
    print 'Initialize ROS nodes - myo'
    g_pub_myo   = rospy.Publisher ('myo',   String, queue_size=10)
    rospy.init_node ('myo', anonymous=True)
    rate = rospy.Rate (10) # 10hz
    
    # starting the server
    print 'Starting Mr.Geppetto server @ {}:{},{}'.format(g_host, g_myo_port, g_kinect_port)
    thread.start_new_thread (myo_server_thread, ())
    thread.start_new_thread (kinect_server_thread, ())
    
    
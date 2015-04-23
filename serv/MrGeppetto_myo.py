import os
import sys
import socket
import thread
import util
from collections import deque

# ros imports
import rospy
from std_msgs.msg import String

# global variabls
g_tok_q     = deque ()
g_host      = socket.gethostbyname (socket.gethostname ())
g_myo_port  = 4006

# global variables for ROS publisher
g_pub_myo       = None

# using ros function
def send_ros (pub, list):
    
    sent = list[0] + ' ' + list[1] + ' ' + list[2]
    print 'sendin {}'.format (sent)
    pub.publish (sent);

def parse_input (data, pub):

    # add the new input to the token queue
    g_tok_q.extend (data.strip().split (' '));
    
    while len(g_tok_q) > 0:

        # check 'Go' tag
        tag = g_tok_q.popleft ()
         
        if tag != 'GO':
            print '{}'.format (tag)
            continue
        
        data = []
        
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        data.append (g_tok_q.popleft ())
        
        # send via ROS functions
        send_ros (pub, data)
        return;

def handler (pub, conn, addr):
    
    try:
        print 'handler started'
        
        # Keep the client here
        while True:
            # get the new (raw) data
            new_data = conn.recv (1024)
            parse_input (new_data.strip(), pub)
            
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

# main function
if __name__ == "__main__":
    
    g_host, g_myo_port = util.read_conf ('myo') 
    
    print 'Initialize ROS nodes - myo'
    g_pub_myo = rospy.Publisher ('myo',   String, queue_size=10)
    rospy.init_node ('myo', anonymous=True)
    rate = rospy.Rate (10) # 10hz
    
    # starting the server
    print 'Starting Mr.Geppetto (myo) server @ {}:{}'.format(g_host, g_myo_port)
    myo_server_thread ()

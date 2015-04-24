# Compile option 
g_testing_mode = True

import os
import sys
import socket
import thread
import util
from collections import deque

if not g_testing_mode:
    # ros imports
    import rospy
    from std_msgs.msg import String

# global variabls
g_tok_q     = deque ()
g_host      = socket.gethostbyname (socket.gethostname ())
g_myo_port  = 4006
g_rec_host  = None
g_rec_port  = None
g_trans     = list ();
g_trans_len = 3

# global variables for ROS publisher
g_pub_myo   = None

def send_rec (lst):
    sent = 'MYO ' + lst[0] + ' ' + lst[1] + ' ' + lst[2]
    sock = socket.socket (socket.AF_INET, socket.SOCK_DGRAM) # UDP
    sock.sendto (sent, (g_rec_host, int(g_rec_port)))

# using ros function
def send_ros (pub, lst):

    if not g_testing_mode:
        sent = lst[0] + ' ' + lst[1] + ' ' + lst[2]
        print 'sendin {}'.format (sent)
        pub.publish (sent)

def parse_input (data, pub):

    # add the new input to the token queue
    g_tok_q.extend (data.strip().split (' '));
    
    while len(g_tok_q) > 0:

        # check 'Go' tag
        tag = g_tok_q.popleft ()
         
        if tag != 'GO':
            print '{}'.format (tag)
            g_trans.append (tag)
            continue
        else:
            g_trans.append (tag)
       
        # check if the condition is met.
        if len(g_trans) == g_trans_len:
            send_ros (pub, g_trans);
            g_trans = []

        # send via ROS functions
        send_rec (g_trans) 
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
    g_rec_host, g_rec_port = util.read_conf ('recorder')    

    print 'Initialize ROS nodes - myo'
    if not g_testing_mode:
        g_pub_myo = rospy.Publisher ('myo',   String, queue_size=10)
        rospy.init_node ('myo', anonymous=True)
        rate = rospy.Rate (10) # 10hz
    
    # starting the server
    print 'Starting Mr.Geppetto (myo) server @ {}:{}'.format(g_host, g_myo_port)
    myo_server_thread ()

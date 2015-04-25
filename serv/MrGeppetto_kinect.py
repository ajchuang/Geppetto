#!/usr/bin/python

g_testing_mod = True

import os
import sys
import socket
import thread
import util
from collections import deque

if not g_testing_mod:
    # ros imports
    import rospy
    from std_msgs.msg import String

# global variabls
g_tok_q = deque ()
g_host = None
g_kinect_port = None

# global variables for ROS publisher
g_pub_kinect    = None

# using ros function
def send_ros (pub, list):

    global g_testing_mod

    sent =        list[0]  + ' ' + list[1]  + ' ' + list[2]  + ' ' + list[3]  + ' '
    sent = sent + list[4]  + ' ' + list[5]  + ' ' + list[6]  + ' '
    sent = sent + list[7]  + ' ' + list[8]  + ' ' + list[9]  + ' ' + list[10] + ' ' 
    sent = sent + list[11] + ' ' + list[12] + ' ' + list[13] + ' '
    
    print 'sendin {}'.format (sent)

    if not g_testing_mod:
        try:
            pub.publish (sent);
        except rospy.ROSInterruptException:
            print 'ros interrupt exception'
            pass


def parse_input (data, pub):

    global g_tok_q

    # add the new input to the token queue
    g_tok_q.extend (data.strip().split (' '));
    
    while len(g_tok_q) > 0:

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
        return

def handler (pub, conn, addr):
    
    print 'handler started'
        
    # Keep the client here
    while True:
        # get the new (raw) data
        new_data = conn.recv (1024)
        parse_input (new_data, pub)
            
            
def kinect_server_thread ():
    global g_kinect_port

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

def main ():
    
    global g_host
    global g_kinect_port
    global g_testing_mod
    global g_pub_kinect   

    g_host, g_kinect_port = util.read_conf ('kinect')
    
    if g_host == None:
        print 'Kinect is not configured. Exit'
        return

    # init ros nodes
    if not g_testing_mod:
        print 'Initialize ROS nodes - kinect'
        g_pub_kinect = rospy.Publisher ('kinect', String, queue_size=10)
        rospy.init_node ('kinect', anonymous=True)
        rate = rospy.Rate (10) # 10hz
    
    # starting the server
    print 'Starting Mr.Geppetto server @ {}:{}'.format(g_host, g_kinect_port)
    kinect_server_thread ()
    
# main function
if __name__ == "__main__":
    main ()

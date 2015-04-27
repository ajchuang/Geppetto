#!/usr/bin/python
import sys

g_testing_mode = 'False'

if (len (sys.argv) == 2):
    g_testing_mode = sys.argv[1]
else:
    g_testing_mode = 'False'
    
print '[KINECT] set testing mode: ' + g_testing_mode
    
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
g_data  = ''
g_tok_q = deque ()
g_trans = list ()

g_host          = None
g_kinect_port   = None

g_rec_host      = None
g_rec_port      = None

g_roll_ang      = 0.0

# global variables for ROS publisher
g_pub_kinect    = None
g_sample_rate   = None
g_sample_cnt    = 0

# using ros function
def send_ros (pub, list):

    global g_testing_mode

    if list[4] != '0.0' :
        r_ang = list[4]
    else:
        r_ang = str(g_roll_ang)

    sent =        list[0]  + ' ' + list[1]  + ' ' + list[2]  + ' ' + list[3]  + ' '
    sent = sent + r_ang    + ' ' + list[5]  + ' ' + list[6]  + ' '
    sent = sent + list[7]  + ' ' + list[8]  + ' ' + list[9]  + ' ' + list[10] + ' ' 
    sent = sent + list[11] + ' ' + list[12] + ' ' + list[13] + ' '
    
    if g_testing_mode == 'False':
        try:
            pub.publish (sent);
        except rospy.ROSInterruptException:
            print 'ros interrupt exception'
            pass

def send_rec (list):
    
    if list[4] != '0.0' :
        r_ang = list[4]
    else:
        r_ang = str(g_roll_ang)
    
    sent ='KNT '+ list[0]  + ' ' + list[1]  + ' ' + list[2]  + ' ' + list[3]  + ' '
    sent = sent + r_ang    + ' ' + list[5]  + ' ' + list[6]  + ' '
    sent = sent + list[7]  + ' ' + list[8]  + ' ' + list[9]  + ' ' + list[10] + ' ' 
    sent = sent + list[11] + ' ' + list[12] + ' ' + list[13] + ' '
    
    # send to the recorder
    sock = socket.socket (socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto (sent, (g_rec_host, int(g_rec_port)))

def parse_input (data, pub):

    global g_tok_q
    global g_trans
    global g_data
    global g_roll_ang
    global g_sample_cnt

    # add the new input to the token queue
    g_data += data
    g_tok_q.extend (g_data.split (' '));
    
    while len(g_tok_q) > 0:

        # check 'Go' tag
        tag = g_tok_q.popleft ()

        # for other thread 
        if tag == 'RL' and len(g_tok_q) > 1:
            g_roll_ang = g_tok_q.popleft ();

        if tag != 'GO':
            
            try:
                tag = "{0:.2f}".format (float (tag))
                g_trans.append (tag)
            except:
                pass
	
            
            if len (g_trans) == 14:
	
		g_sample_cnt += 1

		if g_sample_rate == g_sample_cnt:
                    g_sample_cnt = 0
                    send_ros (pub, g_trans)
                    send_rec (g_trans)
                    g_trans = []
        else:
            g_trans = []
        
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
    global g_testing_mode
    global g_pub_kinect   
    global g_sample_rate
    global g_sample_cnt
    global g_roll_host
    global g_roll_port
    global g_rec_host
    global g_rec_port

    g_host, g_kinect_port, g_sample_rate = util.read_conf ('kinect')
    g_rec_host, g_rec_port, unused = util.read_conf ('recorder')
    
    if g_sample_rate > 1:
        g_sample_cnt = g_sample_rate - 1;

    if g_host == None or g_rec_host == None:
        print 'Kinect is not configured. Exit'
        return

    print 'rec: {}:{}:{}'.format (g_rec_host, g_rec_port, g_sample_rate)

    # init ros nodes
    if g_testing_mode == 'False':
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

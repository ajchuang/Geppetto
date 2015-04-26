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
g_myo_port  = None
g_rec_host  = None
g_rec_port  = None
g_trans     = list ();
g_trans_len = 2
g_is_rec    = False

# global variables for ROS publisher
g_pub_myo   = None

def send_str (s):
    sock = socket.socket (socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto (s, (g_rec_host, int(g_rec_port)))

def send_rec (lst):
    if g_is_rec:
        sent = 'MYO ' + lst[0] + ' ' + lst[1]
        send_str (sent)

# using ros function
def send_ros (pub, lst):

    if g_testing_mode == 'False':
        sent = lst[0] + ' ' + lst[1]
        print 'sendin {}'.format (sent)
        
        try:
            pub.publish (sent)
            return True
        except Exception, e:
            print str(e)
            return False

def parse_input (data, pub):

    global g_tok_q
    global g_trans
    global g_trans_len
    global g_is_rec

    # add the new input to the token queue
    g_tok_q.extend (data.split (' '));
    
    while len(g_tok_q) > 0:

        # check 'Go' tag
        tag = g_tok_q.popleft ()
    
        if tag == 'START' or tag == 'END':
            if tag == 'START':
                g_is_rec = True
            else:
                g_is_rec = False

            send_str (tag)
            continue

        if tag != 'GO':
            g_trans.append (tag)
            
            # check if the condition is met.
            if len(g_trans) == g_trans_len:
                send_ros (pub, g_trans);
                send_rec (g_trans)
                g_trans = []
                continue
        else:
            g_trans = []

def handler (pub, conn, addr):
    
    print 'handler started'
    
    # Keep the client here
    while True:
        try:
            # get the new (raw) data
            new_data = conn.recv (1024)
            parse_input (new_data.strip(), pub)
        except:
            print 'disconnected - abort'
            return

def myo_server_thread ():
   
    global g_host
    global g_myo_port
    global g_pub_myo
    
    server = None

    try:
        print '{}@{}'.format (g_myo_port, g_host)
        server = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
        server.bind ((g_host, g_myo_port))
        server.listen (10)
    except Exception, e:
        print 'failed to start Mr.Geppetto (myo)'
        print str(e)
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

def main ():

    global g_host
    global g_myo_port
    global g_rec_host
    global g_rec_port
    global g_pub_myo

    g_host, g_myo_port, unused = util.read_conf ('myo') 
    g_rec_host, g_rec_port, unused = util.read_conf ('recorder')    

    if g_host == None or g_rec_host == None:
        print 'MYO is not configured. Exit'
        return

    if g_testing_mode == 'False':
        print 'Initialize ROS nodes - myo'
        g_pub_myo = rospy.Publisher ('myo', String, queue_size=10)
        rospy.init_node ('myo', anonymous=True)
        rate = rospy.Rate (10) # 10hz
    
    # starting the server
    print 'Starting Mr.Geppetto (myo) server @ {}:{}'.format(g_host, g_myo_port)
    myo_server_thread ()

# main function
if __name__ == "__main__":
    main ()
    

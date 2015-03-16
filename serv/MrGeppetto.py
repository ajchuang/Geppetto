import os
import sys
import socket
import thread
from collections import deque

# constants
N_REC_SIZE = 6

# global variabls
g_tok_q = deque ()
g_host, g_port = socket.gethostbyname (socket.gethostname ()), 4001

# using ros function
def send_ros (cmd, ts, x, y, z):
    print "ros send: {}:{}:{}:{}:{}".format (cmd, ts, x, y, z)

def parse_input (data):

    # add the new input to the token queue
    g_tok_q.extend (data.strip().split (' '));
    
    while len(g_tok_q) > N_REC_SIZE:

        print 'test 0'

        # check 'Go' tag
        tag = g_tok_q.popleft ()
         
        if tag != 'GO':
            print '{}'.format (tag)
            continue

        cmd = g_tok_q.popleft () 
        ts  = g_tok_q.popleft ()
        x   = g_tok_q.popleft ()
        y   = g_tok_q.popleft ()
        z   = g_tok_q.popleft ()
        
        # send via ROS functions
        send_ros (cmd, ts, x, y, z)

def handler (conn, addr):
    
    # Keep the client here
    while True:
        print 'handler started'
        # get the new (raw) data
        new_data = conn.recv (256)
        parse_input (new_data)
            
def server_thread ():
    
    server = None

    try:
        server = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
        server.bind ((g_host, g_port))
    except:
        print 'failed to start Mr.Geppetto'
        sys.exit ()
    
    server.listen (10)
    
    while True:
        try:
            conn, addr = server.accept ()
            print 'Connected with ' + addr[0] + ':' + str(addr[1])
            thread.start_new_thread (handler, (conn, addr))
        except:
            print 'Mr.Geppetto(server) is not working'
            server.close ()
            return

# main function
if __name__ == "__main__":
    
    print 'Mr.Geppetto is starting @ {}:{}'.format(g_host, g_port)
    thread.start_new_thread (server_thread, ())
    
    while True:
        line = sys.stdin.readline ().strip ()
        print 'admin cmd: {}'.format (line)
        
        if line == 'quit':
            print 'Mr. Geppetto is done. (admin request)'
            sys.exit ()
    
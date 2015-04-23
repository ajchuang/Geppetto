import os
import os.path
import sys
import socket
import thread
from collections import deque

import util

# global variabls
g_tok_q = deque ()
g_host  = 'localhost'
g_port  = 4008
g_isRec = False
g_fname = None
g_fh    = None

def next_file_name ():
    num = 1

    while True:
        file_name = 'recording_%d.txt' % num
        
        if not os.path.exists (file_name):
            return file_name
        
        num += 1

def data_handler (data):
    
    if g_isRec:
        if data == 'STOP':
            g_fh.close ()
            print 'Recording completed - ' + g_fname
            
            g_fh = None
            g_isRec = False
            g_fname = next_file_name ()
        else:
            if data == 'START':
                return
            g_fh.write (data + '\n')
    else:
        if data == 'START':
            print 'Start to record @ ' + g_fname
            g_fname = next_filename ()
            g_fh = open (g_fname, 'a')
        else:
            if data == 'STOP':
                return
            g_fh.write (data + '\n')


def recorder_server_thread ():
    
    server = None

    try:
        server = socket.socket (socket.AF_INET, socket.SOCK_DGRAM)
        server.bind ((g_host, g_port))
    except:
        print 'failed to start Mr.Geppetto (Rec)'
        sys.exit ()
    
    # starting the server loop
    while True:
        data,addr = server.recvfrom (1024)
        print data.strip(), addr
        data_handler (data.strip ())


# main function
if __name__ == "__main__":
    
    dic = util.read_all_conf ()
    print dic

    # starting the server
    print 'Starting Mr.Geppetto recorder @ {}:{}'.format(g_host, g_port)
    recorder_server_thread ()

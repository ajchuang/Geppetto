#!/usr/bin/python

# common import
import os
import os.path
import sys
import socket
import thread
from collections import deque

# local import
import util

# global variabls
g_conf  = None
g_host  = None
g_port  = None
g_isRec = False
g_fname = None
g_fh    = None
g_beginTime = None

def data_handler (data):
    
    if g_isRec:
        if data == 'STOP':
            g_fh.close ()
            print 'Recording completed - ' + g_fname
            
            g_fh = None
            g_isRec = False
            g_fname = next_file_name ()
            g_beginTime = None
        else:
            if data == 'START':
                return
            
            curTime = int (datetime.datetime.now ())
            
            g_fh.write ((curTime - g_beginTime) + data + '\n')
    else:
        if data == 'START':
            print 'Start to record @ ' + g_fname
            g_fname = next_filename ()
            g_fh = open (g_fname, 'a')
            g_beginTime = int (datetime.datetime.now ())

def recorder_server_thread ():

    global g_host
    global g_port
    server = None

    try:
        server = socket.socket (socket.AF_INET, socket.SOCK_DGRAM)
        server.bind ((g_host, g_port))
    except Exception as e:
        print 'failed to start Mr.Geppetto (Rec) - Maybe port in use ?'
        print e
        sys.exit ()

    # starting the server loop
    while True:
        data,addr = server.recvfrom (1024)
        print data.strip(), addr
        data_handler (data.strip ())

def main ():
    global g_host
    global g_port

    g_conf = util.read_all_conf ()
    g_host, g_port = util.read_conf ('recorder')

    if g_host == None:
        print 'Recorder is not configured. Exis'
        return

    # starting the server
    print 'Starting Mr.Geppetto recorder @ {}:{}'.format(g_host, g_port)
    recorder_server_thread ()

# main function
if __name__ == "__main__":
    main ()

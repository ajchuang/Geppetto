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
g_host  = None
g_port  = None
g_isRec = False
g_fname = None
g_fh    = None
g_beginTime = None

def next_file_name():
    num = 1
    
    while True:
        file_name = './rec/recording_%d.txt' % num
        if not os.path.exists (file_name):
            return file_name
        num += 1

def data_handler (data):
    
    global g_isRec
    global g_fname
    global g_fh
    global g_beginTime
    
    if g_isRec:
        if data == 'END':
            g_fh.close ()
            print 'Recording completed - ' + g_fname
            
            g_fh = None
            g_fname = None
            g_beginTime = None
            g_isRec = False
        else:
            if data == 'START':
                return
            
            curTime = util.unix_now ()
            r = str(curTime - g_beginTime) + ' ' + data + '\n'
            
            print '[Rec] write :' + r
            g_fh.write (r)
    else:
        if data == 'START':
            g_fname = next_file_name ()
            print 'Start to record @ ' + g_fname
            
            g_fh = open (g_fname, 'w')
            g_beginTime = util.unix_now ()
            g_isRec = True

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

    g_host, g_port, unused = util.read_conf ('recorder')

    if g_host == None:
        print 'Recorder is not configured. Exis'
        return

    # starting the server
    print 'Starting Mr.Geppetto recorder @ {}:{}'.format(g_host, g_port)
    recorder_server_thread ()

# main function
if __name__ == "__main__":
    main ()

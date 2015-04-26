#!/usr/bin/python

import os
import os.path
import sys
import socket
import thread
import util
from time import sleep
from datetime import datetime
from collections import deque

# global variabls
g_fname = None
g_sock_kinect   = None
g_kinect_host   = None
g_kinect_port   = None
g_sock_myo      = None
g_myo_host      = None
g_myo_port      = None

g_diff = 50 #ms

def init_conn ():
    global g_sock_kinect
    global g_sock_myo
    
    g_sock_kinect = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
    g_sock_kinect.connect ((g_kinect_host, g_kinect_port))
    g_sock_myo = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
    g_sock_myo.connect ((g_myo_host, g_myo_port))

def send_kinect_cmd (toks):
    cmd =  'GO '   + toks[2]  + ' ' + toks[3]  + ' ' + toks[4]  + ' ' + toks[5]  
    cmd += ' '     + toks[6]  + ' ' + toks[7]  + ' ' + toks[8]  + ' ' + toks[9]
    cmd += ' '     + toks[10] + ' ' + toks[11] + ' ' + toks[12] + ' ' + toks[13] 
    cmd += ' '     + toks[14] + ' ' + toks[15]
    g_sock_kinect.send (cmd) 

def send_myo_cmd (toks):
    cmd =  'GO '   + toks[2] + ' ' + toks[3]
    g_sock_myo.send ()

def exec_line (cmd, startTime):
    
    toks = cmd.split () 
    dev = toks[1]
    etime = float (toks[0])
    ctime = float (util.unix_now ())

    # recording format:
    #   time dev data...   
    # waiting for execution time
    while ctime - startTime < 0:
        sleep (0.05)
        ctime = float (util.unix_now ())
        
        print 'exec {' + str(line) + '} @ ' + str(ctime)

        if dev == 'KNT':
            send_kinect_cmd (toks)
            break;

        if dev == 'MYO':
            send_myo_cmd (toks)
            break;

def replay ():

    st = util.unix_now ()
    print 'starting time: ' + str(st)

    with open (g_fname) as rec:
        for line in rec :
            exec_line (line, st)

def main ():
    global g_kinect_host
    global g_kinect_port
    global g_myo_host
    global g_myo_port
    global g_fname
   
    if (len (sys.argv) != 2):
        print '[Error] Please specify recording file correctly'
        print '[Usage] python MrGeppetto_replay.py [recording]'
        return

    g_fname = sys.argv[1]
    
    if not os.path.isfile (g_fname):
        print '[Error] Recording file ' + g_fname + ' does not exist'
        return
    else:
        print 'Replaying file: ' + g_fname

    g_kinect_host, g_kinect_port, unused = util.read_conf ('kinect');
    g_myo_host, g_myo_port, unused = util.read_conf ('myo');    

    init_conn ()
    
    # starting the server
    replay ()
    

# main function
if __name__ == "__main__":    
    main ()
   

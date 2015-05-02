#!/usr/bin/python
import speech_recognition as sr
import sys, socket

###############################################################################
# global vars                                                                 #
###############################################################################
g_host      = None
g_port      = None
g_socket    = None

###############################################################################
# functions                                                                   #
###############################################################################
def send_cmd (cmd):
    try:
        g_socket.send ('GO ' + cmd + ' ')
    except:
        print '[vcmd] comm failed'
    
    return

def v_rec ():
    r = sr.Recognizer()

    while True:
        with sr.Microphone () as source:                # use the default microphone as the audio source
       
            try:
                print 'Please say the command: '
                audio = r.listen (source)               # listen for the first phrase and extract it into audio data
                cmd = r.recognize (audio)

                if cmd == 'go':
                    print 'go forward'
                
                elif cmd == 'back':
                    print 'go back'

                elif cmd == 'stop':
                    print 'stop now'

                elif cmd == 'turn left':
                    cmd = 'left'
                    print 'turn left'

                elif cmd == 'turn right':
                    cmd = 'right'
                    print 'turn right'

                elif cmd == 'exit':
                    print 'Bye-bye'
                    break
                else:
                    print 'cmd: ' + cmd
                    continue

                send_cmd (cmd)

            except LookupError:                         # speech is unintelligible
                print "[ERROR] Could not understand audio"
                pass

def init_conn ():
    global g_host
    global g_port
    global g_socket

    try:
        g_host = sys.argv[1]
        g_port = int (sys.argv[2])
        
        g_socket = socket.socket (socket.AF_INET, socket.SOCK_STREAM)
        g_socket.connect (g_host, g_port)
    except:
        print '[vcmd] Ooops - bad input: [Host] [Port]'

    print '[vcmd] conn established'
    return

def main ():
    init_conn ()
    v_rec ()


if __name__ == '__main__':
    main ()

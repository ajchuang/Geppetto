import socket, sys, threading, time, os, signal, util

#ros imports
import rospy
from std_msgs.msg import String

global SERVER_SOCKET

SERVER_SOCKET = socket.socket()

class OculusThread(threading.Thread):

    def __init__(self, ip, port, socket, pub):
        threading.Thread.__init__(self)
        self.ip = ip
        self.port = port
        self.socket = socket
        self.pub = pub
        print "[*] " + "New thread started for " + ip + ":" + str(port)

    def run(self):

        print "[*] " + "Connection from : "+ self.ip +":" + str(self.port)
        while True:
            msg = self.socket.recv(2048)
            sendROS(pub, msg)
            print msg


def sendROS(pub, sendStr):
    sendList = sendStr.strip().split()
    if sendList[0] == "s":
        sendList = sendList[4:7]
        newStr = '  '.join(sendList)
        pub.publish(sendStr)
    return
    

#exit gracefully function
def gracefulExit(signum, frame):
    print ("[*] " + "\n\nClosing server...\n\n")
    time.sleep(1)
    #closing all the threads
    #for thd in THREADS:
        #thd.end() 
    SERVER_SOCKET.close()
    try:
        sys.exit(0)
    except:
        os._exit(0)


def main():
    
    print "[*] " + "\n\nOculus bridge server started\n\n"
    
    pub = rospy.Publisher('Oculus', String, queue_size=10)
    rospy.init_node('Oculus', anonymous = True)
    signal.signal(signal.SIGINT, gracefulExit)
    port = 10000
    SERVER_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  
    # host = "127.0.0.1"
    host = socket.gethostbyname(socket.gethostname())
    SERVER_SOCKET.bind((host, port))
    SERVER_SOCKET.listen(4)
    while True:
        (oculusServerSocket, (ip, port)) = SERVER_SOCKET.accept()
        oculusThread = OculusThread(ip,port,oculusServerSocket)
        oculusThread.start()

if  __name__ == "__main__":
    main()
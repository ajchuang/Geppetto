import socket, sys, threading, time, os, signal


#ros imports
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint)
from copy import copy

global SERVER_SOCKET
global POS = [0, 0]

SERVER_SOCKET = socket.socket()

class ROSThread(threading.Thread):

    def run(self):
        global POS
        pub = rospy.Publisher ("/head_traj_controller/command", JointTrajectory, queue_size=10)
        rospy.init_node ('HeadTajectoryPublisher', anonymous=True)
        print "[*] ROS publisher initialized"
        while True:
            JTpoint = JointTrajectoryPoint()
            JTpoint.positions = POS
            JTpoint.velocities = [0,0]
            JTpoint.accelerations = [0,0]
            JTmsg = JointTrajectory()
            JTmsg.joint_names = ["head_pan_joint", "head_tilt_joint"]
            JTmsg.points = []
            JTmsg.points.append(copy(JTpoint))
            pub.publish(JTmsg)

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
            sendROS(self.pub, msg)
            print msg


def sendROS(pub, sendStr):
    global POS
    sendList = sendStr.strip().split()
    if sendList[0] == "s":
        POS = sendList[1:3]
        # newStr = '  '.join(sendList)
        # pub.publish(newStr)
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
    port = int(sys.argv[2])
    SERVER_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  
    # host = "127.0.0.1"
    # host = socket.gethostbyname(socket.gethostname())
    # host = "192.168.1.139"
    host = sys.argv[1]
    SERVER_SOCKET.bind((host, port))
    SERVER_SOCKET.listen(4)
    while True:
        (oculusServerSocket, (ip, port)) = SERVER_SOCKET.accept()
        oculusThread = OculusThread(ip,port,oculusServerSocket,pub)
        oculusThread.start()

if  __name__ == "__main__":
    main()

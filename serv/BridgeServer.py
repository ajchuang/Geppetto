import socket, sys, threading, time, os, signal


#ros imports
import rospy
from std_msgs.msg import String
from trajectory_msgs.msg import (JointTrajectory, JointTrajectoryPoint)
from copy import copy

global SERVER_SOCKET
global POS
global START_POS
global FIRST


POS = [0, 0]
START_POS = [0, 0]
FIRST = True
SERVER_SOCKET = socket.socket()

class ROSThread(threading.Thread):
    def __init__(self, pub):
	threading.Thread.__init__(self)
	self.pub = pub
    def run(self):
        global POS
        while True:
            JTpoint = JointTrajectoryPoint()
            JTpoint.positions = POS
            JTpoint.velocities = [0,0]
            JTpoint.accelerations = [0,0]
            JTmsg = JointTrajectory()
            JTmsg.joint_names = ["head_pan_joint", "head_tilt_joint"]
            JTmsg.points = []
            JTmsg.points.append(copy(JTpoint))
            self.pub.publish(JTmsg)

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
    global FIRST
    global START_POS
    global POS
    sendList = sendStr.strip().split()
    if sendList[0] == "s":
        tmpPos = [sendList[3],sendList[2]]
	tmpPos[0] = float("%.3f" % ((10) * float(tmpPos[0])))
	tmpPos[1] = float("%.3f" % ((-10) * float(tmpPos[1])))
	if FIRST:
	    print "First!"
	    START_POS = tmpPos
	    START_POS[1] = 0
	    FIRST = False
	POS = [tmpPos[0] - START_POS[0], tmpPos[1] - START_POS[1]]
	print POS
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
	    
    pub = rospy.Publisher ("/head_traj_controller/command", JointTrajectory, queue_size=1)
    rospy.init_node ('HeadTrajectoryPublisher', anonymous=True)
    print "[*] ROS publisher initialized"
    #pub = rospy.Publisher('Oculus', String, queue_size=10)
    #rospy.init_node('Oculus', anonymous = True)
    signal.signal(signal.SIGINT, gracefulExit)
    port = int(sys.argv[2])
    SERVER_SOCKET.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  
    # host = "127.0.0.1"
    # host = socket.gethostbyname(socket.gethostname())
    # host = "192.168.1.139"
    host = sys.argv[1]
    SERVER_SOCKET.bind((host, port))
    SERVER_SOCKET.listen(4)
    rosThread = ROSThread(pub)
    rosThread.start()
    while True:
        (oculusServerSocket, (ip, port)) = SERVER_SOCKET.accept()
        oculusThread = OculusThread(ip,port,oculusServerSocket,pub)
        oculusThread.start()

if  __name__ == "__main__":
    main()

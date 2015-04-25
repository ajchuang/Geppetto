import socket, sys, threading, time, os, signal

# ros imports
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image

global cameraData
global ip
global port

cameraData = None
ip = sys.argv[2]
port = sys.argv[3]


class SendThread(threading.Thread):

	def __init__(self, socket):
		self.socket = socket

	def run(self):
		print "[*] Server found! Sending data..."
		while True:
			socket.send(data)


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.height)
	cameraData = data.data

def cameraListener():
	rospy.init_node('CameraProxy', anonymous=True)
	rospy.Subscriber("/wide_stereo/left/image_color", Image, callback)
	rospy.spin()
	
def socketInit(ip, port):
	clientSocket = socket.socket()
	clientSocket.connect((ip, port))
	return clientSocket

if __name__ == "__main__":
	try:
		clientSocket = socketInit(ip, port)
		sendThread = SendThread(clientSocket)
		sendThread.start()
	except:
		print "[*] Server not found. Running local mode."

	cameraListener()

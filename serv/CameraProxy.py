#!/usr/bin/python
import socket, sys, threading, time, os, signal
import util

# ros imports
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image

global clientSocket
global cameraData
global ip
global port
global count

count = 0
cameraData = "init"

ip, port, unused = util.read_conf ('cam_proxy')

#ip = sys.argv[1]
#port = int(sys.argv[2])


clientSocket = socket.socket()

class SendThread(threading.Thread):

	def __init__(self, socket):
		threading.Thread.__init__(self)
		self.socket = socket

	def run(self):
		print "[*] Server found! Sending data..."
		while True:
		#	print str(cameraData)
			self.socket.send(cameraData)


def callback(data):
	global cameraData
	global clientSocket
#	rospy.loginfo(rospy.get_caller_id() + "height %s", data.height)
#	rospy.loginfo(rospy.get_caller_id() + "width %s", data.width)
#	rospy.loginfo(rospy.get_caller_id() + "encoding %s", data.encoding)
#	rospy.loginfo(rospy.get_caller_id() + "data len %s",len(data.data))
	sendData = "G" + data.data
	clientSocket.send(sendData)

	# cameraData = data.data
	#clientSocket.send(data.data)



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
		# sendThread = SendThread(clientSocket)
		# sendThread.start()
	except:
		print "[*] Server not found. Running local mode."

	cameraListener()

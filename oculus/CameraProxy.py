import socket, sys, threading, time, os, signal

# ros imports
import rospy
from std_msg.msg import String

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
	rospy.init_node('CameraProxy', anonymous=True)
	rospy.Subcriber("/wide_stereo/left/image_color", String, callback)
	#rospy.spin()


if __name__ == "__main__":
	main()
import socket, sys, threading, time, os, signal

# ros imports
import rospy
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image

def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.height)

def main():
	rospy.init_node('CameraProxy', anonymous=True)
	rospy.Subscriber("/wide_stereo/left/image_color", Image, callback)
	rospy.spin()


if __name__ == "__main__":
	main()

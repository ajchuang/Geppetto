import socket, sys, threading, time, os, signal

# opencv imports


import cv2

import numpy as np





global port
global cameraData

port = int(sys.argv[1])
cameraData = None


class ImageDisplayThread(threading.Thread):

    def run(self):
        global cameraData
        dump = ""
        while True:
            if (cameraData != None) and (cameraData[0] != "i"):
                print cameraData
                count += 1
                if (count == -1):
                    break
        # gray = cv.CreateImage((100, 100), 8, 1)
        # cv.ShowImage("sample", gray)
        # cv.WaitKey(0)
        # print "running image thread"
        # img = cv2.imread('test.jpg')
        # print "showing image"
        # cv2.imshow("test",img)
        # print "image showed"
        # cv2.waitKey(0)

class ClientThread(threading.Thread):

    def __init__(self, ip, port, socket):
        threading.Thread.__init__(self)
        self.socket = socket
        self.ip = ip
        self.port = port
        print "[*] Connection from " + ip + ":" + str(port)

    def run(self):
        writeFile = open("dump.bmp", "w")
        dump = ""
        global cameraData
        while True:
            #print "receiving"
            receive = self.socket.recv(2048) 
            writeFile.write(receive)
        

def socketInit(port):
    print "Camera server started, listening on port " + str(port)
    serverSocket = socket.socket()
    ip = socket.gethostbyname(socket.gethostname())
    serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)          
    serverSocket.bind((ip, port))
    return serverSocket    



if __name__ == "__main__":
    imageDisplayThread = ImageDisplayThread()
    imageDisplayThread.start()
    serverSocket = socketInit(port)
    serverSocket.listen(4)

    

    while True:
        (clientSocket, (clientIP, clientPort)) = serverSocket.accept()
        clientThread = ClientThread(clientIP, clientPort, clientSocket)
        clientThread.start()


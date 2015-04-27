import socket, sys, threading, time, os, signal

# opencv imports
import cv2
import numpy as np
from cv2 import cv


global streamImg
global port
global cameraData

port = int(sys.argv[1])
cameraData = None
streamImg = None


class ImageDisplayThread(threading.Thread):

    def run(self):
        global cameraData
        dump = ""
        # while True:
        #     if (cameraData != None) and (cameraData[0] != "i"):
        #         print cameraData
        #         count += 1
        #         if (count == -1):
        #             break
        # gray = cv.CreateImage((100, 100), 8, 1)
        # cv.ShowImage("sample", gray)
        # cv.WaitKey(0)
        # print "running image thread"
        #img = cv2.imread('test.jpg')
        # print "showing image"
        #cv2.imshow("test",img)
        # print "image showed"
        #cv2.waitKey(0)

class ClientThread(threading.Thread):

    def __init__(self, ip, port, socket):
        threading.Thread.__init__(self)
        self.socket = socket
        self.ip = ip
        self.port = port
        print "[*] Connection from " + ip + ":" + str(port)

    def run(self):
        tmp = ""
        global cameraData
        global streamImg
        while True:
            #print "receiving"
            receive = self.socket.recv(2048)
            index = receive.find("GO")
            if (index > -1):
                tmp += receive[:index]
                cameraData = tmp
                streamImg = transformation(np.fromstring(cameraData, np.uint8))
                tmp = receive[index + 2:]
            else:
                tmp += receive

        

def socketInit(port):
    print "Camera server started, listening on port " + str(port)
    serverSocket = socket.socket()
    ip = socket.gethostbyname(socket.gethostname())
    serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)          
    serverSocket.bind((ip, port))
    return serverSocket    

def transformation(bgrList):
    #img = [[[None] * 3] * 640] * 480]
    nparr = np.ndarray(shape=(480, 640, 3), dtype=np.uint8)
    for i in range(0, 480):
        for j in range(0, 640):
            for k in range(0,3):
                nparr[i][j][k] = bgrList[i*640*3 + j*3 + k]
    return nparr




if __name__ == "__main__":
    imageDisplayThread = ImageDisplayThread()
    imageDisplayThread.start()
    serverSocket = socketInit(port)
    serverSocket.listen(4)
    (clientSocket, (clientIP, clientPort)) = serverSocket.accept()
    clientThread = ClientThread(clientIP, clientPort, clientSocket)
    clientThread.start()
    #img = cv2.imread('test.jpg')
    #cv2.imshow("test",img)
    #cv2.waitKey(0)
    while True:
        try:
            cv2.imshow('PR2', streamImg)
        except:
            print "not yet"
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # while True:
    #     (clientSocket, (clientIP, clientPort)) = serverSocket.accept()
    #     clientThread = ClientThread(clientIP, clientPort, clientSocket)
    #     clientThread.start()


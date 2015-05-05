import socket, sys, threading, time, os, signal

# opencv imports
import cv2
import numpy as np
from cv2 import cv


global streamImg
global port
global cameraData


cameraData = None
streamImg = None


# class ImageDisplayThread(threading.Thread):

#     def run(self):
#         global cameraData
#         dump = ""
        

class ClientThread(threading.Thread):

    def __init__(self, ip, port, socket):
        threading.Thread.__init__(self)
        self.socket = socket
        self.ip = ip
        self.port = port
        print "[*] Connection from " + ip + ":" + str(port)

    def run(self):
        tmp = " " * 921600
        global cameraData
        global streamImg
        while True:
            #print "receiving"
            receive = self.socket.recv(2048)
            index = receive.find("GO")
            if (index > -1) and ((len(tmp) + index) == 921600):
                print "Good!"
                tmp += receive[:index]
                cameraData = tmp
                #if (len(cameraData) == 921600):
                #    print "Good!"
                streamImg = transformation(np.fromstring(cameraData, np.uint8))
                tmp = receive[index + 2:]
            else:
                tmp += receive

        

def socketInit(port):
    print "Camera server started, listening on port " + str(port)
    serverSocket = socket.socket()
    #ip = socket.gethostbyname(socket.gethostname())
    #ip = "209.2.216.211"
    ip = sys.argv[1]
    port = int(sys.argv[2])
    print ip
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

def faceDetect(image, faceCascade):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    faces = faceCascade.detectMultiScale(\
        gray, \
        scaleFactor = 1.1, \
        minNeighbors = 5, \
        minSize = (30, 30), \
        flags =cv.CV_HAAR_SCALE_IMAGE)
    # print "Found {0} faces!".format(len(faces))
    for (x, y, w, h) in faces:
        cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
    image = cv2.resize(image, (1440, 900), interpolation = cv2.INTER_CUBIC)
    return image

def histEqual(image):
    ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCR_CB)
    #channels = [0, 0, 0]
    channels = cv2.split(ycrcb)
    print len(channels)
    cv2.equalizeHist(channels[0], channels[0])
    ycrcb = cv2.merge(channels)
    result = cv2.cvtColor(ycrcb,cv2.COLOR_YCR_CB2BGR)
    return result

def display():
    faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
    while True:
        if (sys.argv[1] == "local"):
            test = cv2.imread("edwin.jpg")
            rows,cols,rgb = test.shape
            M = cv2.getRotationMatrix2D((cols/2,rows/2),-90,1)
            test = cv2.warpAffine(test,M,(cols,rows))
            displayImg = faceDetect(test, faceCascade)
            displayImg = oculusResize(displayImg, displayImg, 900, 1440, 450)
            cv2.imwrite('Camera.bmp', displayImg)
        if (streamImg != None):
            try:
                displayImg = histEqual(streamImg)
                displayImg = faceDetect(displayImg, faceCascade)
                displayImg = oculusResize(displayImg, displayImg, 900, 1440, 450)
                cv2.imwrite('Camera.bmp', displayImg)
                #cv2.imshow('PR2', displayImg)
            except:
                print "No image yet"
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

def oculusResize(leftImage, rightImage, height, width, imgHeight):
    blackHeight = (height - imgHeight) / 2
    blackWidth = width
    blackImage = np.zeros((blackHeight, blackWidth,3), np.uint8)
    image = np.concatenate((leftImage, rightImage), axis=1)
    image = cv2.resize(image, (width, imgHeight), interpolation = cv2.INTER_CUBIC)
    image = np.concatenate((blackImage, image), axis=0)
    image = np.concatenate((image, blackImage), axis=0)
    return image


if __name__ == "__main__":
    # imageDisplayThread = ImageDisplayThread()
    # imageDisplayThread.start()
    if sys.argv[1] != "local":
        port = 11111
        serverSocket = socketInit(port)
        serverSocket.listen(4)
        (clientSocket, (clientIP, clientPort)) = serverSocket.accept()
        clientThread = ClientThread(clientIP, clientPort, clientSocket)
        clientThread.start()
    display()
    #img = cv2.imread('test.jpg')
    #cv2.imshow("test",img)
    #cv2.waitKey(0)

    # while True:
    #     if (streamImg != None):
    #         try:
    #             cv2.imshow('PR2', streamImg)
    #         except:
    #             print "not yet"
    #     if cv2.waitKey(1) & 0xFF == ord('q'):
    #         break

    # while True:
    #     (clientSocket, (clientIP, clientPort)) = serverSocket.accept()
    #     clientThread = ClientThread(clientIP, clientPort, clientSocket)
    #     clientThread.start()


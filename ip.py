'''
    A utility tool for both camera and image processing operations.
'''
import os
#os.system('sudo chmod 666 /dev/ttyTHS1')
import cv2
import numpy as np
import math
import time
#from dronekit import connect, VehicleMode
from ugvModel import Model
# from objectdetection import ObjectDetection


class ImageProcessing:
    # output attributes
    objectAngle = None
    forwardTime = None

    prevMarker = None   # marker at previous frame
    lastSeenMarker = None # last detected numeric marker, cannot be None.
    waitTimes = dict(wait_time=2, waiting=[])

    # Miscellaneous attributes
    frame = None
    cam = None
    cameraStatus = False
    toDisplay = True


    canvas = None
    objectDetector = None
    camtype = 'sjCam'


    # Required Functions
    
    def __init__(self):
        #self.initVehicle()
        pass

    def initVehicle(self):
        #self.vehicleIp = connect('/dev/ttyTHS1', baud=57600, wait_ready=True)
        self.vehicleIp.armed = True
        print('Vehicle in ip is ready')
        print('Vehicle waiting to start visual servo control')


    def publish(self, param=None):

        #while self.vehicleIp.channels['8'] == 1495:
        #    print('Visual servo not triggered')
        #    time.sleep(1)
        #self.vehicleIp = None
        # print("Visual servo control now starting")
        self.openCamera(param=param)
        #self.setCurrentFrame(param=param) #Nagloop ra ni siya sa livestream. Wala nipadayon.
        self.setCurrentFrameCapture(param=param)



    def openCamera(self, param=None):
        if not self.isCameraOpen(param):
            # self.cam = cv2.VideoCapture("/home/jetson/multiprocessing_final/test_unseen2_a.MP4")
            self.cam = cv2.VideoCapture(-1)
            self.cameraStatus = True
            print("Camera opened: ", self.cameraStatus)
            if param is not None:
                param['camStatus'].value = True

    
    def isCameraOpen(self, param=None):
        return self.cameraStatus if param is None else param['camStatus'].value
    
    def setCurrentFrame(self, param=None):
        while self.isCameraOpen(param):
            ret, frame = self.cam.read()
            #frame = self.adjustBrightContrast(frame)
            #frame = self.centerSquare(frame)
            #frame = cv2.rotate(frame, cv2.ROTATE_180)
            self.frame = frame
            if param is not None:
                param['frame'].value = frame
            if self.toDisplay:
                #cv2.imshow('a', frame)
                #cv2.waitKey(1)
                continue
        #self.cam.release()
        #cv2.destroyAllWindows()   


    def setCurrentFrameCapture(self, param=None):
        while self.isCameraOpen(param):
            ret, frame = self.cam.read()

            #cv2.imwrite('CurrentImage.png', frame)
            #self.frame = cv2.imread('/home/johnmel/Documents/ObjectDetection/MultiprocessTwo/CurrentImage.png')

            self.frame = cv2.resize(frame,(640, 640)) #enable if live video
            

            object = self.detectMarker()
            if object is not None:
                imageCenter = self.getImageCenter(object)
                self.forwardTime = self.getForwardTime(object, imageCenter)
                self.objectAngle = self.getObjectAngle(object, imageCenter)
                #print("Angle: ", self.objectAngle)
                if param is not None:
                    param['forwardTime'].value = self.forwardTime
                    param['objectAngle'].value = self.objectAngle
                    param['marker'].value = object
                    param['imageCenter'].value = imageCenter
                else:
                    param['forwardTime'].value = 0
                    param['objectAngle'].value = 90
            else:
                param['forwardTime'].value = 0
                param['objectAngle'].value = 90

            if self.toDisplay:
                cv2.circle(self.canvas, (320, 320), 3, (0, 255, 0), -1)
                # cv2.imshow('a', self.canvas) #comment if program runs at start up using service
                # cv2.waitKey(1)
                continue
                
        self.cam.release()
        cv2.destroyAllWindows()       


    
    def detectMarker(self):
        # self.canvas = self.track(self.preprocessFrame())
        self.canvas = self.frame
        #self.canvas = cv2.resize(self.frame,(640,640))

        if self.objectDetector is None:
            self.objectDetector = Model()
        marker = self.detectObject()
        
        return self.checkMarkerHistory(marker)

    def preprocessFrame(self):
        #self.frame = cv2.flip(self.frame, 1)
        #self.frame = cv2.rotate(self.frame, cv2.ROTATE_180)
        self.frame = cv2.resize(self.frame,(640,640)) #changed the size//jamae//
        if self.camtype == 'raspi':
            self.frame = self.undistort(self.frame)
        return self.frame
    
    
    def detectObject(self):
        area, center = self.objectDetector.detect(img=self.canvas)
        self.canvas = self.objectDetector.getCanvas()
        if area is not None:
            return area, center
        else:
            return self.prevMarker
        
    
    def checkMarkerHistory(self, marker):
        if self.prevMarker != marker: #uav is in frame
            self. prevMarker = marker
            self.lastSeenMarker = marker
            self.waitTimes['waiting'].clear()
        else: # uav is outside frame
            if self.isElapsedWaitTime(status='waiting'):
                self.prevMarker = None
                self.waitTimes['waiting'].clear()
            else:
                self.prevMarker = marker
                self.lastSeenMarker = marker
        return self.lastSeenMarker

    
    def isElapsedWaitTime(self, status):
        now = time.time()
        self.waitTimes[status].append(now)
        elapsed = now - self.waitTimes[status][0]
        return elapsed > self.waitTimes['wait_time']
    

    def showFrame(self):
        #while self.vehicleIp.channels['8'] == 1495:
        #    continue
        canvas = self.canvas.copy()
        cv2.imshow('Marker', canvas)
        cv2.waitKey()
    

    def closeDisplay(self):
        cv2.destroyAllWindows()


    def getImageCenter(self, marker):
        h, w = self.frame.shape[:2]
        cx, cy = (int(w/2), int(h/2))
        return cx, cy
    

    def getForwardTime(self, marker, imageCenter):
        (area, heightPixel, widthPixel), (markerCenterX, markerCenterY) = marker  #tr br added by John Mel
        imageCenterX, imageCenterY = imageCenter
        #heightCm, widthCm = 18, 18  #static code based on actual marker dimension
        #heightCm, widthCm = 29, 29 #small drone (kang Jam)
        #heightCm, widthCm = 3.5, 3.5
        # heightCm, widthCm = 44, 39 #size of the ugv
        heightCm, widthCm = 22, 19 #of the ugv #half lang sa UGV
        # TODO: possible problematic conversion
        xyDistance = ((markerCenterX-imageCenterX)**2 + (markerCenterY-imageCenterY)**2)**(1/2) #Euclidean Distance between the UGV and the Red Marker based on a 2D space (x-coordinates)
        xyDistanceToCm = ((heightCm)/((heightPixel + widthPixel)/2)) * xyDistance
        
        yDistance = abs(imageCenterY - markerCenterY)
        yDistanceToCm = ((heightCm)/((heightPixel + widthPixel)/2)) * yDistance
        #x_distance_to_cm = 29.85 - 0.0213*area + 0.03182*abs(markerCenterX - imageCenterX) + 0.01205*abs(markerCenterY - imageCenterY) + 0.2606*(x_distance)
        #return (0.0317*x_distance_to_cm) + 0.4673
        #return xyDistanceToCm, yDistanceToCm
        return xyDistanceToCm, yDistanceToCm
    

    def getObjectAngle(self, marker, imageCenter):
        (area, height, width), (markerCenterX, markerCenterY) = marker    #tr br added by John Mel
        imageCenterX, imageCenterY = imageCenter
        computedAngle = 0 if imageCenterX==markerCenterX else math.degrees(math.atan((imageCenterY-markerCenterY)/(imageCenterX-markerCenterX)))
        if markerCenterX > imageCenterX: # marker on the positive x-quadrants
            objectAngle = computedAngle
        else:   # marker on the negative x-quadrants
            objectAngle = 180 + computedAngle

        if imageCenterX == markerCenterX:
            if markerCenterY < imageCenterY:
                objectAngle = 270
            else:
                objectAngle = 90
        
        # return (objectAngle+360) % 360
    
        if objectAngle < 0 or objectAngle == 0:
            objectAngle = objectAngle*-1
        else:
            objectAngle = abs(objectAngle-270) + 90

        return (objectAngle+360) % 360


    def undistort(self, frame):
        K = np.array([[571.99144659, 0.0, 389.37106708], [0.0, 578.12247901, 200.85252072], [0.0, 0.0, 1.0]])
        D = np.array([[-0.45318365], [0.24363718], [0.00400835], [0.01058107]]) #, [-0.07743168]
        h, w = frame.shape[:2]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, (w,  h), cv2.CV_16SC2)
        return cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


    # enhance detection with tracking
    def track(self, img, scope=2.5):
        if self.prevMarker is None:
            return img
        else:
            h,w = img.shape[:2]
            black = np.zeros((img.shape), np.uint8)
            (area, height, width), (markerCenterX, markerCenterY) = self.prevMarker     # tr br added by John Mel
            side = math.sqrt(area)
            topLeftX, topLeftY = int(max(0, markerCenterX - side*scope)), int(max(0, markerCenterY - side*scope))
            botRightX, botRightY = int(min(w, markerCenterX + side*scope)), int(min(h, markerCenterY, side*scope))
            black[topLeftY: botRightY, topLeftX: botRightX] = img[topLeftY: botRightY, topLeftX: botRightX]
            return black
        

if __name__ == '__main__':


    ip = ImageProcessing()
    ip.publish()

    ip.showFrame()

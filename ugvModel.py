import cv2
import pandas as pd
import numpy as np
from ultralytics import YOLO

class Model:

    #miscellaneous attribute
    canvas = None

    def __init__(self):
        self.model = YOLO("/home/jetson/multiprocessing_final/train9/weights/best.pt")
        # self.model = YOLO("/home/jetson/multiprocessing_final/train4/best.pt")


    def detect(self, img, detectedObject=1, confidence=0.8): # set to 0.96 or anything higher within this value if necessary
        # frame = param['videoFrame'].value
        self.canvas = img.copy()                             
                                                                      
        results = self.model.predict(self.canvas.copy(), verbose = False)
        img = results[0].plot()


        if (len(results[0]) == detectedObject) and (results[0][0].boxes.conf >= confidence):    #Runs only if object detected is one and confidence is high
            
            center = self.getCenter(results[0][0])
            area = self.getAreaMask(results[0][0])

            #area = self.getAreaBox(self.getDimensions(results[0][0]))
            #topCornerBox = self.getTopCornerBox(results[0][0])
            topCornerMask = self.getTopCornerMask(results[0][0])
            dimensions = self.getDimensions(results[0][0])
            img = cv2.circle(img, (center), 6, (255,0,0), -1)
            cv2.line(img,(topCornerMask),(center), (255,255,255), 3 )
            self.canvas = img.copy()
            # print("Area, Center: ", (area, center))
            return area, center

        return None, None

    
    def getCanvas(self):
        return self.canvas
    
    def getDimensions(self, results):
        return int(results.boxes.xywh[0][2]), int(results.boxes.xywh[0][3]) #width and height
    
    def getAreaMask(self, results): 
        topCornerX, topCornerY = self.getTopCornerMask(results)
        centerX, centerY = int(results.boxes.xywh[0][0]), int(results.boxes.xywh[0][1])
        area = int(results.boxes.xywh[0][2])*int(results.boxes.xywh[0][3])

        #Leg Length in Pixels (Substitute for Height and Width Yield)
        legLength = ((topCornerX -centerX)**2 + (topCornerY-centerY) **2  ) ** (1/2) #Euclidean Distance

        

        return area, int(legLength), int(legLength)
    
    def getCenter(self, results):
        return (int(results.boxes.xywh[0][0]), int(results.boxes.xywh[0][1]))  #xywh format is x center, y center, width, height
    
    def getAreaBox(self, dim): #the bounding box only
        w, h = dim
        return h*w, h, w
    
    def getTopCornerMask(self, results):
        return int(results.masks.xy[0][0][0]), int(results.masks.xy[0][0][1])
    
    def getTopCornerBox(self, results): #of the box
        return int(results.boxes.xyxy[0][0]), int(results.boxes.xyxy[0][1])
      

if __name__ == '__main__':
    print("passed")
    o = Model()
    # path = "/home/jetson/multiprocessing_final/test_for_mask.MP4"
    #img = cv2.imread(path)  #for images only
    cap = cv2.VideoCapture(0)
    #cap = cv2.VideoCapture(0, cv2.CAP_V4L2) #change 0 to the video file if not livestream
    ret, img = cap.read()
    while ret:
        img = cv2.resize(img,(640,640))
        o.detect(img)
        frame = o.getCanvas()
        cv2.imshow('Frame', frame)
        ret, img = cap.read()
        k = cv2.waitKey(30) & 0xff
        if k == 27:
            ret = False
            break

    cap.release()
    cv2.destroyAllWindows()





# import cv2 as cv
# import numpy as np
# import math
# from ultralytics import YOLO
# from pathlib import Path

# # Load YOLO model
# m = YOLO('D:/Final Trained Model/ugv2_yolov8/ultralytics/runs/segment/train9/weights/best.pt')


# class Model:

#     def detection(self,param):
#         frame = param['videoFrame'].value
#         # Resize the frame
#         width = 800  # specify the desired width
#         height = 700  # specify the desired height
#         frame = cv.resize(frame, (width, height))

#         # Predict objects in the frame using YOLO
#         res = m.predict(frame, conf=0.7)

#         # Define the center coordinates of the frame
#         frame_center_x = width // 2
#         frame_center_y = height // 2

#         # Draw a circle at the center of the frame
#         cv.circle(frame, (frame_center_x, frame_center_y), 5, (0, 0, 255), -1)

#         # Iterate detection results
#         for r in res:
#             img = np.copy(r.orig_img)
#             img_name = Path(r.path).stem

#             # Iterate each object contour
#             for ci, c in enumerate(r):
#                 label = c.names[c.boxes.cls.tolist().pop()]

#                 b_mask = np.zeros(img.shape[:2], np.uint8)

#                 # Create contour mask
#                 contour = c.masks.xy[ci].astype(np.int32).reshape(-1, 1, 2)
#                 # print(contour)
#                 _ = cv.drawContours(b_mask, [contour], -1, (255, 255, 255), cv.FILLED)

#                 # Isolate object with black background
#                 mask3ch = cv.cvtColor(b_mask, cv.COLOR_GRAY2BGR)
#                 masked = cv.bitwise_and(mask3ch, img)
                

#                 # Calculate center coordinates of the contour within masked object
#                 M = cv.moments(contour)
#                 if M["m00"] != 0:
#                     cX = int(M["m10"] / M["m00"])
#                     cY = int(M["m01"] / M["m00"])

#                     # Calculate the Euclidean distance between the frame center and the contour center
#                     distance = math.sqrt((cX - frame_center_x) ** 2 + (cY - frame_center_y) ** 2)
#                     print("Center Frame to UGV Frame {}: {:.2f}".format(label, distance))

#                     # Draw a red circle at the center coordinates on the masked object
#                     cv.circle(masked, (cX, cY), 3, (0, 255, 0), -1)

#                     # Draw a line connecting the center of the frame to the center of the object
#                     cv.line(frame, (frame_center_x, frame_center_y), (cX, cY), (255, 0, 0), 2)

#                     # Add label for the center of the object
#                     cv.putText(frame, f'{label} Center: ({cX}, {cY})', (cX + 5, cY + 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2, cv.LINE_AA)
#                 else:
#                     # If the moment is 0, set center coordinates to None
#                     cX, cY = None, None

#                 # # Display the masked object
#                 # cv.imshow('Masked Object - {}'.format(label), masked)

#         # Display the original frame
#         # print(frame)
#         cv.imshow('Original Frame', frame)
#         cv.waitKey(1)



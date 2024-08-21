'''
    To run multiple programs simultaneously through multi-procesing

    Note:
        - next version will be migrated to ROS env
'''


from multiprocessing import Process, Manager
import time
# from camera import Camera 
# from imageprocessing import ImageProcessing
from ip import ImageProcessing
from turtleviz import TurtleVisualization
#from rover import Rover
from drone import Drone


rotationStatus = False
previousTime = 0
lastError = 0
cumError = 0


def runCamera(param):
    cam = Camera()
    cam.toDisplay = False
    cam.publish(param)
    

def closeCamera(param):
    print("Opening Camera.")
    time.sleep(60)
    print("Closing Camera.")
    param['camStatus'].value = False
    
  

def runImageProcessing(param):
    #while param['frame'].value is None:
    #    print("run")
    #    continue # wait while frame is available
    ip = ImageProcessing()
    ip.publish(param)
    ip.showFrame()
    # while(param['camStatus'].value):
    #     ip.publish(param)
    #     ip.showFrame()
    # ip.closeDisplay()




def runVisualization(param):
    print("visualize")
    while param['forwardTime'].value is None and param['objectAngle'].value is None:
        continue # wait while time and angle are available
    viz = TurtleVisualization()
    while(param['camStatus'].value):
        viz.forwardTime = param['forwardTime'].value
        viz.objectAngle = param['objectAngle'].value
        viz.marker = param['marker'].value
        viz.imageCenter = param['imageCenter'].value
        viz.changeScreen()
        viz.drawMarker()
        viz.rotateBot()


def runRover(param):
    while param['forwardTime'].value is None and param['objectAngle'].value is None:
        continue # wait while time and angle are available
    rover = Rover()
    while(param['camStatus'].value):
        rover.objectAngle = param['objectAngle'].value
        rover.marker = param['marker'].value
        rover.imageCenter = param['imageCenter'].value
        rover.rotate()
        #print(rover.objectAngle)
        #rover.rotate() #my add
        #time.sleep(1)
        #rover.forward()


def runDrone(param):

    while param['forwardTime'].value is None and param['objectAngle'].value is None:
        # time.sleep(5)
        continue # wait while time and angle are available
    drone = Drone()
    #print(param['objectAngle'].value)
    while(param['camStatus'].value):

        #if drone.vehicle.mode == "GUIDED":
        #    drone.turnToRTL()

        print("Waiting to start!")
        #if drone.vehicle.channels['7'] != 1495: #activate for automatic visual servoing
        if drone.vehicle.mode == 'GUIDED':    
            #if drone.vehicle.mode!= "GUIDED":
            print("Going Guided Mode!")
            #drone.turnToGuided()
            print("Now on Guided mode. Waiting for 5 seconds")
            time.sleep(5)
            
            drone.forwardTime = param['forwardTime'].value #distance
            drone.objectAngle = param['objectAngle'].value
            drone.marker = param['marker'].value
            drone.imageCenter = param['imageCenter'].value
            
            drone.stopMovementCommandVelocity()
            
            drone.forwardTime = param['forwardTime'].value
            drone.objectAngle = param['objectAngle'].value
            drone.marker = param['marker'].value
            drone.imageCenter = param['imageCenter'].value
            
            print("Rotate!")
            drone.rotate()
            time.sleep(7)
            
            drone.forwardTime = param['forwardTime'].value
            drone.objectAngle = param['objectAngle'].value
            drone.marker = param['marker'].value
            drone.imageCenter = param['imageCenter'].value
            
            
            if drone.objectAngle >= 60 and drone.objectAngle <= 120 and drone.forwardTime != 0:
                actualDistance, linearDistance = drone.forwardTime
                print("Linear Distance is: ", linearDistance)
                while linearDistance >= 65 :
                    print(linearDistance)
                    drone.forwardTime = param['forwardTime'].value
                    if drone.forwardTime == 0: drone.forwardTime =0,0
                    actualDistance, linearDistance = drone.forwardTime
                    PID = computePID(linearDistance)
                    print("Drone forward! Linear Distance: ", linearDistance, " PID: ", PID)
                    
                    drone.sendMovementCommandVelocity(PID, 0, 0)
                    time.sleep(1)
                    drone.sendMovementCommandVelocity(0,0,0)
                
                
                #drone.forwardRover()
                #time.sleep(5)
            else:
                drone.rotate()
                time.sleep(7)
        else:
            drone.forwardTime = param['forwardTime'].value
            drone.objectAngle = param['objectAngle'].value
            drone.marker = param['marker'].value
            drone.imageCenter = param['imageCenter'].value
            
        
            #print(drone.objectAngle)

            
def computePID(input, setPoint=5, kp=0.003, ki=0, kd=0):
    global previousTime
    global lastError
    global cumError
    
    currentTime = time.time()
    elapsedTime = (currentTime - previousTime)
    previousTime = currentTime
    error = setPoint - input
    
    cumError += error * elapsedTime
    
    rateError = (error - lastError) / elapsedTime
    out = (kp* error + ki * cumError + kd*rateError) * -1
    lastError = error
    
    if input < 0.25: out = 0
    
    return out
            

if __name__ == '__main__':

    param = {}

    param['camStatus'] = Manager().Value('camStatus', False)
    param['frame'] = Manager().Value('frame', None)
    param['forwardTime'] = Manager().Value('forwardTime', None)
    param['objectAngle'] = Manager().Value('objectAngle', None)
    param['marker'] = Manager().Value('marker', None)
    param['imageCenter'] = Manager().Value('imageCenter', None)


    p = [
        # Process(target=runCamera, args=(param,)),
        Process(target=runImageProcessing, args=(param,)),
        # Process(target=runVisualization, args=(param,)),
        #Process(target=runRover, args=(param,)),
        Process(target=runDrone, args=(param,)),
     #   Process(target=closeCamera, args=(param,)),
    ]

    
    for process in p:
        process.start()

    for process in p:
        process.join()

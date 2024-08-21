import os
os.system('sudo chmod 666 /dev/ttyTHS1')

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time




class Drone:
    # input attribute
    forwardTime = None #added by John Mel
    objectAngle = None
    marker = None
    imageCenter = None

    #miscellaneous attributes
    #vehicle = None
    headingSensor = -1.0
    thresholdAngle = 10
    destinationReached = False

    def __init__(self):
        self.initVehicle()
        #print("Vehicle Connected by ", self.vehicle)
        #self.armVehicle() #arm vehicle and change vehicle to guided mode
        #pass


    def initVehicle(self):
         self.vehicle = connect('/dev/ttyTHS1', baud=921600, wait_ready=True)
         print("Vehicle is connected.")


    def armVehicle(self): #Pre arm safety checks if drone is ready to takeoff.
        print('Basic pre-arm checks')
        #Don't let the user try to arm until autopilot is ready
        #while not self.vehicle.is_armable:
         #   print("Waiting for vehicle to initialize")
          #  time.sleep(1)

        print("Arming motors")
        #Copter should arm in GUIDED mode
        #self.vehicle.mode = VehicleMode("GUIDED")
        #self.vehicle.armed = True

        #while not self.vehicle.armed:
        #    print("Waiting for arming...")
        #    self.vehicle.armed = True
        #    time.sleep(1)


    def heading(self):
        degree = 360.0
        return degree

    def adjustDegreeTurn(self): #converts the object angle to 0 to 360 (clockwise)
        if self.objectAngle <= 90:
            degree = 90 - self.objectAngle
        elif self.objectAngle <= 180 and self.objectAngle > 90:
            degree = 360 - (self.objectAngle - 90)
        elif self.objectAngle <= 270 and self.objectAngle > 180:
            degree = 270 - (self.objectAngle - 180)
        elif self.objectAngle <= 360 and self.objectAngle > 270:
            degree = 90 + (360 - self.objectAngle)

        return degree


    def rotate(self, angle=None):
        if self.forwardTime == 0:
            self.forwardTime = 0,0
        actualDistance, linearDistance = self.forwardTime
        if angle is None:
            self.objectAngle = self.adjustDegreeTurn()
            degree = self.objectAngle
        else:
            degree = angle - self.heading()
        degree = (degree + 360) % 360
        print("Rotation Degree:", degree)  # Print the degree value
        # print(degree)
        
        
        if actualDistance is None:
            actualDistance = 0

        if actualDistance < 65 and actualDistance is not None:
            degree = 0

        if degree < self.thresholdAngle:
            self.sendMovementCommandYaw(0)  #No rotation
        else:
            self.sendMovementCommandYaw(degree)


    def forward(self):
        actualDistance, linearDistance = self.forwardTime
        
        if actualDistance >= 5:
            self.destinationReached = False
        else:
            self.destinationReached = True
            
            
    def forwardRover(self):     #wala nani
        actualDistance = self.forwardTime
        
        if self.marker is not None:
            (area, height, width), (mx, my) = self.marker
            ix,iy = self.imageCenter
            input = abs(my-iy)
        else:
            input = 0
            
            
        
        if actualDistance >= 10:
            self.destinationReached = False
        else:
            self.distinationReached = True
            
        if self.destinationReached == False:
            kp = 0.015 #0.0191
            seconds = input*kp + 0.3 #0.4252
            if seconds > 1:
                seconds = 1
                
            if actualDistance < 10:
                speed = 0
                self.destinationReached = True
            else: speed = 1
            
        #     self.sendMovementCommandVelocity(0.8,0,0)
        #     time.sleep(2)
        #     self.stopMovementCommandVelocity()
            
        # else:
        #     self.stopMovementCommandVelocity()


    def sendMovementCommandYaw(self, heading):
        speed = 0
        direction = 1 #direction -1 ccw, 1 cw
        print("Rotating")

        if heading > 180: #rotateLeft
            heading = abs(heading-360)
            direction = -1

        msg = self.vehicle.message_factory.command_long_encode(
            0,0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            speed,  #speed deg/s
            direction,
            1,      #relative offset 1
            0,0,0
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    
    def sendMovementCommandVelocity(self, Vx, Vy, Vz):
        print("Forward in!", Vx)
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0,0,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            0b0000111111000111, #BITMASK > Consider only the velocities
            0,0,0,  #--Position
            Vx, Vy, Vz, #--Velocity m/s
            0,0,0,  #--Acceleration
            0,0
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()


    def stopMovementCommandVelocity(self):
        self.sendMovementCommandVelocity(0,0,0)
        
        
    def stopMovementCommandYaw(self):
        self.sendMovementCommandYaw(0)
        
        
    def sendMovementCommandTakeOff(self, targetAltitude):
        self.vehicle.simple_takeoff(targetAltitude)

        while True:
            print("Altitude", self.vehicle.location.global_relative_frame.alt)
            if self.vehicle.location.global_relative_frame.alt >= targetAltitude*0.95:
                print("Reached target altitude!")
                break
            time.sleep(1)
         
         
    def turnToLand(self):
        self.vehicle.mode = VehicleMode("LAND")
    
    def turnToGuided(self):
        self.vehicle.mode = VehicleMode("GUIDED")

    def turnToRTL(self):
        self.vehicle.mode = VehicleMode("RTL")

    def print(self):
        print(self.vehicle.channels['7'])

if __name__ == "__main__":
    drone = Drone()

    while True:
        drone.print()


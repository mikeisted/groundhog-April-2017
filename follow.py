# import the necessary packages
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import numpy as np
from PiVideoStream import PiVideoStream
from imutils.video import FPS
from picamera.array import PiRGBArray
from picamera import PiCamera
import argparse
import imutils
import cv2
import sys
from datetime import datetime, timedelta


#--------------------------SEND VELOCITY VECTORS IN NED FRAME--------------------
# vx is north. vy is east. vz is down.  This is unrelated to the yaw of the UAV.
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    vehicle.send_mavlink(msg)



#--------------------------SEND YAW IN LOCAL NED FRAME--------------------
# This sets the absolute yaw, in degrees, measured clockwise from North.
# Heading in range 0-360 degrees.  No negs!
def condition_yaw(heading, clockwise, relative=False): # In degrees
    if relative:
        is_relative=1 #yaw relative to direction of travel
    else:
        is_relative=0 #yaw is an absolute angle
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        clockwise,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)

#--------------------------VERTICAL ANGLE FROM VERTICAL IMAGE OFFSET--------------------
# Returns the angle (radians) in the vertical plane between a target and the centreline in the
# camera image. 

def vert_image_angle (y):
    # print y
    vertical_angle_of_view = 48.8 # degrees
    vertical_resolution = 240 # pixels
    pix_per_degree = vertical_resolution / vertical_angle_of_view

    vertical_angle = (y / pix_per_degree)
    # print vertical_angle
    vertical_angle = np.radians(vertical_angle)
    

    return (vertical_angle)

#--------------------------HORIZONTAL ANGLE FROM HORIZONTAL IMAGE OFFSET----------------
# Returns the angle (radians) in the horizontal plane between a target and the centreline in the
# camera image. x is the x coordinate of the target, origin top left of frame.

def horiz_image_angle (x):
    horiz_angle_of_view = 62.2 # degrees
    horiz_resolution = 320 # pixels
    pix_per_degree = horiz_resolution / horiz_angle_of_view
    
    horiz_angle = np.radians(x / pix_per_degree)

    return (horiz_angle)


#--------------------------GET CAMERA GIMBAL PITCH-------------------------------------
# Returns the pitch (degrees) as measured from the horizontal. Negative is down.

def servo_gimbal_pitch():
    minpwm = 982
    maxpwm = 2006
    #midpwm = (minpwm+maxpwm)/2
    servo_range = 90 #degrees
    pwm_per_degree = (maxpwm-minpwm) / servo_range
    calibrated_pwm = 1568 # The pwm value at a known pitch
    calibrated_pitch = 0 # Nose down is negative pitch

    current_pwm = vehicle.channels['6']
    delta_pwm = calibrated_pwm - current_pwm # Neg delta is down.
    delta_pitch = delta_pwm / pwm_per_degree 
    gimbal_pitch = calibrated_pitch + delta_pitch

    return (gimbal_pitch)

#--------------------------GET CAMERA GIMBAL PAN-------------------------------------
# Returns the pan (degrees) as measured from the centreline.  Positive is clockwise.

def servo_gimbal_pan():
    minpwm = 982
    maxpwm = 2006
    #midpwm = (minpwm+maxpwm)/2
    servo_range = 90 #degrees
    pwm_per_degree = (maxpwm-minpwm) / servo_range
    calibrated_pwm = 1427 # The pwm value at a known pan
    calibrated_pan = 0 # Positive pan (higher pwm) is clockwise

    current_pwm = vehicle.channels['8']
    delta_pwm = current_pwm - calibrated_pwm 
    delta_pan = delta_pwm / pwm_per_degree
    gimbal_pan = calibrated_pan + delta_pan

    return (gimbal_pan)


#--------------------------GET BEARING A to B-------------------------------------
# Returns the pan (radians) as measured from the centreline.  Positive is clockwise.

def new_bearing(locA, locB):
    na = locA[0]
    ea = locA[1]
    nb = locB[0]
    eb = locB[1]

    #print na, nb, ea, eb
    deltan = nb - na
    deltae = eb - ea

    #alphaAB = np.arctan(deltae/deltan)
    alphaAB = np.arctan2(deltae,deltan)

    #if np.abs(deltan) < 0.01 :
    #    alphaAB = 0
    #else:
    #    alphaAB = np.arctan2(deltan,deltae)

    return (alphaAB)
"""
#--------------------------GET NEW BEARING WITH SIDESLIP -------------------------------------
# Returns the pan (degrees) as measured from the centreline.  Positive is clockwise.

def bearing_with_roll(bearingAB, bearingOA):
    fwd_bias = 10 # The degree to which the emphasis is on going forward, not sideways

    northOA = np.cos(bearingOA)
    northAB = np.cos(bearingAB)
    northRes = northOA + (fwd_bias * northAB)

    eastOA = np.sin(bearingOA)
    eastAB = np.sin(bearingAB)
    eastRes = eastOA + (fwd_bias * eastAB)

    bearingOB = np.arctan2(eastRes,northRes)

    return (bearingOB)
    
"""
#--------------------------SET UP CONNECTION TO VEHICLE----------------------------------

# Parse the arguments  
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to the physical UAV or to the simulator on the network
if not connection_string:
    print ('Connecting to pixhawk.')
    vehicle = connect('/dev/serial0', baud=57600, wait_ready= True)
else:
    print ('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)



#--------------------------SET UP VIDEO THREAD ----------------------------------

# created a *threaded *video stream, allow the camera sensor to warmup,
# and start the FPS counter
print('[INFO] sampling THREADED frames from `picamera` module...')
vs = PiVideoStream().start()
time.sleep(2.0)


#-------------- FUNCTION DEFINITION TO ARM AND TAKE OFF TO GIVEN ALTITUDE ---------------
def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print ('Basic pre-arm checks')
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ('Waiting for vehicle to initialise...')
        time.sleep(1)

        
    print ('Arming motors')
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print ('Waiting for arming...')
        time.sleep(1)

    print ('Taking off!')
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    while True:
        # print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            break
        time.sleep(1)


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE TRACKING  ---------------------
def tracking (vstate):
    
    print vstate
    
    #The vehicle process images and maintains all data ready to fly autonomously.
    #However, it does not send velocity vectors unless GUIDED flight mode is selected.

    # Open a local file to append location of target data.
    #f = open("locationdata.txt","a")

    red1Good = red2Good = False # Set True when returned target offset is reliable.
    target = None # Initialise tuple returned from video stream

    # Initialise the FPS counter.
    #fps = FPS().start()

    while vstate == "tracking":

     
        # grab the frame from the threaded video stream and return coords A and B wrt centrelines
        target = vs.read()
        coordA = target[0]
        coordA_Good = target[1]
        coordB = target[2]
        coordB_Good = target[3]

        #condition_yaw(np.degrees(0), 1) # clockwise

        # Get height
        height = vehicle.location.global_relative_frame.alt  * -1
        #height = -0.3 #vehicle.location.local_frame.down * -1
        #north = vehicle.location.local_frame.north
        #east = vehicle.location.local_frame.east
        #print height, north, east

        #pitch_from_gimbal = np.radians(servo_gimbal_pitch()) # radians from horizontal, down is negative
        pitch_from_gimbal = np.radians(-45) # servo_gimbal_pitch 

        #pan_from_gimbal = np.radians(servo_gimbal_pan()) # Yaw in radians wrt North
        pan_from_gimbal = 0.0
        pan_from_yaw = vehicle.attitude.yaw            

        if coordA_Good == True:


            pitch_from_image = vert_image_angle(coordA[1])
            #print "A: ", pitch_from_image
            total_pitch = pitch_from_gimbal + pitch_from_image # Down is negative, in radians

            pan_from_image = horiz_image_angle(coordA[0])
            total_pan = pan_from_yaw + pan_from_gimbal + pan_from_image

            # print total_pitch,
            #print "A Pitch: ", pitch_from_gimbal, pitch_from_image, total_pitch
            #print "A Pan: ", pan_from_yaw, pan_from_gimbal, pan_from_image, total_pan
            
            mult = height / np.tan(total_pitch)
            coordA_north = mult * np.cos(total_pan)
            coordA_east = mult * np.sin(total_pan)
            locA = (coordA_north, coordA_east) # Relative to current position NED
        else:
            locA = (0,0)
            

        if coordB_Good == True:


            pitch_from_image = vert_image_angle(coordB[1])
            #print "B: ", pitch_from_image
            total_pitch = pitch_from_gimbal + pitch_from_image # Down is negative, in radians

            pan_from_image = horiz_image_angle(coordB[0])
            total_pan = pan_from_yaw + pan_from_gimbal + pan_from_image

            # print total_pitch,
            #print "B Pitch: ", pitch_from_gimbal, pitch_from_image, total_pitch
            #print "B Pan: ", pan_from_yaw, pan_from_gimbal, pan_from_image, total_pan

            
            mult = height / np.tan(total_pitch)
            coordB_north = mult * np.cos(total_pan)
            coordB_east = mult * np.sin(total_pan)
            locB = (coordB_north, coordB_east) # Relative to current position NED
        else:
            locB = (0,0)

        # So we test of we have a full lock with ground locations ready to calculate bearings.
        if coordA_Good == True and coordB_Good == True:

            #locdata = (locA)
            #f.write(str(locdata) + '\n')
            #print locA,locB
            # time.sleep(0.5)
            
            vmax = 1.0
            fwd = 3 # The larger this value, the smaller the sideslip.
            v = vmax/(1+(1/fwd))
            
            bearingAB = (new_bearing(locA,locB))
            
            bearingOA = np.arctan2(locA[1], locA[0])
            #print np.degrees(bearingAB), np.degrees(bearingOA)
            
            vn = v * (np.cos(bearingAB) + (np.cos(bearingOA)/fwd))
            ve = v * (np.sin(bearingAB)  + (np.sin(bearingOA)/fwd))


            # Check if operator has transferred to autopilot using TX switch.
            if vehicle.mode == "GUIDED":
                send_ned_velocity (vn,ve,0)
      
                if (np.degrees(bearingAB-vehicle.attitude.yaw)) > 2.0:
                    if bearingAB < 0:
                        bearingAB = bearingAB + (2*np.pi)
                    condition_yaw(np.degrees(bearingAB), 1) # clockwise
                    
                elif (np.degrees(bearingAB-vehicle.attitude.yaw)) < -2.0:
                    if bearingAB < 0:
                        bearingAB = bearingAB + (2*np.pi)
                    condition_yaw(np.degrees(bearingAB), -1) # anticlockwise

        else:
            vstate = "lost"
            break
                    
                
        # update the FPS counter
        #fps.update()



    # stop the timer and display FPS information
    #fps.stop()
    #print('Elasped time in tracking state: {:.2f}'.format(fps.elapsed()))
    #print('Approx. FPS: {:.2f}'.format(fps.fps()))

    #f.close()
    return vstate


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE SEARCHING  ---------------------
def searching (vstate):
    
    print vstate
    
    #The vehicle process images and maintains all data ready to fly in the following state.
    #However, it does not send attitude messages as the vehicle is still under manual control.

    red1Good = red2Good = False # Set True when returned target offset is reliable.
    target = None # Initialise tuple returned from video stream

    # Initialise the FPS counter.
    #fps = FPS().start()

    while vstate == "searching":

     
        # grab the frame from the threaded video stream and return coords A and B wrt centrelines
        target = vs.read()
        coordA = target[0]
        coordA_Good = target[1]
        coordB = target[2]
        coordB_Good = target[3]

        #condition_yaw(np.degrees(0), 1) # clockwise

        # Get height
        height = vehicle.location.global_relative_frame.alt  * -1
        # height = -0.35 #vehicle.location.local_frame.down * -1

        #pitch_from_gimbal = np.radians(servo_gimbal_pitch()) # radians from horizontal, down is negative
        pitch_from_gimbal = np.radians(-45) # servo_gimbal_pitch 

        #pan_from_gimbal = np.radians(servo_gimbal_pan()) # Yaw in radians wrt North
        pan_from_gimbal = 0.0
        pan_from_yaw = vehicle.attitude.yaw

        if coordA_Good == True and coordB_Good == True:
            vstate = "tracking"
            break
        

        elif coordA_Good == True:


            pitch_from_image = vert_image_angle(coordA[1])
            #print "A: ", pitch_from_image
            total_pitch = pitch_from_gimbal + pitch_from_image # Down is negative, in radians

            pan_from_image = horiz_image_angle(coordA[0])
            total_pan = pan_from_yaw + pan_from_gimbal + pan_from_image

            # print total_pitch,
            #print "A Pitch: ", pitch_from_gimbal, pitch_from_image, total_pitch
            #print "A Pan: ", pan_from_yaw, pan_from_gimbal, pan_from_image, total_pan
            
            mult = height / np.tan(total_pitch)
            coordA_north = mult * np.cos(total_pan)
            coordA_east = mult * np.sin(total_pan)
            locA = (coordA_north, coordA_east) # Relative to current position NED

            locB = locA
            locA = (0,0)
            
            

        elif coordB_Good == True:


            pitch_from_image = vert_image_angle(coordB[1])
            #print "B: ", pitch_from_image
            total_pitch = pitch_from_gimbal + pitch_from_image # Down is negative, in radians

            pan_from_image = horiz_image_angle(coordB[0])
            total_pan = pan_from_yaw + pan_from_gimbal + pan_from_image

            # print total_pitch,
            #print "B Pitch: ", pitch_from_gimbal, pitch_from_image, total_pitch
            #print "B Pan: ", pan_from_yaw, pan_from_gimbal, pan_from_image, total_pan

            
            mult = height / np.tan(total_pitch)
            coordB_north = mult * np.cos(total_pan)
            coordB_east = mult * np.sin(total_pan)
            locB = (coordB_north, coordB_east) # Relative to current position NED
            locA = (0,0)
            #print "BGood: ", locA, locB

        else:
            vstate = "lost"
            break
        
        
             
        vmax = 1.0
        fwd = 3
        v = vmax/(1+(1/fwd))
        
        bearingAB = (new_bearing(locA,locB))
        
        bearingOA = np.arctan2(locA[1], locA[0])
        # print np.degrees(bearingAB), np.degrees(bearingOA)
        
        vn = v * (np.cos(bearingAB) + (np.cos(bearingOA)/fwd))
        ve = v * (np.sin(bearingAB)  + (np.sin(bearingOA)/fwd))


        # Check if operator has transferred to autopilot using TX switch.
        if vehicle.mode == "GUIDED":
            send_ned_velocity (vn,ve,0)
  
            if (np.degrees(bearingAB-vehicle.attitude.yaw)) > 2.0:
                if bearingAB < 0:
                    bearingAB = bearingAB + (2*np.pi)
                condition_yaw(np.degrees(bearingAB), 1) # clockwise
                
            elif (np.degrees(bearingAB-vehicle.attitude.yaw)) < -2.0:
                if bearingAB < 0:
                    bearingAB = bearingAB + (2*np.pi)
                condition_yaw(np.degrees(bearingAB), -1) # anticlockwise
                    
                
        # update the FPS counter
        #fps.update()



    # stop the timer and display FPS information
    #fps.stop()
    #print('Elasped time in tracking state: {:.2f}'.format(fps.elapsed()))
    #print('Approx. FPS: {:.2f}'.format(fps.fps()))


    return vstate


#-------------- FUNCTION DEFINITION TO FLY IN VEHICLE STATE LOST  ---------------------
def lost (vstate):
    
    print vstate
    vstate = "lost"

    
    #The vehicle process images and maintains all data ready to fly in the following state.
    #However, it does not send attitude messages as the vehicle is still under manual control.

    red1Good = red2Good = False # Set True when returned target offset is reliable.
    target = None # Initialise tuple returned from video stream

    # Initialise the FPS counter.
    #fps = FPS().start()

    while vstate == "lost":

     
        # grab the frame from the threaded video stream and return coords A and B wrt centrelines
        target = vs.read()
        coordA = target[0]
        coordA_Good = target[1]
        coordB = target[2]
        coordB_Good = target[3]

        if coordA_Good == True or coordB_Good == True:
            vstate = "searching"
            break

        else:        

            # Check if operator has transferred to autopilot using TX switch.
            if vehicle.mode == "GUIDED":
                send_ned_velocity (0,0,0)

                condition_yaw(2, 1, True)
                # print np.degrees(vehicle.attitude.yaw)
                time.sleep(0.2)      
                
        # update the FPS counter
        #fps.update()


    # stop the timer and display FPS information
    #fps.stop()
    #print('Elasped time in tracking state: {:.2f}'.format(fps.elapsed()))
    #print('Approx. FPS: {:.2f}'.format(fps.fps()))

    return vstate





# MAIN PROGRAM

vstate = "tracking" # Set the vehicle state to tracking in the finite state machine.


# If on simulator, arm and take off.
if connection_string:

    print ('Basic pre-arm checks')
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print ('Waiting for vehicle to initialise...')
        time.sleep(1)

    print ('Arming motors')
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True    


    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:      
        print ('Waiting for arming...')
        time.sleep(1)

    # Get airborne and hover
    arm_and_takeoff(10)
    print "Reached target altitude - currently in Guided mode on altitude hold"
    
# This needs to be commented out prior to real flight!
# vehicle.mode = VehicleMode("GUIDED")


while True :

    if vstate == "tracking":
        # Enter tracking state
        vstate = tracking(vstate)

    elif vstate == "searching":
        # Enter searching state
        vstate = searching(vstate)

    else:
        # Enter lost state
        vstate = lost(vstate)


    
"""

#---------------------------- RETURN TO HOME AND CLEAN UP ----------------------------


# Initiate return to home
print "Returning to Launch"
vehicle.mode = VehicleMode("RTL")
print "Pause for 10s before closing vehicle"
time.sleep(10)

"""

#Close vehicle object before exiting script
print "Close vehicle object"
vehicle.close()

# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()

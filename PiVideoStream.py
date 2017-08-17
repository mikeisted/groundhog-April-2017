# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import numpy as np
import imutils
import time
import cv2

#global colour1Confidence
#global colour1Target
#colour1Confidence = [0,0,0,0,0]
#colour1Target = [0,0,0,0,0]
#colour1Good = False

global red1Confidence
global gradRed
red1Confidence = [0,0,0,0,0]
gradRed = [0,0,0,0,0]
red1Good = False

global red2Confidence
global intRed
red2Confidence = [0,0,0,0,0]
intRed = [0,0,0,0,0]
red2Good = False

global red1lConfidence
global gradlRed
red1lConfidence = [0,0,0,0,0]
gradlRed = [0,0,0,0,0]
red1lGood = False

global red2lConfidence
global intlRed
red2lConfidence = [0,0,0,0,0]
intlRed = [0,0,0,0,0]
red2lGood = False

global framecount
framecount = 10


def preparemask (hsv, lower, upper):
    mask = cv2.inRange(hsv, lower, upper);
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    return mask;

def meldmask (mask_0, mask_1):
    mask = cv2.bitwise_or(mask_0, mask_1)
    return mask;


class PiVideoStream:
        
         
	def __init__(self, resolution=(320, 240), framerate=32):
               
		# initialize the camera and stream
		self.camera = PiCamera()
		self.camera.resolution = resolution
		self.camera.framerate = framerate
		self.rawCapture = PiRGBArray(self.camera, size=resolution)
		self.stream = self.camera.capture_continuous(self.rawCapture,
			format="bgr", use_video_port=True)

		# initialize the frame and the variable used to indicate
		# if the thread should be stopped
		self.frame = None
		self.stopped = False


	def start(self):
                
		# start the thread to read frames from the video stream
		Thread(target=self.update, args=()).start()
		print("Thread starting")
		return self

	def update(self):
		# keep looping infinitely until the thread is stopped
		for f in self.stream:
			# grab the frame from the stream and clear the stream in
			# preparation for the next frame
			self.frame = f.array
			self.rawCapture.truncate(0)

			# if the thread indicator variable is set, stop the thread
			# and resource camera resources
			if self.stopped:
				self.stream.close()
				self.rawCapture.close()
				self.camera.close()
				return

	def read(self):

                global red1lConfidence
                global gradRed
                global red2Confidence
                global intRed
                global framecount

                framecount = framecount + 1
                if framecount > 99:
                    framecount  = 10

                # Set the image resolution.
                xres = 320
                yres = 240
                xColour1 = xRed = 0.0

                # Initialise confidence to indicate the line has not been located with the current frame
                coordA_Good = False
                red1Good = False
                coordB_Good = False
                red2Good = False

                # Initialise variables for line calculations
                coordA = [0,0]
                coordB = [0,0]
                xRed1 = xRed2 = 0
                yRed1 = yRed2 = 0
                intc = m = 0.0
                dx = dy = 0

                bearing = offset = 0
                            

                # return the frame most recently read
                frame = self.frame
                #frame = cv2.flip(frame,-1)
                #blurred = cv2.GaussianBlur(frame, (11, 11), 0)

                #fullhsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                # Set y coords of regions of interest.
                # The upper and lower bounds
                roidepth = 20    # vertical depth of regions of interest
                roiymin = 40    # minumum ranging y value for roi origin
                roiymintop = roiymin - roidepth
                roiymax = yres - roidepth -1   # maximum ranging y value for bottom roi origin
                
                # Convert to hsv and define region of interest before further processing.
                #fullhsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                # green = 60
                # blue = 120;
                # yellow = 30;
                #Colour1 = 60

                # Set the sensitivity of the hue
                sensitivity = 20

                # Red is a special case as it sits either side of 0 on the HSV spectrum
                # So we create two masks, one greater than zero and one less than zero
                # Then combine the two.
                lower_red_0 = np.array([0, 100, 100]) 
                upper_red_0 = np.array([sensitivity, 255, 255])
                
                lower_red_1 = np.array([180 - sensitivity, 100, 100]) 
                upper_red_1 = np.array([180, 255, 255])


                # Initialise the bottom roi at the maximum limit
                y3 = roiymax
                y4 = y3 + roidepth

                while y3 > roiymin:

                    # This defines the lower band, looking closer in
                    roihsv2 = frame[y3:y4, 0:(xres-1)]
                    blurred2 = cv2.GaussianBlur(roihsv2, (11, 11), 0)
                    roihsv2 = cv2.cvtColor(blurred2, cv2.COLOR_BGR2HSV)
                

                    # Prepare the masks for the lower roi 
                    maskr_2 = preparemask (roihsv2, lower_red_0 , upper_red_0)
                    maskr_3 = preparemask (roihsv2, lower_red_1 , upper_red_1 )
                    maskr2 = meldmask ( maskr_2, maskr_3)

                    # find contours in the lower roi and initialize the center
                    cnts_red2 = cv2.findContours(maskr2.copy(), cv2.RETR_EXTERNAL,
                            cv2.CHAIN_APPROX_SIMPLE)[-2]
                    center2 = None

                    # Now to find the tracking line in the lower roi
                    # only proceed if at least one contour was found
                    if len(cnts_red2) > 0:
                        
                        # find the largest contour in the mask, then use
                        # it to compute the minimum enclosing circle and
                        # centroid
                        c_red2 = max(cnts_red2, key=cv2.contourArea)
                        ((x_red2, y_red2), radius_red2) = cv2.minEnclosingCircle(c_red2)
                        M_red2 = cv2.moments(c_red2)

                        # compute the center of the contour
                        cx_red2 = int(M_red2["m10"] / M_red2["m00"])
                        cy_red2 = int(M_red2["m01"] / M_red2["m00"])
                        

                        # cy_red is set in the region of interest, so need to adjust for origin in frame
                        cy_red2 = cy_red2 + y3
                        # center = ( cx_red, cy_red )

                        # only proceed if the radius meets a minimum size
                        if radius_red2 > 5:
                            coordA_Good = True
                            # draw the circle and centroid on the frame
                            cv2.circle(frame, (cx_red2, cy_red2), int(radius_red2),
                            (0, 0, 255), 2)                            

                            # calculate offset from centreline
                            xRed2 = cx_red2 - (xres/2) # Contrived so pstve to right of centreline
                            yRed2 = (yres/2) - cy_red2  # Negative values below centreline
                            coordA = [xRed2, yRed2]

                            # The target has been found, so we can break out of the loop here
                            break

                    # But here the target has not been found, we need to move the ROI up
                    y3 = y3 - roidepth
                    y4 = y3 + roidepth

                # And here we have either hit the buffers or found the target.

                
                # So now try for the top roi, working down.                      
                # Initialise the top roi at the very top
                y1 = 0
                y2 = y1 + roidepth

                while y2 < y3: # Go as far as the lower roi but no more.

                    # This defines the upper roi, looking further away
                    roihsv1 = frame[y1:y2, 0:(xres-1)]
                    blurred1 = cv2.GaussianBlur(roihsv1, (11, 11), 0)
                    roihsv1 = cv2.cvtColor(blurred1, cv2.COLOR_BGR2HSV)

                    # Prepare the masks for the top roi 
                    maskr_0 = preparemask (roihsv1, lower_red_0 , upper_red_0)
                    maskr_1 = preparemask (roihsv1, lower_red_1 , upper_red_1 )
                    maskr1 = meldmask ( maskr_0, maskr_1)

                    # find contours in the upper roi and initialize the center
                    cnts_red1 = cv2.findContours(maskr1.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]
                    center1 = None

                    # Now to find the tracking line in the upper roi
                    # only proceed if at least one contour was found
                    if len(cnts_red1) > 0:
                        
                        # find the largest contour in the mask, then use
                        # it to compute the minimum enclosing circle and
                        # centroid
                        c_red1 = max(cnts_red1, key=cv2.contourArea)
                        ((x_red1, y_red1), radius_red1) = cv2.minEnclosingCircle(c_red1)
                        M_red1 = cv2.moments(c_red1)

                        # compute the center of the contour
                        cx_red1 = int(M_red1["m10"] / M_red1["m00"])
                        cy_red1 = int(M_red1["m01"] / M_red1["m00"])
                        

                        # cy_red is set in the region of interest, so need to adjust for origin in frame
                        cy_red1 = cy_red1 + y1
                        # center = ( cx_red, cy_red )

                        # only proceed if the radius meets a minimum size
                        if radius_red1 > 5:
                            coordB_Good = True
                            # draw the circle and centroid on the frame
                            cv2.circle(frame, (cx_red1, cy_red1), int(radius_red1),
                            (0, 0, 255), 2)

                            # calculate offset from centreline
                            xRed1 = cx_red1 - (xres/2) # Contrived so pstve to right of centreline
                            yRed1 = (yres/2) - cy_red1  # Negative values below centreline
                            coordB = [xRed1, yRed1]

                            # The target has been found, so we can break out of the loop here
                            break

                    # But here the target has not been found, we need to move the ROI down
                    y1 = y1 + roidepth
                    y2 = y1 + roidepth

                # And here we have either hit the buffers or found the target.

                

                # Draw Region of interest
                cv2.line(frame, (0, y1), (xres, y1), (255,0,0))
                cv2.line(frame, (0, y2), (xres, y2), (255,0,0))
                cv2.line(frame, (0, y3), (xres, y3), (255,0,0))
                cv2.line(frame, (0, y4), (xres, y4), (255,0,0))

                cv2.imshow('Frame',frame)
                #cv2.imshow('Mask1',maskr1)
                #cv2.imshow('Mask2',maskr2)
                
                
                if framecount%20 == 0:
                    cv2.imwrite('/home/pi/images/image'+ (time.strftime("%H_%M_%S_"))+str(framecount)+'.jpg', frame)
                key = cv2.waitKey(1) & 0xFF

                # CoordA is at the bottom closest to the UAV
                # CoordB is at the top furthest away
                # Coords are in pixels wrt to centre of image
                # print coordA,  coordB
		return (coordA, coordA_Good, coordB, coordB_Good)



	def stop(self):
		# indicate that the thread should be stopped
		self.stopped = True


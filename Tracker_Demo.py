#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   File:       Tracker.py
#   Author:     Colin Lundquist
#   Date:       1/22/2020
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~

#from imutils.video import VideoStream
#from imutils.video import FPS

from imutils.video import WebcamVideoStream

# trailTracer libraries
from trailTracer.servo import ServoDriver
from trailTracer.gpio import IODriver
from trailTracer.fan import FanDriver

import argparse
import imutils
import time
import cv2

CAMERA_BOOTTIME = 0.5

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Fan Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

fan = FanDriver.Fan()    # Create fan object
fan.set(speed=1.0, on=1) # Fan on, speed full

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Load Data
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

video_data = open("./data/video_data","r+") # open video data file
video_num = int(video_data.read())         # read the current video number
video_num += 1                             # increment video_number: next video will be ++
video_data.seek(0,0)
video_data.write(str(video_num))
video_data.close()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Argument Parsing
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

parser = argparse.ArgumentParser(description='trailTrace Tracking demo.')
parser.add_argument('-v', help='Save video to file', action='store_true')
args = parser.parse_args()


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Video Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

print("Beginning PiCam stream...")  # notify user
#video = VideoStream(src=0).start()  # start camera stream
video = WebcamVideoStream(src=0).start()
time.sleep(CAMERA_BOOTTIME)         # allow camera to warm up

if args.v is True: # Create video writer object if video argument is passed
    video_out = cv2.VideoWriter(filename='./Videos/trailTrace_%d.mp4' % video_num, 
            fourcc=cv2.VideoWriter_fourcc('X','2','6','4'), 
            fps=30, 
            frameSize=(640,480))

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Tracker Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

tracker = cv2.TrackerMedianFlow_create() # Create tracker
BBox = None                              # Bounding box initialize


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Servo Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

servo = ServoDriver.Servo()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Main Loop
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

while True: # loop over every frame in video object

    frame = video.read() # get video frame
    
    # if the video object ends, exit program
    if frame is None:
        break

    # rotate frame 
    #frame = imutils.rotate(frame, 180)
    
    # get frame dimensions
    dimensions = tuple(frame.shape[:2])
    midpoint = tuple((x/2) for x in dimensions)
    #print(midpoint)

    if BBox is not None:
        # get new bounding box
        (success, box) = tracker.update(frame)

        # check for tracker success
        if success:
            
            (x, y, width, height) = [int(v) for v in box]
            cv2.rectangle(frame, (x,y), (x + width, y + height), (255,0,255), 2)
            center = (int(x + (width/2)), int(y + (height/2)))

            cv2.circle(frame, center, 10, (0,0,255), -1)

            # tilt servo using proportional movement
            print(center)
            proportion_tilt = center[1] - (dimensions[0] / 2)
            proportion_tilt = 6 * (float(proportion_tilt / (dimensions[0] / 2)))
            if proportion_tilt > 0.15 or proportion_tilt < -0.15:
                servo.t_vel = proportion_tilt
            else:
                servo.t_vel = 0.0
            #servo.t_vel = 1.0

            # pan servo using proportional movement
            proportion = center[0] - (dimensions[1] / 2)
            proportion = -6 * (float(proportion / (dimensions[1] / 2)))
            if proportion > 0.15 or proportion < -0.15:
                servo.p_vel = proportion
            else:
                servo.p_vel = 0.0

    
    if args.v is True: # Write to video
        video_out.write(frame)

    cv2.imshow("My cute tracker :)", frame) # Show the frame on monitor
    key = cv2.waitKey(1) & 0xFF


    if key == ord("s"):
        BBox = cv2.selectROI("My cute tracker :)", frame, fromCenter=False, showCrosshair=True)
        tracker.init(frame, BBox)

    elif key == ord("q"):
        fan.set(on=0)
        break

if args.v is True:
    video_out.release()
video.release()

cv2.destroyAllWindows()




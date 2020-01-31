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
#   Argument Parsing
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

parser = argparse.ArgumentParser(description='trailTrace Tracking demo...\n\n')
parser.add_argument('-v', help='Save video to file', action='store_true')
parser.add_argument('-r', help='Restore trailTracer:    Delete Videos', action='store_true')
args = parser.parse_args()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Fan Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

fan = FanDriver.Fan()    # Create fan object
fan.set(speed=0.8, on=1) # Fan on, speed 80%

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Load Data
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

video_data = open("./data/video_data","r+") # open video data file
video_num = int(video_data.readline())      # read the current video number
print(video_num)
video_num += 1                              # increment video_number: next video will be ++
video_data.seek(0,0)                        # move to beginning

if args.r is True:                                              # User requested videos to be deleted
    user_in = input('Restore: are you sure? (y/n): ')
    if user_in.lower() == 'y' or user_in.lower() == 'yes':      # Test for any yes response
        video_data.write(bytes(0))                              # restore video number to zero   
    quit()

video_data.write(str(video_num))            # write newe num
video_data.close()

servo_data = open("./data/servo_data","r+") # open servo data file
servo_pos = { "tilt": servo_data.readline(), "pan": servo_data.readline() }


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

pan_servo = ServoDriver.PanServo()
tilt_servo = ServoDriver.TiltServo()


while True:
    current_time = time.clock_gettime(time.CLOCK_REALTIME) # get current time
    print(current_time)

#if current_time - prev_time >= 0.01:            

            # tilt servo using proportional movement
            #print(center)
#                error_tilt = center[1] - (dimensions[0] / 2)
#                tilt_servo.update(error=error_tilt, axis_range=(dimensions[0] / 2))
            
                       # pan servo using proportional movement
#                error_pan = center[0] - (dimensions[1] / 2)
#                pan_servo.update(error=error_pan, axis_range=(dimensions[1] / 2))
#            prev_time = current_time


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Main Loop
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

prev_time = 0;

while True: # loop over every frame in video object

       
        
    frame = video.read() # get video frame
    #print(time.clock_gettime(time.CLOCK_REALTIME))   
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

            current_time = time.clock_gettime(time.CLOCK_REALTIME) # get current time
 
           
            if current_time - prev_time >= 0.01:
            
                print("servo update")


            # tilt servo using proportional movement
            #print(center)
                error_tilt = center[1] - (dimensions[0] / 2)
                tilt_servo.update(error=error_tilt, axis_range=(dimensions[0] / 2))
            
                       # pan servo using proportional movement
                error_pan = center[0] - (dimensions[1] / 2)
                pan_servo.update(error=error_pan, axis_range=(dimensions[1] / 2))
            prev_time = current_time



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




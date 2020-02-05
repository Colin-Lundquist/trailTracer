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
import signal
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
servo_pos = { "pan": int(servo_data.readline()), "tilt": int(servo_data.readline()) }


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
            fps=15, 
            frameSize=(640,480))

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Tracker Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

tracker = cv2.TrackerMedianFlow_create()    # Create tracker
BBox = None                                 # Bounding box initialize
deltaBox = None

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Servo Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

pan_servo = ServoDriver.PanServo(pos=servo_pos["pan"])
tilt_servo = ServoDriver.TiltServo(pos=servo_pos["tilt"])
#pan_servo.moveDefault(pan_current = int(servo_pos["pan"]), tilt_current = int(servo_pos["pan"])

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Signal Handlers
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def shutdown():
    fan.set(on=0)
    servo_data.seek(0,0)
    servo_data.writelines([str(int(pan_servo.pos))+'\n', str(int(tilt_servo.pos))+'\n'])

    cv2.imwrite('test.jpg',im_out)

    if args.v is True:
        video_out.release()

    cv2.destroyAllWindows()



def ctrl_c_handler(sig, frame):
    shutdown()
    
signal.signal(signal.SIGINT, ctrl_c_handler)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Main Loop
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

prev_time = 0;
last_gray = None
img_out = None
delta = None
tracking = False
tracker_timeout = 1.0
tracking_still_time = 0.0
acquiring_warmup = 0.3
acquiring_still_time = 0.0

acquire_contours = 0
contours_list = [[0, 0], [0, 0], [0, 0]]

while True: # loop over every frame in video object
       
    current_time = time.clock_gettime(time.CLOCK_REALTIME)


    img = video.read() # get video frame
    #print(time.clock_gettime(time.CLOCK_REALTIME))   
    #frame = cv2.blur(img,(10,10))
    frame = img
    gray = imutils.resize(img, width=320)
    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    cv2.fastNlMeansDenoising(gray, gray, searchWindowSize=5)               
    
    
    
    
    #frame = imutils.resize(frame, width=320)
    # if the video object ends, exit program
    if frame is None:
        break

    # rotate frame 
    #frame = imutils.rotate(frame, 180)
    
    # get frame dimensions
    dimensions = tuple(frame.shape[:2])
    midpoint = tuple((x/2) for x in dimensions)
    #print(midpoint)
    
    
    
    if tracking is True:
    #if BBox is not None:
        (success, BBox) = tracker.update(frame)

        #print(BBox)
        
        if args.v is True: # Write to video
            video_out.write(img)

        # check for tracker success
        if success:
            print("tracking")            
            (x, y, width, height) = [int(v) for v in BBox]
            cv2.rectangle(frame, (x,y), (x + width, y + height), (255,0,255), 2)
            center = (int(x + (width/2)), int(y + (height/2)))

            cv2.circle(frame, center, 10, (0,0,255), -1)

            #current_time = time.clock_gettime(time.CLOCK_REALTIME) # get current time
           
            if current_time - prev_time >= 0.01:

            # tilt servo using proportional movement
            # print(center)
                error_tilt = center[1] - (dimensions[0] / 2)
                tilt_servo.update(error=error_tilt, axis_range=(dimensions[0] / 2))
            
                # pan servo using proportional movement
                error_pan = center[0] - (dimensions[1] / 2)
                pan_servo.update(error=error_pan, axis_range=(dimensions[1] / 2))
           
                print("error_tilt: %f", error_tilt)
                
                if error_tilt < 10.0:
                    #print(error_tilt)
                    tracking_still_time += 0.01
                else:
                    tracking_still_time = 0.0
                if tracking_still_time >= tracker_timeout:
                    print("tracker_timeout")
                    tracking = False
                    tracking_still_time = 0.0

                

        else:
            print("tracking_fail")
            tracking = False
    else:
        print("Not Tracking")    
        if not last_gray is None:
            time.sleep(0.1)
            delta = cv2.absdiff(gray, last_gray)
            delta = cv2.threshold(delta, 25, 255, cv2.THRESH_BINARY)[1]
            delta = cv2.erode(delta, None, iterations=1)
            delta = cv2.dilate(delta, None, iterations=6)
            
            cv2.imshow("gray", delta)
            
            cnts = cv2.findContours(delta.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cnts = imutils.grab_contours(cnts)
            
            if cnts:
                c = max(cnts, key=cv2.contourArea)
                #for c in cnts:
#                    extLeft = tuple(c[c[:, :, 0].argmin()][0])
                if (cv2.contourArea(c) > 800):
                    extLeft = tuple( c[c[:, :, 0].argmin()][0] )
                    extLeft = tuple( [x*2 for x in extLeft] )

                    extRight = tuple( c[c[:, :, 0].argmax()][0] )
                    extRight = tuple( [x*2 for x in extRight] )

                    extTop = tuple( c[c[:, :, -1].argmax()][0] )
                    extTop = tuple( [x*2 for x in extTop] )

                    extBot = tuple( c[c[:, :, -1].argmin()][0] )
                    extBot = tuple( [x*2 for x in extBot] )

                    cv2.rectangle(frame, (extLeft[0],extTop[1]),(extRight[0],extBot[1]), (255,255,0), thickness=3)
                    
                    print(acquire_contours)
                    contours_list[acquire_contours][0] = c # Add a contour to the list
                    contours_list[acquire_contours][1] = (extLeft[0], extBot[1], (extRight[0] - extLeft[0]), (extTop[1] - extBot[1]))
                

                    acquire_contours = acquire_contours + 1

                    if acquire_contours >= 3:
                        
                        area_list = [x[1][2]*x[1][3] for x in contours_list]
                        max_index = area_list.index(max(area_list))

                        #max_index = contours_list.index(max([x[1][2]*x[1][3] for x in contours_list]))
                    
                        #BBox = contours_list[max_index][1]
                        BBox = contours_list[2][1]
                        print(BBox)
                        tracking = True
                        acquire_contours = 0
                        #tracker.update(frame, BBox)
                        tracker = cv2.TrackerMedianFlow_create() 
                        tracker.init(frame, BBox)
        
        last_gray = gray
            
            


    


    cv2.imshow("My cute tracker :)", frame) # Show the frame on monitor
    cv2.imshow("Output", img)
    key = cv2.waitKey(1) & 0xFF


    if key == ord("s"):
        BBox = cv2.selectROI("My cute tracker :)", frame, fromCenter=False, showCrosshair=True)
        #print(BBox)
        tracker.init(frame, BBox)
        tracking = True

    elif key == ord("q"):
        shutdown()       
        break

    prev_time = current_time


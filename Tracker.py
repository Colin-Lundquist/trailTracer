#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   File:       Tracker.py
#   Author:     Colin Lundquist
#   Date:       1/22/2020
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~

from imutils.video import WebcamVideoStream

import argparse
import imutils
import signal
import time
import cv2
import os

# trailTracer libraries
from trailTracer.servo import ServoDriver
from trailTracer.gpio import IODriver
from trailTracer.fan import FanDriver

CAMERA_BOOTTIME = 0.5

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Argument Parsing
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

parser = argparse.ArgumentParser(description='trailTrace Tracking demo...\n\n')
parser.add_argument('-f', help='Save video to file', action='store_true')
parser.add_argument('-r', help='Restore trailTracer:    Delete Videos', action='store_true')
parser.add_argument('-v', help='Verbose output', action='store_true')
parser.add_argument('-ascii', help='Display motion on console as ascii-art', action='store_true')
parser.add_argument('-profile', help='Run performance profiling on program', action='store_true')
args = parser.parse_args()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Profiler Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

if args.profile is True:
    import cProfile, pstats, io
    from pstats import SortKey
    pr = cProfile.Profile()
    pr.enable()

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Display Splash
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

splash_screen = open("./data/splash.txt", "r")
for line in splash_screen:
    print(line[0:-1])
splash_screen.close()


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Fan Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

fan = FanDriver.Fan()    # Create fan object
fan.set(speed=1.0, on=1) # Fan on, speed 80%

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Video and Servo Data
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

servo_data = open("./data/servo_data","r+") # open servo data file
servo_pos = { "pan": int(servo_data.readline()), "tilt": int(servo_data.readline()) }


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Video Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

print(" Beginning PiCam stream...")  # notify user
#video = VideoStream(src=0).start()  # start camera stream
video = WebcamVideoStream(src=0).start()
time.sleep(CAMERA_BOOTTIME)         # allow camera to warm up

if args.f is True: # Create video writer object if video argument is passed

    video_data = open("./data/video_data","r+") # open video data file
    video_num = int(video_data.readline())      # read the current video number


    video_num += 1                              # increment video_number: next video will be ++
    print("save stream to: trailTracer/Videos/trailTrace_%d.mp4" % video_num)

    video_data.seek(0,0)                        # move to beginning


    if args.r is True:                                              # User requested videos to be deleted
        user_in = input(' Delete all Videos: are you sure? (y/n): ')
        if user_in.lower() == 'y' or user_in.lower() == 'yes':      # Test for any yes response
            video_data.write("0\n")                              # restore video number to zero
            os.system("rm ./Videos/*.mp4")
        quit()



    video_data.write("%s\n" %(str(video_num)))            # write new num
    video_data.close()

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
#tilt_servo.SetEnabled(enabled=1)                            # Disable tilt servo until we track
#pan_servo.moveDefault(pan_current = int(servo_pos["pan"]), tilt_current = int(servo_pos["pan"]))

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Signal Handlers
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def shutdown():

    fan.set(on=0)
    tilt_servo.disable()


    servo_data.seek(0,0)
    servo_data.writelines([str(int(pan_servo.pos))+'\n', str(int(tilt_servo.pos))+'\n'])

    if args.f is True:
        video_out.release()

    cv2.destroyAllWindows()
    print('\nCTRL+C EVENT\n\n')
    
    if args.profile is True:
        pr.disable()
        s = io.StringIO()
        sortby = SortKey.CUMULATIVE
        ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
        ps.print_stats()
        print(s.getvalue())

    quit()

def ctrl_c_handler(sig, frame):
    shutdown()
    #break
signal.signal(signal.SIGINT, ctrl_c_handler)


#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Main Loop
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

states = {"Tracking": False, "Runtime": 0.0}

img = None
prev_time = 0
last_gray = None
img_out = None
delta = None
tracking = False
tracker_timeout = 0.45
tracking_still_time = 0.0
acquiring_warmup = 0.4
acquiring_timeout = 0.2
acquiring_still_time = 0.0
start_time = acquisition_start_time = time.clock_gettime(time.CLOCK_REALTIME)
error_tilt = None
error_pan = None
acquire_contours = 0
contours_list = [[0, 0], [0, 0], [0, 0]]

while True: # loop over every frame in video object

    current_time = time.clock_gettime(time.CLOCK_REALTIME)
    states["Runtime"] = current_time - start_time

    frame = video.read() # get video frame
    #frame = cv2.blur(img,(10,10))
    #frame = img.copy()
    #frame = img
    gray = imutils.resize(frame, width=320)

    gray = cv2.cvtColor(gray, cv2.COLOR_BGR2GRAY)
    if args.v is True: cv2.imshow("Gray", gray)

    gray = cv2.blur(gray,(10,10))
    


    # if the video object ends, exit program
    if frame is None:
        break

    #frame = imutils.rotate(frame, 180)

    # get frame dimensions
    dimensions = tuple(frame.shape[:2])
    midpoint = tuple((x/2) for x in dimensions)


    if states["Tracking"] is True:

        (success, BBox) = tracker.update(frame)

        if args.f is True: # Write to video
            video_out.write(frame)

        # check for tracker success
        if success:

            (x, y, width, height) = [int(v) for v in BBox]
            #cv2.rectangle(frame, (x,y), (x + width, y + height), (255,0,255), 2)
            center = (int(x + (width/2)), int(y + (height/2)))

            #cv2.circle(frame, center, 10, (0,0,255), -1)

            if current_time - prev_time >= 0.01:

                error_tilt = center[1] - (dimensions[0] / 2)
                tilt_servo.update(error=error_tilt)

                error_pan = center[0] - (dimensions[1] / 2)
                pan_servo.update(error=error_pan)

                #print("error_tilt: %f", error_tilt)

                if error_tilt < 15.0 and error_pan < 15.0:
                    #print(error_tilt)
                    tracking_still_time += 0.01
                    #tilt_servo.SetEnabled(0)
                    #tilt_servo.disable()
                else:
                    tracking_still_time = 0.0
                    #tilt_servo.SetEnabled(1)
                    tilt_servo.enable()

                # If our box gets too big, or we timeout standing stll
                if tracking_still_time >= tracker_timeout or width*height > 202500: # 202,500 == 450x450 px
                    print("tracker_timeout")
                    states["Tracking"] = False
                    tracking_still_time = 0.0
                    time.sleep(0.1)
                    acquisition_start_time = current_time
                    tilt_servo.disable()
                    #tilt_servo.SetEnabled(0)

        else:
            print("tracking_failure")
            states["Tracking"] = False
            tracking_still_time = 0.0
            tilt_servo.disable()
            time.sleep(0.25)
            acquisition_start_time = current_time
    else:

        if not last_gray is None:

            delta = cv2.absdiff(gray, last_gray)
            #cv2.fastNlMeansDenoising(delta, delta, searchWindowSize=5)
            delta = cv2.dilate(delta, None, iterations=5)

            #delta = cv2.threshold(delta, 4, 255, cv2.THRESH_BINARY)[1]

            #cnts = cv2.findCountours(delta.copy(),cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


            if args.v is True: cv2.imshow("diff", delta)
            delta = cv2.erode(delta, None, iterations=7)
            delta = cv2.dilate(delta, None, iterations=2)
            delta = cv2.threshold(delta, 4, 255, cv2.THRESH_BINARY)[1]
            #delta = cv2.dilate
            #delta = cv2.blur(delta, (10,10))
            #delta = cv2.threshold(delta, 6, 255, cv2.THRESH_BINARY)[1]
            #delta = cv2.erode(delta, None, iterations=2)
            delta = cv2.dilate(delta, None, iterations=5)

            small = imutils.resize(delta, width=40)

            if args.ascii is True:
                ascii_art = [' ' if y == 0 else '@' for x in small for y in x]
                i = 0
                for character in ascii_art:
                    print(ascii_art[i] if i % 40 != 0 else ascii_art[i] + '\n', end='')
                    i=i+1

            if args.v is True:
                cv2.imshow("diff blurred", delta)

            cnts = cv2.findContours(delta.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


            cnts = imutils.grab_contours(cnts)

            if cnts:
                c = max(cnts, key=cv2.contourArea)
                #for c in cnts:
#                    extLeft = tuple(c[c[:, :, 0].argmin()][0])
                if (cv2.contourArea(c) > 1000):

                    BBox = cv2.boundingRect(c)
                    BBox = (x,y,w,h) = tuple([2*x for x in BBox] )

                    #print(acquire_contours)
                    contours_list[acquire_contours][0] = c # Add a contour to the list
                    contours_list[acquire_contours][1] = (x, (y+h), w, h)


                    acquire_contours = acquire_contours + 1

                    #print((current_time - acquisition_start_time))
                    if current_time - acquisition_start_time > 0.2:
                        acquire_contours = 0
                        acquisition_start_time = 0 + current_time
                        #print("took too long")

                    if acquire_contours >= 3:

                        acquisition_start_time = current_time
                        states["Tracking"] = True
                        acquire_contours = 0
                        #tracker.update(frame, BBox)
                        tracker = cv2.TrackerMedianFlow_create()
                        tracker.init(frame, BBox)
                        time.sleep(0.1)
                        #tilt_servo.SetEnabled(1)
                        tilt_servo.enable()
    last_gray = gray


    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    #   Verbose output
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    if args.v == True:
        print("\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print("# T+ %f" %states["Runtime"], "  Tracking: %s" %("False" if states["Tracking"] is False else "True"), \
                "  Pan: %s" %("Disabled" if pan_servo.enabled is False else "Enabled"), "  Tilt: %s" %("Disabled" if\
                tilt_servo.enabled is False else "Enabled"), "\n#\t\t\t\tError: %s" %(str(error_pan) if error_pan is not None else "N/A"),\
                "  Error: %s" %(str(error_tilt) if error_tilt is not None else "N/A"), "\n#")
   #print(pan_servo.enabled)

    #

    print (BBox)
    if args.v is True: 
        #if img is not None: cv2.imshow("My cute tracker :)", img) # Show the frame on monitor
        if args.v is True and BBox is not None and states["Tracking"] is False:                       #img = frame.copy()
            img = frame.copy()     
            cv2.rectangle(img, (BBox[0],BBox[1]),(BBox[0] + BBox[2],BBox[1] + BBox[3]), (255,255,0), thickness=3)
            cv2.putText(img, '%dx%d' %((w),(h)), (10,50), cv2.FONT_HERSHEY_COMPLEX,\
                fontScale=1, thickness=2, color=(0,255,100))
            cv2.imshow("image", img)
       
        #cv2.imshow("Output", img)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("s"):
            BBox = cv2.selectROI("My cute tracker :)", frame, fromCenter=False, showCrosshair=True)
            #print(BBox)
            tracker.init(frame, BBox)
            states["Tracking"] = True

        elif key == ord("q"):
            shutdown()
            break

    prev_time = current_time


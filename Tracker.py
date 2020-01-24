from imutils.video import VideoStream
from imutils.video import FPS

#trailTracer libraries
from lib.servo import ServoDriver
from lib.gpio import GPIO

import imutils
import time
import cv2

# pause duration when camera wakes up
CAMERA_BOOTTIME = 0.75

servo = ServoDriver.Servo()
#servo2 = ServoDriver.TiltServo()

# creat tracker
tracker = cv2.TrackerMedianFlow_create()

#initialize bounding box
BBox = None

# begin camera stream
print("Beginning PiCam stream...")
video = VideoStream(src=0).start()
# allow camera to warm up
time.sleep(CAMERA_BOOTTIME)

# loop over every frame in video object
while True:

    frame = video.read()
    
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

    
        # rotate image
    #M = cv2.getRotationMatrix2D(midpoint,180,1.0)
    #frame = cv2.warpAffine(frame, M, dimensions)

    
    # show frame in window
    cv2.imshow("My cute tracker :)", frame)
    key = cv2.waitKey(1) & 0xFF


    if key == ord("s"):
        BBox = cv2.selectROI("My cute tracker :)", frame, fromCenter=False, showCrosshair=True)
        tracker.init(frame, BBox)

    elif key == ord("q"):
        break

video.release()

cv2.destroyAllWindows()




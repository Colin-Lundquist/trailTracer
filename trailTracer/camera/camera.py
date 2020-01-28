from picamera.array import PiRGBArray
from picamera import PiCamera
import imutils
import pigpio
import time
import cv2

# Setup Camera
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

#Setup GPIO

pi = pigpio.pi()
pi.set_pull_up_down(18, pigpio.PUD_OFF)

# allow cam warmup
time.sleep(0.1)

last_gray = 0

avgLeft = 0;
avgRight = 0;
avgTop = 0;
avgBot = 0;

out = cv2.VideoWriter('outpy.mp4', cv2.VideoWriter_fourcc('M','P','4','V'), 10, (640,480))



#grap image
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #grab raw NumPy array representing image
    image = frame.array
   
    #rotate image


    #convert to gray, reduce resolution, and blur
    gray = imutils.resize(image, width=320)
    #gray = image
    gray = cv2.cvtColor(gray,cv2.COLOR_BGR2GRAY)
    
    
    if not (last_gray is None):
        delta = cv2.absdiff(gray, last_gray)
        delta = cv2.threshold(delta, 25 ,255, cv2.THRESH_BINARY)[1]
        delta = cv2.erode(delta, None, iterations = 1)
        delta = cv2.dilate(delta, None, iterations=6)

        
    last_gray = gray

    #M = cv2.moments(delta)

    #if M["m00"] != 0:
    #    cX = int(M["m10"] / M["m00"])
    #    cY = int(M["m01"] / M["m00"])

    #    cX = int(cX*2)
    #    cY = int(cY*2)
        
    #cv2.circle(image, (cX,cY), 12, (255,0,0), -1)


    cnts = cv2.findContours(delta.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    
    
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        #for c in cnts:
#            extLeft = tuple(c[c[:, :, 0].argmin()][0])
        if (cv2.contourArea(c) > 800):
            extLeft = tuple( c[c[:, :, 0].argmin()][0] )
            extLeft = tuple( [x*2 for x in extLeft] )

            extRight = tuple( c[c[:, :, 0].argmax()][0] )
            extRight = tuple( [x*2 for x in extRight] )

            extTop = tuple( c[c[:, :, -1].argmax()][0] )
            extTop = tuple( [x*2 for x in extTop] )

            extBot = tuple( c[c[:, :, -1].argmin()][0] )
            extBot = tuple( [x*2 for x in extBot] )

        
            cv2.circle(image, extRight, 10, (255,0,255), -1)
            cv2.circle(image, extLeft, 10, (255,255,0), -1)
            cv2.circle(image, extBot, 10, (0,255,255), -1)
            cv2.circle(image, extTop, 10, (128,128,128), -1)

        
        
        cv2.rectangle(image, (extLeft[0],extTop[1]),(extRight[0],extBot[1]), (255,255,0), thickness=3)
       
    M = cv2.getRotationMatrix2D((320,240),90,1.0)
    img_rotated = cv2.warpAffine(image, M, (640,480))

    #show the frame
    cv2.imshow("Video :P", img_rotated)
    out.write(image)

    
    key = cv2.waitKey(1) & 0xFF

    #clear the stream in preparation for next frame
    rawCapture.truncate(0)

    #stop is 's' is pressed
    if key == ord("s"):
        break
    
out.release()

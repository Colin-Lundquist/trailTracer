from imutils.video import WebcamVideoStream
import time

import signal


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



def ctrl_c_handler(sig, frame):
    shutdown()
    
signal.signal(signal.SIGINT, ctrl_c_handler)

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   Video Setup
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def video_setup(boottime):

    print(" Beginning PiCam stream...")  # notify user
    #video = VideoStream(src=0).start()  # start camera stream
    video = WebcamVideoStream(src=0).start()
    time.sleep(boottime)         # allow camera to warm up

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






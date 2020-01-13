# File:     ServoDriver.py
# Project:  trailTracer
# Author:   Colin Lundquist
# Date:     1/11/2020
#
# This module contains the methods and members 
# needed to drive the servos.

import GPIO
import signal


class PanServo:

    # Class attributes
    POS_MAX = 2380     # The highest value allowed for the pan servo
    POS_MIN = 520      # The lowest value allowed for the pan servo
    DEFAULT_POS = 1450 # Initial position of pan servo
    DEFAULT_VEL = 0    # By default, the servo does not move!

    #position = DEFAULT_POS # set the position to default
    #velocity = DEFAULT_VEL # set velocity to default

    def __init__(self, pos=DEFAULT_POS, vel=DEFAULT_VEL):
        # Instance attributes
        self.position = pos # Initialize position
        self.velocity = vel # Initialize velocity
    
        signal.signal(signal.SIGALRM, self.SERVO_1MS_HANDLER)
        signal.setitimer(signal.ITIMER_REAL, 1.0, 0.02)
        #signal.alarm(0.01)
    def SERVO_1MS_HANDLER(self, frame, other):
        
        #os.system("clear")
        #print('Position: ',PanServo.position, 'Velocity: ', PanServo.velocity)
        if (((self.position + self.velocity) <= self.POS_MAX) and ((self.position + self.velocity) >= self.POS_MIN)):
            self.position = self.position + self.velocity
            GPIO.pi.set_servo_pulsewidth(14, self.position)
            
        else:
            self.velocity = self.DEFAULT_VEL
            




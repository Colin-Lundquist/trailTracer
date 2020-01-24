# File:     ServoDriver.py
# Project:  trailTracer
# Author:   Colin Lundquist
# Date:     1/11/2020
#
# This module contains the methods and members 
# needed to drive the servos.

from lib.gpio import GPIO
import signal

class Servo:

    # Pan Servo class attributes 
    PAN_POS_MAX = 2380     # The highest value allowed for the pan servo
    PAN_POS_MIN = 520      # The lowest value allowed for the pan servo
    PAN_DEFAULT_POS = 2000 # Initial position of pan servo
    PAN_DEFAULT_VEL = 0    # By default, the servo does not move!
    PAN_GPIO_PIN = 14

    # Tilt Servo class attributes
    TILT_POS_MAX = 1800      # The highest value allowed for the tilt servo
    TILT_POS_MIN = 1200      # The lowest value allowed for the tilt servo
    TILT_DEFAULT_POS = 1550  # Initial position of pan servo
    TILT_DEFAULT_VEL = 0     # By default, the servo does not move!
    TILT_GPIO_PIN = 25
 
    #position = DEFAULT_POS # set the position to default
    #velocity = DEFAULT_VEL # set velocity to default

    def __init__(self, p_pos=PAN_DEFAULT_POS,  t_pos=TILT_DEFAULT_POS):
        
        # Pan Instance attributes
        self.p_pos = p_pos # Initialize pan position
        self.p_vel = self.PAN_DEFAULT_POS # Initialize velocity
        # Tilt Instance attributes
        self.t_pos = t_pos # initialize tilt position 
        self.t_vel = self.TILT_DEFAULT_VEL

        
        signal.signal(signal.SIGALRM, self.SERVO_1MS_HANDLER)
        signal.setitimer(signal.ITIMER_REAL, 1.0, 0.02)
        
        
        #signal.alarm(0.01)
    def SERVO_1MS_HANDLER(self, frame, other):
        
        #os.system("clear")
        #print('Position: ',PanServo.position, 'Velocity: ', PanServo.velocity)
        if (((self.p_pos + self.p_vel) <= self.PAN_POS_MAX) and ((self.p_pos + self.p_vel) >= self.PAN_POS_MIN)):
            self.p_pos = self.p_pos + self.p_vel
            GPIO.pi.set_servo_pulsewidth(self.PAN_GPIO_PIN, self.p_pos)
            
        else:
            self.p_vel = self.PAN_DEFAULT_VEL
 
        if (((self.t_pos + self.t_vel) <= self.TILT_POS_MAX) and ((self.t_pos + self.t_vel) >= self.TILT_POS_MIN)):
            self.t_pos = self.t_pos + self.t_vel
            GPIO.pi.set_servo_pulsewidth(self.TILT_GPIO_PIN, self.t_pos)
            
        else:
            self.t_vel = self.TILT_DEFAULT_VEL
 

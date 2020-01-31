# File:     ServoDriver.py
# Project:  trailTracer
# Author:   Colin Lundquist
# Date:     1/11/2020
#
# This module contains the methods and members 
# needed to drive the servos.

from trailTracer.gpio import IODriver

class Servo:
    
    POS_MAX = None
    POS_MIN = None
    DEFAULT_POS = None
    DEFAULT_VEL = None
    DEFAULT_POS = None
    
    # PID control constants
    TIME_STEP = 0.01    # time between updates
    DEAD_ZONE = 0.5     # minimum velocity to be applied to servo
    P_FACTOR = 0.0      # proportional multiplier (increases proportional effect)
    I_FACTOR = 0.0      # integral multiplier (increases integral effect)
    D_FACTOR = 0.0      # derivitave multiplier (increases derivitave effect)
    
    prev_error = 0      # how much the servo was in error at Time: now - TIME_STEP 
    integral = 0        # integral component 'I'


    # class constructor
    def __init__(self, pos=None):           
    
        if pos is None:
            self.pos = self.DEFAULT_POS     # set default position
        else:
            self.pos = pos                  # Initialize position
        self.vel = self.DEFAULT_VEL         # Initialize velocity        
        
    def update(self, error, axis_range):
      
        self.integral = self.integral + error * self.TIME_STEP             # Integral ramps up based on time in error
        self.derivative = (error - self.prev_error) / self.TIME_STEP       # Derivitave dampens based on speed
        
        self.vel = self.P_FACTOR * error + self.I_FACTOR * self.integral \
            - self.D_FACTOR * self.derivative
        self.prev_error = error             # save error value for next time
        
        if (abs(self.vel) > self.DEAD_ZONE):
            if (((self.pos + self.vel) <= self.POS_MAX) and ((self.pos + self.vel) >= self.POS_MIN)):
                self.pos = self.pos + self.vel
                IODriver.pi.set_servo_pulsewidth(self.GPIO_PIN, self.pos)
        else:
            vel = self.DEFAULT_VEL

class PanServo(Servo):

    # Pan Servo class attribute redefinitions
    POS_MAX = 2380     
    POS_MIN = 520      
    DEFAULT_POS = 2000 
    DEFAULT_VEL = 0 
    GPIO_PIN = 14

    # PID control:
    TIME_STEP = 0.01
    DEAD_ZONE = 0.5
    P_FACTOR = -0.09
    I_FACTOR = 0.0
    D_FACTOR = 0.0018


class TiltServo(Servo):

    # Tilt Servo class attribute redefinitions
    POS_MAX = 1800      
    POS_MIN = 1200  
    DEFAULT_POS = 1550
    DEFAULT_VEL = 0     
    GPIO_PIN = 25
 
    # PID control:
    TIME_STEP = 0.01
    DEAD_ZONE = 0.5     
    P_FACTOR = 0.08
    I_FACTOR = 0.0
    D_FACTOR = 0.0018


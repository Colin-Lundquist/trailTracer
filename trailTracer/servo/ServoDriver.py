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
    VEL_MAX = None
    DEFAULT_POS = None
    DEFAULT_VEL = None
    DEFAULT_POS = None
    
    # PID control constants
    TIME_STEP = 0.01    # time between updates
    DEAD_ZONE = 0.8     # minimum error to be applied to servo
    P_FACTOR = 0.0      # proportional multiplier (increases proportional effect)
    I_FACTOR = 0.0      # integral multiplier (increases integral effect)
    D_FACTOR = 0.0      # derivitave multiplier (increases derivitave effect)
    
    prev_error = 0      # how much the servo was in error at Time: now - TIME_STEP 
    integral = 0        # integral component 'I'
    enabled = False

    # class constructor
    def __init__(self, pos=None, enabled=True):           
    
        self.enabled = enabled

        if pos is None:
            self.pos = self.DEFAULT_POS     # set default position
        else:
            self.pos = pos                  # Initialize position
        self.vel = self.DEFAULT_VEL         # Initialize velocity        
            



    def update(self, error):

        if self.enabled is True:
            if abs(error) > self.DEAD_ZONE:
                self.integral = self.integral + error * self.TIME_STEP             # Integral ramps up based on time in error
                self.derivative = (error - self.prev_error) / self.TIME_STEP       # Derivitave dampens based on speed
        
                self.vel = self.P_FACTOR * error + self.I_FACTOR * self.integral \
                    - self.D_FACTOR * self.derivative
           
                if abs(self.vel) > self.VEL_MAX:
                    self.vel = self.VEL_MAX if self.vel > 0 else -1*self.VEL_MAX

                self.pos = self.pos + self.vel
            
                IODriver.pi.set_servo_pulsewidth(self.GPIO_PIN, self.pos)
            

                if  (((self.pos + self.vel) <= self.POS_MAX) and ((self.pos + self.vel) >= self.POS_MIN)):
                    self.pos = self.pos + self.vel

            else:
                vel = self.DEFAULT_VEL

        else:
            IODriver.pi.set_servo_pulsewidth(self.GPIO_PIN, 0)
        
        self.prev_error = error             # save error value for next time
    

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False
        IODriver.pi.set_servo_pulsewidth(self.GPIO_PIN, 0)
                       

class PanServo(Servo):

    # Pan Servo class attribute redefinitions
    POS_MAX = 2380     
    POS_MIN = 520 
    VEL_MAX = 10
    DEFAULT_POS = 1500
    DEFAULT_VEL = 0 
    GPIO_PIN = 14

    # PID control:
    TIME_STEP = 0.01
    DEAD_ZONE = 0.0
    P_FACTOR = -0.05
    I_FACTOR = 0.001
    D_FACTOR = 0.00125


class TiltServo(Servo):
    # Tilt Servo class attribute redefinitions
    POS_MAX = 1800      
    POS_MIN = 1200
    VEL_MAX = 10
    DEFAULT_POS = 1500
    DEFAULT_VEL = 0     
    GPIO_PIN = 25
 
    # PID control:
    TIME_STEP = 0.01
    DEAD_ZONE = 2.0
    P_FACTOR = 0.08
    I_FACTOR = 0.001
    D_FACTOR = 0.000298



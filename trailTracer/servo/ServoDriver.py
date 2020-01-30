# File:     ServoDriver.py
# Project:  trailTracer
# Author:   Colin Lundquist
# Date:     1/11/2020
#
# This module contains the methods and members 
# needed to drive the servos.

from trailTracer.gpio import IODriver
import signal

class Servo:

    #position = DEFAULT_POS # set the position to default
    #velocity = DEFAULT_VEL # set velocity to default

    

    POS_MAX = None
    POS_MIN = None
    DEFAULT_POS = None
    DEFAULT_VEL = None
    DEFAULT_POS = None
    
    # PID control:
    TIME_STEP = 0.01
    DEAD_ZONE = 0.5     # minimum velocity ot be applied to servo
    P_FACTOR = 0.0
    I_FACTOR = 0.0
    D_FACTOR = 0.0
    
    prev_error = 0    
    integral = 0        # integral component 'I'

    


    def __init__(self, pos=None):
    
        if pos is None:
            self.pos = self.DEFAULT_POS
        else:
            self.pos = pos                  # Initialize position
        self.vel = self.DEFAULT_VEL     # Initialize velocity
        
        
        
    def update(self, error, axis_range):
      
        self.integral = self.integral + error * self.TIME_STEP     
        self.derivative = (error - self.prev_error) / self.TIME_STEP
        self.vel = self.P_FACTOR * error + self.I_FACTOR * self.integral - self.D_FACTOR * self.derivative
        self.prev_error = error
        print(self.vel)
        
        #os.system("clear")
        #print('Position: ',PanServo.position, 'Velocity: ', PanServo.velocity)
        #print(self.pos)
        #print(self.vel)
        #print(self.POS_MAX)
        #print(self.POS_MIN)

        
        
        if (abs(self.vel) > self.DEAD_ZONE):
            if (((self.pos + self.vel) <= self.POS_MAX) and ((self.pos + self.vel) >= self.POS_MIN)):
                self.pos = self.pos + self.vel
                IODriver.pi.set_servo_pulsewidth(self.GPIO_PIN, self.pos)
        else:
            vel = self.DEFAULT_VEL

class PanServo(Servo):

    # Pan Servo class attributes 
    POS_MAX = 2380     # The highest value allowed for the pan servo
    POS_MIN = 520      # The lowest value allowed for the pan servo
    DEFAULT_POS = 2000 # Initial position of pan servo
    DEFAULT_VEL = 0    # By default, the servo does not move!
    GPIO_PIN = 14

    # PID control:
    TIME_STEP = 0.01
    DEAD_ZONE = 0.5     # minimum velocity ot be applied to servo
    P_FACTOR = -0.1
    I_FACTOR = 0.0
    D_FACTOR = 0.0018


class TiltServo(Servo):

    # Tilt Servo class attributes
    POS_MAX = 1800      # The highest value allowed for the tilt servo
    POS_MIN = 1200      # The lowest value allowed for the tilt servo
    DEFAULT_POS = 1550  # Initial position of pan servo
    DEFAULT_VEL = 0     # By default, the servo does not move!
    GPIO_PIN = 25
 
    # PID control:
    TIME_STEP = 0.01
    DEAD_ZONE = 0.5     # minimum velocity ot be applied to servo
    P_FACTOR = 0.09
    I_FACTOR = 0.0
    D_FACTOR = 0.0018


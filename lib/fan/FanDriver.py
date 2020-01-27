#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   File:       FanDriver.py
#   Author:     Colin Lundquist
#   Date:       1/26/2020
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~

from lib.gpio import IODriver

class Fan:
    
    SPEED_DEFAULT = 0.5 # Default fan speed is half speed
    STATE_DEFAULT = 0   # Fan is off by default
    FAN_IODriver = 15       # GPIO pin
    SPEED_MIN = 0.3     # Minimum fan speed
    SPEED_MAX = 1.0     # Maximum fan speed

    def __init__(self, on=STATE_DEFAULT, speed=SPEED_DEFAULT):

        self.on = on                    # Initialize fan state       
        self.fan_pin = self.FAN_IODriver    # Initialize fan pin

       
        # Only set the speed if it is within limits, else DEFAULT
        if (self.SPEED_MAX >= speed >= self.SPEED_MIN):
            self.speed = speed  # Initialize fan speed
        else:
            self.speed = self.SPEED_DEFAULT
        
        # Set the fan on/off
        if self.on:
            IODriver.pi.set_PWM_dutycycle(self.fan_pin, (self.speed * 0xFF))
        else:
            IODriver.pi.set_PWM_dutycycle(self.fan_pin, 0)


    def set(self, speed=SPEED_DEFAULT, on=STATE_DEFAULT):

        # Only set speed if it is within limits
        if (self.SPEED_MAX >= speed >= self.SPEED_MIN):
            self.speed = speed  # Set new speed, or kee pit the same
        
        self.on = on        # Set new state, or keep it the same

        # Set the fan on/off
        if self.on:
            IODriver.pi.set_PWM_dutycycle(self.fan_pin, (self.speed * 0xFF))
        else:
            IODriver.pi.set_PWM_dutycycle(self.fan_pin, 0)


    def get(self):
        # return a tuple containing stats
        return { "on": self.on, "speed": self.speed } # Return fan info

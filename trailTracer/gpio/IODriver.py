#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#   File:       IODriver.py
#   Author:     Colin Lundquist
#   Date:       1/26/2020
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#~~~~~~~~~~~~~~~~~~
#~~~~~~~

import pigpio

PAN_SERVO_PIN = 22
TILT_SERVO_PIN = 27
#FAN_PIN = 15


pi = pigpio.pi() # Create GPIO object


#pi.set_mode(SENSOR_GPIO_PIN, pigpio.INPUT) # Sensor input
pi.set_mode(PAN_SERVO_PIN, pigpio.OUTPUT)   # Pan servo output
pi.set_mode(TILT_SERVO_PIN, pigpio.OUTPUT)  # Tilt servo output
#pi.set_mode(FAN_PIN, pigpio.OUTPUT)         # Fan control output
#pi.set_mode(SERVO_EN_PIN, pigpio.OUTPUT)    # Servo power control pin
#pi.set_pull_up_down(SERVO_EN_PIN, pigpio.PUD_DOWN)

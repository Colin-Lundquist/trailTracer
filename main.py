from lib.servo import ServoDriver
from lib.gpio import GPIO

# import pigpio

# builtins.pi = pigpio.pi()
# builtins.pi.set_mode(14, pigpio.OUTPUT)


servo = ServoDriver.PanServo()

while True:
    servo.velocity = float(input('Servo Velocity: '))



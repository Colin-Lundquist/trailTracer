import pigpio

pi = pigpio.pi()
serial = pi.serial_open("/dev/ttyS0", 9600)

pi.serial_read(serial, 1)
pi.serial_write(serial, 1)


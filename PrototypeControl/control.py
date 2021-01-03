from gpiozero import DigitalOutputDevice
from time import sleep
from random import choice
import math


ARM_CLOSE = 1
ARM_OPEN = 0

MOTOR_0 = 0
MOTOR_1 = 1

POS_ARM_0 = 0
POS_ARM_1 = 0


# Elbow 400 steps = 90 degree   400/90 (step/degree) = 4.44444444444      degree/step 400

dirs = [DigitalOutputDevice(20), DigitalOutputDevice(2)]
steps = [DigitalOutputDevice(21), DigitalOutputDevice(3)]

[d.off() for d in dirs]
[s.off() for s in steps]


def deg2step(DEG):
	return int(math.ceil(DEG*4.44444444444))


''' Direction off:
0: arms closes (towards starting position)
1: Arm opens '''
def direction(DIRECTION):
	if DIRECTION:
		dirs[0].on()
		dirs[1].off()
	else:
		dirs[0].off()
		dirs[1].on()


def powerMoter(motor, DIRECTION, speed=1000):
	direction(DIRECTION)
	pulse_time = 1/speed
	if motor == 0: # Shoulder movement
		steps[0].on()
		steps[1].on()
		sleep(pulse_time)
		steps[0].off()
		steps[1].off()
		sleep(pulse_time)
	
	else:	  # Elbow Movement
		steps[motor].on()
		sleep(pulse_time)
		steps[motor].off()
		sleep(pulse_time)


def moveToPos(POSE):

#	if POSE[0] = POS_ARM0 and POSE = POS_ARM1: # Already at the correct position

#	if POSE[]


	# POS_ARM_0
	# POS_ARM_1


#motor = 0
#direction = 1 # 0 is forward
#duration = 0.001



	for i in range(deg2step(90)):
		powerMoter(MOTOR_1, ARM_OPEN)
		POS_ARM_0+=1

from gpiozero import DigitalOutputDevice
from time import sleep
from random import choice
import math


class motor_io():
    def __init__(self):
        self.Motor_scaling = 2 # alternative 1

        self.ARM_CLOSE = 1
        self.ARM_OPEN = 0

        self.MOTOR_0 = 0
        self.MOTOR_1 = 1

        self.POS_ARM_0 = 0
        self.POS_ARM_1 = 0


        self.LIMIT_ARM_0 = [-(980)*self.Motor_scaling, 0] # Hard bound range 0-205 degrees (0-800 steps) - but is limited by arm1 config.
        self.LIMIT_ARM_1 = [0, 450*self.Motor_scaling]  # Total range 0-100 degrees - increases with current config of arm0

        # 400 steps = 90 degrees -> steps/degree = 4.44444444444
        
        self.deg_to_step = (400/90)*self.Motor_scaling
        self.degs_per_step = (90/400)*(1/self.Motor_scaling)

        self._dirs = [DigitalOutputDevice(20), DigitalOutputDevice(2)]
        self._steps = [DigitalOutputDevice(21), DigitalOutputDevice(3)]

        self.gripper_open = DigitalOutputDevice(10)


        # send to UR -> pin 11

        [d.off() for d in self._dirs]
        [s.off() for s in self._steps]


    def deg2step(self, deg):
	    return int(math.ceil(deg*self.deg_to_step))

    def set_direction(self, direction, motor):
        if direction == self.ARM_OPEN:
            if motor == self.MOTOR_0:
                self._dirs[0].on()
            else:
                self._dirs[1].off()
        else:
            if motor == self.MOTOR_0:
                self._dirs[0].off()
            else:
                self._dirs[1].on()
    
    def move(self, x=None, y=None, sleeptime=1/500):
        s_time=sleeptime/2
        if s_time < 1/8000:
            s_time = 1/8000
        # pulse_time = 1/(speed*2)
        # print("pulse: ", pulse_time)
        move_y=False
        move_x=False
        if abs(x)> 1 or abs(y)> 1:
            print("Error - To high motor-step-value given - reduce step size or increase Frequency")
        # Update the legal bounds - both if x and/or y is given as input.
        if y==1:
            self.POS_ARM_1 += 1
            self.LIMIT_ARM_0[0] += 1
            self.LIMIT_ARM_0[1] += 1
            self.set_direction(self.ARM_OPEN, self.MOTOR_1)
            move_y = True  
        elif y==-1:
            self.POS_ARM_1 -= 1
            self.LIMIT_ARM_0[0] -= 1
            self.LIMIT_ARM_0[1] -= 1
            self.set_direction(self.ARM_CLOSE, self.MOTOR_1)
            move_y = True
        if x==1:
            self.POS_ARM_0 += 1
            self.LIMIT_ARM_1[0] += 1
            self.LIMIT_ARM_1[1] += 1
            self.set_direction(self.ARM_OPEN, self.MOTOR_0)
            move_x = True
        if x==-1:
            self.POS_ARM_0 -= 1
            self.LIMIT_ARM_1[0] -= 1
            self.LIMIT_ARM_1[1] -= 1
            self.set_direction(self.ARM_CLOSE, self.MOTOR_0)
            move_x = True

        # Check if arm 0 is withn legal bounds (clipped to be within hard-bounds)
        if self.POS_ARM_0 < max(self.LIMIT_ARM_0[0], 0): 
            self.POS_ARM_0 += 1
            self.LIMIT_ARM_1[0] += 1
            self.LIMIT_ARM_1[1] += 1
            print("ERROR - Arm 0 is out of bounds - lower.")
        elif self.POS_ARM_0 > min(self.LIMIT_ARM_0[1], (980)*self.Motor_scaling):
            self.POS_ARM_0 -= 1
            self.LIMIT_ARM_1[0] -= 1
            self.LIMIT_ARM_1[1] -= 1
            print("ERROR - Arm 0 is out of bounds - upper.")
        else:
            # If inside bounds - set arm 0 motor on.
            if move_x:
                self._steps[0].on()

        # Check if arm 1 is within the legal bounds
        if self.POS_ARM_1 < self.LIMIT_ARM_1[0]:
            self.POS_ARM_0 += 1
            self.LIMIT_ARM_1[0] += 1
            self.LIMIT_ARM_1[1] += 1
            print("ERROR - Arm 1 is out of bounds - lower.")
        elif self.POS_ARM_1 > self.LIMIT_ARM_1[1]:
            self.POS_ARM_1 -= 1
            self.LIMIT_ARM_0[0] -= 1
            self.LIMIT_ARM_0[1] -= 1
            print("ERROR - Arm 1 is out of bounds - upper.")
        else:
            # If inside bounds -  set arm 1 motor on
            if move_y:
                self._steps[1].on()

        # Sleep for half the duty cycle - turn off motors, and sleep for rest of duty cycle.
        sleep(s_time)
        self._steps[0].off()
        self._steps[1].off()
        sleep(s_time)

    def print_status(self):
        print(f'Current config: ({self.POS_ARM_0}, {self.POS_ARM_1}). Arm_0 limits: {self.LIMIT_ARM_0}. Arm_1 limits: {self.LIMIT_ARM_1}.')

    def get_pose(self, human_readable=False):
        if human_readable:
            return [self.POS_ARM_0*self.degs_per_step, self.POS_ARM_1*self.degs_per_step-self.POS_ARM_0*self.degs_per_step]
        else:
            return [self.POS_ARM_0*self.degs_per_step, self.POS_ARM_1*self.degs_per_step]

if __name__ == "__main__":
    tester = motor_io()
    tester.move(x=0, y=1)
    tester.print_status()
    tester.move(x=0, y=1)
    tester.print_status()
    tester.move(x=0, y=1)
    tester.print_status()
    tester.move(x=-1, y=0)
    tester.print_status()
    tester.move(x=-1, y=0)
    tester.print_status()
    tester.move(x=-1, y=0)
    tester.print_status()

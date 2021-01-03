from gpiozero import DigitalInputDevice
from motor_io import motor_io
import kinematics
import time

motor = motor_io()

UR_signal = DigitalInputDevice(9)

#a = 1 / 8
#deg_per_step = 1.8 * a

# complete joint limits - (0-220 degrees), (joint 1 values + (0-90 degrees))
def move(q_end_in, duration, frequency=1000, verbose : bool = False):
	n = int(duration * frequency)

	t_start = time.time()

	q_start = motor.get_pose()
	q_end = [q_end_in[0], q_end_in[0]+q_end_in[1]]

	qs = []
	for i in range(n + 1):
		t = i / n

		q_deg = [(q2 * t + q1 * (1 - t)) for q1, q2 in zip(q_start, q_end)]
		q_step = [int(q / motor.degs_per_step) for q in q_deg]
		qs.append(q_step)

	qs_changes = [(i / frequency, [q2 - q1 for q1, q2 in zip(qs[i], qs[i + 1])]) for i in range(n)]

	qs_changes = [(t, [d1, d2]) for t, (d1, d2) in qs_changes if (d1 != 0 or d2 != 0)]

	#print(qs_changes)

	for t, (d1, d2) in qs_changes:
		if verbose:
			print(f"{t:0.2f}s: {d1}, {d2}")

		dt = time.time() - t_start
		motor.move(d1, d2, t - dt)

		# print(f"t: {t:0.2f}, dt: {t:0.2f}, dt-t: {t:0.2f}")
		
		#dt = time.time() - t_start

		#if dt < t:
		#	time.sleep(t - dt)

	#for i, (t, (d1, d2)) in enumerate(qs_changes):
	#	if d1 != 0 or d2 != 0:
	#		t = i / frequency
	#		if verbose:
	#			print(f"{t:0.2f}s: {d1}, {d2}")
	#		motor.move(d1, d2, frequency)
	#	else:
	#		time.sleep(1/frequency)

def test_of_decoupled_joints():
	# test with joint decoupling:
	move([107.5, 0], 2)
	motor.print_status()
	print(motor.get_pose(human_readable=True))
	time.sleep(0.5)
	move([0, 0], 2)
	move([107, 0], 2)
	motor.print_status()
	print(motor.get_pose(human_readable=True))
	time.sleep(0.5)
	move([0, 0], 2)

def circularMove(use_gripper=True):
	# Complete Circular move routine. the time.sleep(1.60) should be replaced with wait until signal from UR
	while True:
		move([0,88],0.75, 8000) #WP 1 - Curl ind, så man ikke rammer muren
		question = input("Enter to continue with circularMove. input \'x\' to quit. \n")
		if question == 'x':
			break

		for i in range(3):
			move([207,88],1.0, 8000) #WP 2 - entry point til pick up rebar
			if use_gripper:
				motor.gripper_open.on() # gør gripper klar til at tage en rebar
			move([220,75],0.5, 8000) #WP 3 - rebar pickup point
			if use_gripper:
				motor.gripper_open.off() # luk gripper for at holde rebar
			# time.sleep(1.60) # DETTE SKAL ERSTATES MED ET WAIT FOR UR SIGNAL... TIMING...
			while not UR_signal.is_active:
				pass 
			time.sleep(0.3)
			move([97.64,52.8],0.25, 8000) #WP - Entry point til placement
			move([68,52.8],0.25, 8000) #WP - Let go of rebar
			if use_gripper:
				motor.gripper_open.on() # åben gripper for at slippe rebar
			move([38.36,52.8],0.25, 8000) #WP - End of placement point
			if use_gripper:
				motor.gripper_open.off() # luk gripper igen...
			move([0,0],0.55, 8000)  # gør igen klar til at curle up
			move([0,88],0.55, 8000)

	move([0,0],0.75, 8000) # skal fjernes.

def linearMove(depth=0.143, length=0.12, steps=1000, letGoStep=450, use_gripper=False):
	kin = kinematics.kinematics(L1=0.055,L2=0.105,offset=-17)
	# Metoden nedenfor laver en list af Q-values, som skal afspilles med jævn fart, så burde C-space linear movement opnåes.
	LinPath = kinematics.MoveL([-length/2, depth],[length/2, depth-0.005],steps)
	EntryPath = kin.invers(-length/2, depth-0.001)
	ExitPath = kin.invers(length/2, depth-0.015)
	count=0
	while True:
		move([0,88],0.75, 8000) #WP 1 - Curl ind, så man ikke rammer muren
		# question = input("Enter to continue with LinearMove. input \'x\' to quit. \n")
		# if question == 'x':
		# 	break
		count+=1
		if count > 5:
			break
		print(count)
		for i in range(1):
			move([207,88],1.0, 8000) #WP 2 - entry point til pick up rebar
			if use_gripper:
				motor.gripper_open.on() # gør gripper klar til at tage en rebar
			move([220,75],0.5, 8000) #WP 3 - rebar pickup point
			if use_gripper:
				motor.gripper_open.off() # luk gripper for at holde rebar
				closed=True
			# time.sleep(0.5) # DETTE SKAL ERSTATES MED ET WAIT FOR UR SIGNAL... TIMING.
			while not UR_signal.is_active:
				pass 
			time.sleep(0.3)
			move(EntryPath,0.25,8000)
			# move([97.64,52.8],0.25, 8000) #WP - Entry point til placement
			for Qs in range(len(LinPath)):
				move(LinPath[Qs],0.0005,80000)
				if use_gripper and Qs > letGoStep and closed==True:
					motor.gripper_open.on()
					closed=False
			move(ExitPath,0.5, 8000)
			if use_gripper:
				motor.gripper_open.off()
				closed=True

			# move([68,52.8],0.125, 8000) #WP - Let go of rebar
			# if use_gripper:
			# 	motor.gripper_open.on() # åben gripper for at slippe rebar
			# move([38.36,52.8],0.125, 8000) #WP - End of placement point
			# if use_gripper:
			# 	motor.gripper_open.off() # luk gripper igen...
			move([0,0],0.55, 8000)  # gør igen klar til at curle up
			move([0,88],0.55, 8000)
	

	move([0,0],0.75, 8000) # skal fjernes.




# circularMove(use_gripper=True)
# depth = push depth, length = length of linear motion, steps = number of steps the linear motion is divided into, legGoStep = step number when the gripper lets go.
linearMove(depth=0.146, length=0.12, steps=800, letGoStep=300, use_gripper=True)

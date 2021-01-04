from gpiozero import DigitalInputDevice
from motor_control import motor_control
import kinematics
import time

motor = motor_control()
UR_signal = DigitalInputDevice(9)

#a = 1 / 8
#deg_per_step = 1.8 * a
# complete joint limits - (0-220 degrees), (joint 1 values + (0-90 degrees))

def moveQ(q_end_in, duration, frequency=8000, verbose : bool = False):
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

	for t, (d1, d2) in qs_changes:
		if verbose:
			print(f"{t:0.2f}s: {d1}, {d2}")
		dt = time.time() - t_start
		motor.StepperMove(d1, d2, t - dt)

def MoveL(start,end,steps,time):
	kin = kinematics.kinematics(L1=0.055,L2=0.105,offset=-17)
	# Metoden nedenfor laver en list af Q-values, som skal afspilles med jævn fart, så burde C-space linear movement opnåes.
	LinPath = kinematics.LinDecomp(start,end,steps)
	for Qs in range(len(LinPath)):
		moveQ(LinPath[Qs],time/steps,8000)

# ----------------- TESTS AND DEMONSTRATIONS BELOW ----------------------------

def test_of_decoupled_joints():
	# test with joint decoupling:
	moveQ([107.5, 0], 2)
	motor.print_status()
	print(motor.get_pose(decoupled=True))
	time.sleep(0.5)
	moveQ([0, 0], 2)
	moveQ([107, 0], 2)
	motor.print_status()
	print(motor.get_pose(decoupled=True))
	time.sleep(0.5)
	moveQ([0, 0], 2)

# Can be used for demonstration in case linear motion fails. -> only uses moveQ commands
def circularMove(use_gripper=True):
	# Complete Circular move routine. the time.sleep(1.60) should be replaced with wait until signal from UR
	while True:
		moveQ([0,88],0.75, 8000) #WP 1 - Curl ind, så man ikke rammer muren
		question = input("Enter to continue with circularMove. input \'x\' to quit. \n")
		if question == 'x':
			break
		for i in range(3):
			moveQ([207,88],1.0, 8000) #WP 2 - entry point til pick up rebar
			if use_gripper:
				motor.gripper_open.on() # gør gripper klar til at tage en rebar
			moveQ([220,75],0.5, 8000) #WP 3 - rebar pickup point
			if use_gripper:
				motor.gripper_open.off() # luk gripper for at holde rebar
			while not UR_signal.is_active:
				pass 
			time.sleep(0.3)
			moveQ([97.64,52.8],0.25, 8000) #WP - Entry point til placement
			moveQ([68,52.8],0.25, 8000) #WP - Let go of rebar
			if use_gripper:
				motor.gripper_open.on() # åben gripper for at slippe rebar
			moveQ([38.36,52.8],0.25, 8000) #WP - End of placement point
			if use_gripper:
				motor.gripper_open.off() # luk gripper igen...
			moveQ([0,0],0.55, 8000)  # gør igen klar til at curle up
			moveQ([0,88],0.55, 8000)

	moveQ([0,0],0.75, 8000) # sætter roboten i home til sidst, såfremt while loopet brydes

def testMoveL():
	moveQ([0,88],0.75, 8000) #WP 1 - Curl ind, så man ikke rammer muren
	kin = kinematics.kinematics(L1=0.055,L2=0.105,offset=-17)
	#brug forward kinematics for at sikre at startpunkt er identisk med nuværende punkt.
	startPosition = kin.forward(motor.get_pose(decoupled=True,degrees=True))
	endPosition = [0.09,0.121]
	moveL(startPosition,endPosition,steps=1000,time=1.2)
	moveQ([0,0],0.75, 8000) # sætter roboten i home til sidst

# Bruges til demonstrationen. I dette tilfælde har vi ikke brugt Forward kinematics som startpunkt i LinDecomp funktionen.
def linearMove(depth=0.143, length=0.12, steps=1000, letGoStep=450, use_gripper=False):
	kin = kinematics.kinematics(L1=0.055,L2=0.105,offset=-17)
	# Metoden nedenfor laver en list af Q-values, som skal afspilles med jævn fart, så burde C-space linear movement opnåes.
	LinPath = kinematics.LinDecomp([-length/2, depth],[length/2, depth-0.005],steps)
	EntryPath = kin.invers(-length/2, depth-0.001)
	ExitPath = kin.invers(length/2, depth-0.015)
	count=0
	while True:
		moveQ([0,88],0.75, 8000) #WP 1 - Curl ind, så man ikke rammer muren
		count+=1
		if count > 5:
			break
		print(count)
		for i in range(1):
			moveQ([207,88],1.0, 8000) #WP 2 - entry point til pick up rebar
			if use_gripper:
				motor.gripper_open.on() # gør gripper klar til at tage en rebar
			moveQ([220,75],0.5, 8000) #WP 3 - rebar pickup point
			if use_gripper:
				motor.gripper_open.off() # luk gripper for at holde rebar
				closed=True
			while not UR_signal.is_active:
				pass 
			time.sleep(0.3)
			moveQ(EntryPath,0.25,8000)
			for Qs in range(len(LinPath)):
				moveQ(LinPath[Qs],0.0005,80000)
				if use_gripper and Qs > letGoStep and closed==True:
					motor.gripper_open.on()
					closed=False
			moveQ(ExitPath,0.5, 8000)
			if use_gripper:
				motor.gripper_open.off()
				closed=True
			moveQ([0,0],0.55, 8000)  # gør igen klar til at curle up
			moveQ([0,88],0.55, 8000)
	
	moveQ([0,0],0.75, 8000) # sætter roboten i home til sidst, såfremt while loopet brydes

# Backup:
# circularMove(use_gripper=True)

# Primary show:
# depth = push depth, length = length of linear motion, steps = number of steps the linear motion is divided into, legGoStep = step number when the gripper lets go.
linearMove(depth=0.146, length=0.12, steps=800, letGoStep=300, use_gripper=True)

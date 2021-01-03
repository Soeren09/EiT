from gpiozero import DigitalOutputDevice
from time import sleep

high = [DigitalOutputDevice(6), DigitalOutputDevice(13)]
low = [DigitalOutputDevice(19), DigitalOutputDevice(26)]

def reset():
	for i in range(2):
		high[i].off()
		low[i].off()

def open():
	reset()
	sleep(0.2)
	high[0].on()
	low[0].on()
	sleep(0.2)
	reset()

def close():
	reset()
	sleep(0.2)
	high[1].on()
	low[1].on()
	sleep(0.2)
	reset()


reset()
print("Reset. Sleeping for 2 seconds.")
sleep(2)
print("Opening fingers.")
open()
print("Fingers open.")

print("Sleeping for 5 seconds.")
sleep(5)

print("Closing fingers.")
close()
print("Fingers closed.")

print("Sleeping for 5 seconds.")
sleep(5)

print("Opening fingers.")
open()
print("Fingers open.")

print("Resetting.")
reset()

print("Terminating program.")


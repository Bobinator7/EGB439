#!usr/bin/env
import time
import math 
import penguinPi as ppi

##INITIALISATION
ppi.init()
#Create our device objects
mA = ppi.Motor(ppi.AD_MOTOR_A)
mB = ppi.Motor(ppi.AD_MOTOR_B)

# mA.get_all()
# mB.get_all()

##CONSTANTS
distance = 1
angle = 90
wheel_dia = 0.065
turn_dia = 0.145
ticks_translate = distance*(360/(3.14*wheel_dia))
ticks_rotate = turn_dia*angle/wheel_dia
motor_L = -50
motor_R = -50
error = 3 

def enc_diff(init_count, current_count):
	half_res = 15360
	full_res = 30720
	scaled_count = current_count - init_count + half_res
	return_count = current_count - init_count
	if (scaled_count < 0):
		return_count = (half_res - init_count) + (current_count + half_res)# TODO (half_res - init_count) + (current_count + half_res) #scaled_count = scaled_count + full_res
	elif (scaled_count >= full_res):
		return_count = (half_res - current_count) + (init_count + half_res) #scaled_count = scaled_count - full_res
	return return_count

def translate(distance, ticks_translate, motor_L, motor_R):
	tick_diffT = 0
	countA_tot = 0
	countB_tot = 0
	epsilon = 40
	limit = 2
	increment = 0.25

	t1A = mA.get_ticks()
	t1B = mA.get_ticks()
	while tick_diffT < ticks_translate:
		mA.set_power(motor_L)
		mB.set_power(motor_R)
		t2A = mA.get_ticks()
		t2B = mA.get_ticks()
		tick_diffT = abs(t2A - t1A)
		countA = enc_diff(t1A, t2A)
		countB = enc_diff(t1B, t2B)

		countA_tot = countA_tot + countA
		countB_tot = countB_tot + countB

		if (abs(countA_tot - countB_tot) > epsilon):
			if (countA_tot < countB_tot):
				if ((motor_L - motor_R) < limit ):
					motor_L = round(motor_L + increment)
			elif (countA_tot > countB_tot):
				if ((motor_R - motor_L) < limit) :
					motor_L = round( motor_L - increment)

	mA.set_power(0)
	mB.set_power(0)

	return

def rotate(angle, ticks_rotate, motor_L, motor_R):
	tick_diffR = 0 
	t3 = mA.get_ticks()
	while tick_diffR < ticks_rotate:
		mA.set_power(motor_L)
		mB.set_power(-motor_R)
		t4 = mA.get_ticks()
		tick_diffR = abs(t4 - t3) + error

	mA.set_power(0)
	mB.set_power(0)
	return

#rotate(angle, ticks_rotate, 50, 50)

for x in range(0, 4):
	translate(1, ticks_translate, motor_L, motor_R)
	time.sleep(0.1)
	rotate(90, ticks_rotate, motor_L, motor_R)
	time.sleep(0.1)
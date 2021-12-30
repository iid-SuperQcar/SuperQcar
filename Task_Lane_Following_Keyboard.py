from Quanser.product_QCar import QCar
from Quanser.q_control import *
from Quanser.q_dp import *
from Quanser.q_interpretation import *
from Quanser.q_misc import *
from Quanser.q_ui import *
from Quanser.q_essential import *
import time
import struct
import numpy as np 
import cv2
import math
import keyboard

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Timing Parameters and methods 
sampleRate = 60
sampleTime = 1/sampleRate
print('Sample Time: ', sampleTime)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Additional parameters
counter = 0
imageWidth = 1640
imageHeight = 820
cameraID = '3'

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
#Setting Filter
steering_filter = Filter().low_pass_first_order_variable(25, 0.033)
next(steering_filter)
dt = 0.033

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Initialize the CSI cameras
myCam = Camera2D(camera_id=cameraID, frame_width=imageWidth, frame_height=imageHeight, frame_rate=sampleRate)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## QCar Initialization
myCar = QCar()

def control_from_keyboard(LB, RT, left_lateral, A):
	left_lateral = -0.0
	A = 0
	if LB == 1:
		if A == 1 :
			throttle_axis = -0.3 * RT #going backward
			steering_axis = left_lateral * 0.5 
		else:
			throttle_axis = 0.3 * RT #going forward
			steering_axis = left_lateral * 0.5
	else:
		throttle_axis = 0 
		steering_axis = 0

	command = np.array([throttle_axis, steering_axis])
	return command

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Main Loop
try:
	while True:
		start = time.time()
		# Capture RGB Image from CSI
		myCam.read()

		# Crop out a piece of the RGB to improve performance
		cropped_rgb = myCam.image_data[524:674, 0:820]

		# Convert to HSV and then threshold it for yellow
		hsv_buf = cv2.cvtColor(cropped_rgb, cv2.COLOR_BGR2HSV)
		binary = binary_thresholding(hsv_buf, lower_bounds=np.array([10, 50, 100]), upper_bounds=np.array([45, 255, 255]))

		# Display the RGB (Original) as well as the Binary in 1/4th resolution for speed
		# cv2.imshow('My RGB image', cv2.resize(myCam.image_data, (410, 205) ) )
		# cv2.imshow('My RGB image', cropped_rgb )
		cv2.imshow('My Binary image', cv2.resize(binary, (410, 75) ))

		# Find slope and intercept of linear fit from the binary image
		slope, intercept = find_slope_intercept_from_binary(binary)

		# steering from slope and intercept
		raw_steering = 1.5*(slope - 0.3419) + (1/150)*(intercept+5)
		steering = steering_filter.send((saturate(raw_steering, 0.5, -0.5), dt))

		keyC = 0
		keyX = 0
		if keyboard.is_pressed('c'):
			keyC = 1
		if keyboard.is_pressed('x'):
			keyX = 1
		# Write steering to qcar
		# control_from_gamepad(0/1, 0.35, -0.0, 0)
		mtr_cmd = control_from_keyboard(keyC, 0.35, -0.0, 0)
		if keyX == 1:
			if math.isnan(steering):
				mtr_cmd[1] = 0
			else:
				mtr_cmd[1] = steering
			mtr_cmd[0] = mtr_cmd[0]*np.cos(steering)
		myCar.write_mtrs(mtr_cmd)
		cv2.waitKey(1)
		end = time.time()
		dt = end - start

except KeyboardInterrupt:
	print("User interrupted!")

finally:
	# Terminate camera and QCar
	myCam.terminate()
	myCar.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 

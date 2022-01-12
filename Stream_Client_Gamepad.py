from Quanser.q_essential import Camera2D
from Quanser.q_misc import BasicStream 
from Quanser.q_ui import *

import time 
import cv2
import numpy as np

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Image parameters
gpad = gamepadViaTarget(1)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Create a BasicStream object configured as a Client agent, with buffer sizes enough to send/receive the image above.
myClient = BasicStream('tcpip://10.19.136.61:18005', agent='c')
prev_con = False

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Timing Parameters and methods 
startTime = time.time()
def elapsed_time():
    return time.time() - startTime

sampleRate = 30.0
sampleTime = 1/sampleRate
simulationTime = 500.0
print('Sample Time: ', sampleTime)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Initialize the CSI cameras
# myCam1 = Camera2D(camera_id="0", frame_width=imageWidth, frame_height=imageHeight, frame_rate=sampleRate)


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Main Loop
try:
    while elapsed_time() < simulationTime:
        if not myClient.connected:
            myClient.checkConnection()
        
        if myClient.connected and not prev_con:
            print('Connection to Server was successful.')
            prev_con = myClient.connected
            continue

        if myClient.connected:

            # Start timing this iteration
            start = time.time()

            # Capture RGB Image from CSI
            gpad.read()

            # Send data to server after converting image to float32 data type
            # bytes_sent = myClient.send( np.array( myCam1.image_data, dtype=np.float32 )/255 )
            bytes_sent = myClient.send(np.array([gpad.X, gpad.LB, gpad.RT], dtype=np.uint8))

            # End timing this iteration
            end = time.time()

            # Calculate the computation time, and the time that the thread should pause/sleep for
            computationTime = end - start
            sleepTime = sampleTime - ( computationTime % sampleTime )

            # Pause/sleep for sleepTime in milliseconds
            msSleepTime = int(1000*sleepTime)
            if msSleepTime <= 0:
                msSleepTime = 1 # this check prevents an indefinite sleep as cv2.waitKey waits indefinitely if input is 0
            
            # cv2.imshow('Client Image Captured', myCam1.image_data)
            cv2.waitKey(msSleepTime)
        
except KeyboardInterrupt:
    print("User interrupted!")

finally:
    # Terminate Webcam    
    # myCam1.terminate()
    gpad.terminate()
    
    # Terminate Client
    myClient.terminate()

    print('All the right turns in all the right places.')

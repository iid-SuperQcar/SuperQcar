from Quanser.q_misc import BasicStream 
import time 
import numpy as np
import cv2

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Image Parameters

gamepad_data = np.array([0,0], dtype = np.uint8)

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
# Create a BasicStream object configured as a Server agent, with buffer sizes enough to send/receive the image above.
myServer = BasicStream('tcpip://10.19.136.61:18005', agent='s')
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
# Main Loop
try:
    while elapsed_time() < simulationTime:

        if not myServer.connected:
            myServer.checkConnection()

        if myServer.connected and not prev_con:
            print('Connection to Client was successful.')
        prev_con = myServer.connected

        if myServer.connected:

            # Start timing this iteration
            start = time.time()    

            # Receive data from client 
            gamepad_data, bytes_received = myServer.receive(gamepad_data)
            
            if bytes_received < len(gamepad_data.tobytes()):
                print('Client stopped sending data over.')
                break
            # End timing this iteration
            end = time.time()

            # Calculate the computation time, and the time that the thread should pause/sleep for
            computationTime = end - start
            sleepTime = sampleTime - ( computationTime % sampleTime )

            # Pause/sleep for sleepTime in milliseconds
            msSleepTime = int(1000*sleepTime)
            if msSleepTime <= 0:
                msSleepTime = 1 # this check prevents an indefinite sleep as cv2.waitKey waits indefinitely if input is 0
            
            print(gamepad_data)
            #cv2.waitKey(msSleepTime)

except KeyboardInterrupt:
    print("User interrupted!")

finally:
    # Terminate Server
    myServer.terminate()
    print('All the right turns in all the right places.')





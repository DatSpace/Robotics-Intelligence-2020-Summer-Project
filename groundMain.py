#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
# Miguel Cadenas is not who he thinks he is
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.

import sim
import cv2
import time
import numpy as np
import droneMain as drone
import multiprocessing

# Program Constants
SIMULATION_STEP = 50.0  # in milliseconds

# Program Variables
leftMotorFSpeed = 0.0
rightMotorFSpeed = 0.0
leftMotorBSpeed = 0.0
rightMotorBSpeed = 0.0
L0Speed = 0.0
L1Speed = 0.0
L2Speed = 0.0
L3Speed = 0.0
F1Speed = 0.0
F2Speed = 0.0
leftWheelOdom = 0.0
rightWheelOdom = 0.0
lastLeftWheelPosition = 0.0
lastRightWheelPosition = 0.0
MrY = 0
kernel = np.ones((5, 5), np.float32)/25
position = 0


if __name__ == "__main__":

    # Creating process and queue...
    drone_queue = multiprocessing.Queue()
    drone_process = multiprocessing.Process(
        target=drone.main, args=(drone_queue,))

    # Starting drone process
    drone_process.start()

    # Start Program and just in case, close all opened connections
    print('Program started...')
    sim.simxFinish(-1)

    # Connect to simulator running on localhost
    # V-REP runs on port 19997, a script opens the API on port 19999
    clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

    # Connect to the simulation
    if clientID != -1:
        print('Connected to remote API server...')
        print('Obtaining handles of simulation objects...')

        # Get handles to simulation objects
        print('Obtaining handles of simulation objects')
        # Vision and proximity sensors in Hand
        res, camera = sim.simxGetObjectHandle(
            clientID, 'Camera', sim.simx_opmode_oneshot_wait)
        res, distance = sim.simxGetObjectHandle(
            clientID, 'Proximity', sim.simx_opmode_oneshot_wait)
        # Vision and proximity sensors in car
        res, cameraCar = sim.simxGetObjectHandle(
            clientID, 'CameraCar', sim.simx_opmode_oneshot_wait)
        res, distanceCar = sim.simxGetObjectHandle(
            clientID, 'InPlace', sim.simx_opmode_oneshot_wait)
        #res,ForceHand = sim.simxGetObjectHandle(clientID, 'Force', sim.simx_opmode_oneshot_wait)
        # Wheel drive motors
        res, leftMotorF = sim.simxGetObjectHandle(
            clientID, 'MotorA_FL', sim.simx_opmode_oneshot_wait)
        res, leftMotorB = sim.simxGetObjectHandle(
            clientID, 'MotorB_BL', sim.simx_opmode_oneshot_wait)
        res, rightMotorF = sim.simxGetObjectHandle(
            clientID, 'MotorA_FR', sim.simx_opmode_oneshot_wait)
        res, rightMotorB = sim.simxGetObjectHandle(
            clientID, 'MotorB_BR', sim.simx_opmode_oneshot_wait)
        # Wheels
        res, leftWheelF = sim.simxGetObjectHandle(
            clientID, 'WheelA_FL', sim.simx_opmode_oneshot_wait)
        res, leftWheelB = sim.simxGetObjectHandle(
            clientID, 'WheelB_BL', sim.simx_opmode_oneshot_wait)
        res, rightWheelF = sim.simxGetObjectHandle(
            clientID, 'WheelA_FR', sim.simx_opmode_oneshot_wait)
        res, rightWheelB = sim.simxGetObjectHandle(
            clientID, 'WheelB_BL', sim.simx_opmode_oneshot_wait)
        # RobotArm
        res, L0 = sim.simxGetObjectHandle(
            clientID, 'L0', sim.simx_opmode_oneshot_wait)
        res, L1 = sim.simxGetObjectHandle(
            clientID, 'L1', sim.simx_opmode_oneshot_wait)
        res, L2 = sim.simxGetObjectHandle(
            clientID, 'L2', sim.simx_opmode_oneshot_wait)
        res, L3 = sim.simxGetObjectHandle(
            clientID, 'L3', sim.simx_opmode_oneshot_wait)
        # Gripper
        res, F1 = sim.simxGetObjectHandle(
            clientID, 'Barrett_openCloseJoint0', sim.simx_opmode_oneshot_wait)
        if res != sim.simx_return_ok:
            print('Could not get handle to F1')
        res, F2 = sim.simxGetObjectHandle(
            clientID, 'Barrett_openCloseJoint', sim.simx_opmode_oneshot_wait)
        if res != sim.simx_return_ok:
            print('Could not get handle to F2')
        # Body
        res, body = sim.simxGetObjectHandle(
            clientID, 'Robot', sim.simx_opmode_oneshot_wait)
        # Floor
        res, floor = sim.simxGetObjectHandle(
            clientID, 'Floor', sim.simx_opmode_oneshot_wait)

        # Start main control loop
        print('Starting control loop...')
        res, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera, 0, sim.simx_opmode_streaming)
        res, resolutionCar, imageCar = sim.simxGetVisionSensorImage(
            clientID, cameraCar, 0, sim.simx_opmode_streaming)
        # While connected to the simulator
        while (sim.simxGetConnectionId(clientID) != -1):
            start_ms = int(round(time.time() * 1000))

            res, resolution, image = sim.simxGetVisionSensorImage(
                clientID, camera, 0, sim.simx_opmode_buffer)
            res, resolutionCar, imageCar = sim.simxGetVisionSensorImage(
                clientID, cameraCar, 0, sim.simx_opmode_buffer)

            # the position values are rounded to just the integer number

            if res == sim.simx_return_ok:
                #Camera in Hand
                original = np.array(image, dtype=np.uint8)
                original.resize([resolution[0], resolution[1], 3])
                original = cv2.flip(original, 0)
                original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
                green = cv2.inRange(original, (0, 100, 0), (32, 255, 255))
                cv2.imshow('View in Hand', green)
                #Camera in Car
                originalCar = np.array(imageCar, dtype=np.uint8)
                originalCar.resize([resolutionCar[0], resolutionCar[1], 3])
                originalCar = cv2.flip(originalCar, 0)
                originalCar = cv2.cvtColor(originalCar, cv2.COLOR_RGB2BGR)
                greenCar = cv2.inRange(
                    originalCar, (0, 100, 0), (32, 255, 255))

                position = sim.simxGetObjectPosition(
                    clientID, body, floor, sim.simx_opmode_oneshot)[1]
                res, LD0 = sim.simxGetJointPosition(
                    clientID, L0, sim.simx_opmode_oneshot_wait)
                res, LD1 = sim.simxGetJointPosition(
                    clientID, L1, sim.simx_opmode_oneshot_wait)
                res, LD2 = sim.simxGetJointPosition(
                    clientID, L2, sim.simx_opmode_oneshot_wait)
                res, i1, i2, i3, i4 = sim.simxReadProximitySensor(
                    clientID, distance, sim.simx_opmode_oneshot_wait)
                res, c1, c2, c3, c4 = sim.simxReadProximitySensor(
                    clientID, distanceCar, sim.simx_opmode_oneshot_wait)
                # Moments

                MCar = cv2.moments(greenCar)
                if MCar["m00"] != 0:
                    cXCar = int(MCar["m10"] / MCar["m00"])
                    cYCar = int(MCar["m01"] / MCar["m00"])
                else:
                    # If there are no green color a 0 will the value be
                    cXCar, cYCar = 0, 0
                cv2.circle(greenCar, (cXCar, cYCar), 5, (100, 155, 155), -1)
                ci2 = np.around(i2, 2)
                ci3 = np.around(i3, 2)
                ci4 = np.around(i4, 2)
                cc2 = np.around(i4, 2)
                cLD0 = LD0/.0166667
                cLD1 = LD1/.0166667
                cLD2 = LD2/.0166667
                ccLD0 = np.around(cLD0, 1)
                ccLD1 = np.around(cLD1, 1)
                ccLD2 = np.around(cLD2, 1)
                #print(i1, ci2, ci3, ci4)
                if (ccLD0 > -52 and MrY == 0):
                    L0Speed = -0.5
                    if (ccLD2 < 60):
                        L2Speed = 0.5
                    else:
                        L2Speed = 0
                else:
                    L0Speed = 0
                if(ccLD0 < -45 and ccLD2 > 60 and ci2[2] < 0.1 and i1 == True):
                    F1Speed = -0.5
                    F2Speed = -0.5
                    MrY = 1
                if (MrY == 1):
                    L0Speed = 0
                    L1Speed = 0
                    L2Speed = 0
                    if (ccLD1 < 170):
                        L1Speed = 0.2
                    if(ccLD0 < -15):
                        L0Speed = 0.5
                if (ccLD0 > -15 and ccLD1 > 170 and ccLD2 > -8 and MrY == 1):
                    L2Speed = -0.5

                if (cXCar > 0.1 and cYCar > 0.1 and c1 == True):
                    print('GO', MrY)
                else:
                    print('Wait', MrY)

                print('Arm Joints Position', ccLD0, ccLD1, ccLD2)
                print("Proximity Sensor", ci2[2], c1)
                print(cXCar, cYCar)

                cv2.imshow('View in Car', greenCar)
            elif res == sim.simx_return_novalue_flag:
                # Camera has not started or is not returning images
                pass
            else:
                # Something else has happened
                print("Unexpected error returned", res)

            # Get wheel odometry directly from rotary joints in radians
            leftWheelDiameter = \
                sim.simxGetObjectFloatParameter(clientID, leftWheelF, 18, sim.simx_opmode_oneshot)[1] \
                - sim.simxGetObjectFloatParameter(clientID,
                                                  leftWheelF, 15, sim.simx_opmode_oneshot)[1]
            rightWheelDiameter = \
                sim.simxGetObjectFloatParameter(clientID, rightWheelF, 18, sim.simx_opmode_oneshot)[1] \
                - sim.simxGetObjectFloatParameter(clientID,
                                                  rightWheelF, 15, sim.simx_opmode_oneshot)[1]
            leftWheelPosition = sim.simxGetJointPosition(
                clientID, leftMotorF, sim.simx_opmode_oneshot)[1]
            rightWheelPosition = sim.simxGetJointPosition(
                clientID, rightMotorF, sim.simx_opmode_oneshot)[1]
            # Deal with the joint singularity
            dTheta = leftWheelPosition - lastLeftWheelPosition
            if dTheta > np.pi:
                dTheta -= 2*np.pi
            elif dTheta < -np.pi:
                dTheta += 2*np.pi
            leftWheelOdom += dTheta * leftWheelDiameter / 2
            lastLeftWheelPosition = leftWheelPosition
            dTheta = rightWheelPosition - lastRightWheelPosition
            if dTheta > np.pi:
                dTheta -= 2*np.pi
            elif dTheta < -np.pi:
                dTheta += 2*np.pi
            rightWheelOdom += dTheta * rightWheelDiameter / 2
            lastRightWheelPosition = rightWheelPosition

            # Place your Mobile Robot Control code here

            # Read keypresses for external control (note you must have the OpenCV image window selected when pressing keys!)
            # I left this because it is not really an obstacle
            # will read a value of 255 if no key is pressed
            keypress = cv2.waitKey(1) & 0xFF
            if keypress == ord(' '):
                leftMotorFSpeed = 0.0
                rightMotorFSpeed = 0.0
                leftMotorBSpeed = 0.0
                rightMotorBSpeed = 0.0
            elif keypress == ord('w'):
                leftMotorFSpeed = -5
                rightMotorFSpeed = -5
                leftMotorBSpeed = -5
                rightMotorBSpeed = -5
            elif keypress == ord('s'):
                leftMotorFSpeed = 5
                rightMotorFSpeed = 5
                leftMotorBSpeed = 5
                rightMotorBSpeed = 5
            elif keypress == ord('a'):
                leftMotorFSpeed = 2
                rightMotorFSpeed = -2
                leftMotorBSpeed = 2
                rightMotorBSpeed = -2
            elif keypress == ord('d'):
                leftMotorFSpeed = -2
                rightMotorFSpeed = 2
                leftMotorBSpeed = -2
                rightMotorBSpeed = 2
            elif keypress == ord('1'):
                L0Speed = 5
            elif keypress == ord('2'):
                L0Speed = 0
            elif keypress == ord('3'):
                L0Speed = -5
            elif keypress == ord('4'):
                L1Speed = -5
            elif keypress == ord('5'):
                L1Speed = 0
            elif keypress == ord('6'):
                L1Speed = 5
            elif keypress == ord('7'):
                L2Speed = -5
            elif keypress == ord('8'):
                L2Speed = 0
            elif keypress == ord('9'):
                L2Speed = 5
            elif keypress == ord('i'):
                L3Speed = -5
            elif keypress == ord('o'):
                L3Speed = 0
            elif keypress == ord('p'):
                L3Speed = 5
            elif keypress == ord('j'):
                F1Speed = -1
                F2Speed = -1
            elif keypress == ord('k'):
                F1Speed = -0
                F2Speed = -0
            elif keypress == ord('l'):
                F1Speed = 1
                F2Speed = 1

            elif keypress == ord('x'):
                break

            # Set actuators on mobile robot
            sim.simxSetJointTargetVelocity(
                clientID, leftMotorF, leftMotorFSpeed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, rightMotorF, rightMotorFSpeed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, leftMotorB, leftMotorBSpeed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, rightMotorB, rightMotorBSpeed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, L0, L0Speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, L1, L1Speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, L2, L2Speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, L3, L3Speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, F1, F1Speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, F2, F2Speed, sim.simx_opmode_oneshot)

            end_ms = int(round(time.time() * 1000))
            dt_ms = end_ms - start_ms
            sleep_time = SIMULATION_STEP-dt_ms
            if (sleep_time > 0.0):
                time.sleep(sleep_time/1000.0)

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)
        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
        cv2.destroyAllWindows()
    else:
        print('Failed connecting to remote API server...')

    # Wait until both processes have finished
    drone_queue.close()
    drone_queue.join_thread()
   # drone_process.join()

    # Both processes finished
    print('Simulation has ended...')

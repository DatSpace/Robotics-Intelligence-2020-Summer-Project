#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sim
import cv2
import time
import numpy as np
import droneMain as drone
import multiprocessing
from enum import Enum, auto


class RobotState(Enum):
    TRAVELLING = auto()
    SEARCHING = auto()
    PICKING = auto()
    RETURNING = auto()


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
MrY = 2
kernel = np.ones((5, 5), np.float32)/25
position = 0
path = []
robot_state = RobotState.TRAVELLING


def speedController(clientID, leftMotorF, rightMotorF, leftMotorB, rightMotorB, error):
    gain = 2.0
    default_speed = 2.0

    delta = gain*error

    left_speed = default_speed - delta
    right_speed = default_speed + delta

    sim.simxSetJointTargetVelocity(
        clientID, leftMotorF, left_speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotorF, right_speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, leftMotorB, left_speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotorB, right_speed, sim.simx_opmode_oneshot)


def getOrientationError(clientID, body, target_orientation):
    # Get the current euler angles for the orientation of the robot to the floor
    orientation = (sim.simxGetObjectOrientation(
        clientID, body, -1, sim.simx_opmode_oneshot)[1])[2]

    orientation_error = orientation - target_orientation

    # print(orientation)
    in_min = -np.pi
    in_max = np.pi
    out_min = -1.0
    out_max = 1.0
    return (orientation_error - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def pickBear():
    res, resolution, image = sim.simxGetVisionSensorImage(
        clientID, camera, 0, sim.simx_opmode_buffer)
    res, resolutionCar, imageCar = sim.simxGetVisionSensorImage(
        clientID, cameraCar, 0, sim.simx_opmode_buffer)

    # the position values are rounded to just the integer number

    # Camera in Hand
    original = np.array(image, dtype=np.uint8)
    original.resize([resolution[0], resolution[1], 3])
    original = cv2.flip(original, 0)
    original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
    green = cv2.inRange(original, (0, 100, 0), (32, 255, 255))

    # Camera in Car
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
    res, fd1, fd2, fd3, fd4 = sim.simxReadProximitySensor(
        clientID, FrontDistance, sim.simx_opmode_oneshot_wait)
    # Moments in the car
    MCar = cv2.moments(greenCar)
    if MCar["m00"] != 0:
        cXCar = int(MCar["m10"] / MCar["m00"])
        cYCar = int(MCar["m01"] / MCar["m00"])
    else:
        # If there are no green color a 0 will the value be
        cXCar, cYCar = 0, 0
    cv2.circle(greenCar, (cXCar, cYCar), 5, (100, 155, 155), -1)
    # Moments in the hand
    MHand = cv2.moments(green)
    if MHand["m00"] != 0:
        cXHand = int(MHand["m10"] / MHand["m00"])
        cYHand = int(MHand["m01"] / MHand["m00"])
    else:
        # If there are no green color a 0 will the value be
        cXHand, cYHand = 0, 0
    cv2.circle(green, (cXHand, cYHand), 5, (100, 155, 155), -1)
    ci2 = np.around(i2, 2)
    cfd2 = np.around(fd2, 2)
    # Robotic Arm Joints DOF adjusted to 90
    cLD0 = LD0/.0166667
    cLD1 = LD1/.0166667
    cLD2 = LD2/.0166667
    ccLD0 = np.around(cLD0, 1)
    ccLD1 = np.around(cLD1, 1)
    ccLD2 = np.around(cLD2, 1)
    #    Set Arm to initial Position
    if MrY == 0:
        if ccLD1 < 155:
            L1Speed = 0.5
        else:
            L1Speed = 0.0
            MrY = 0.5
    elif MrY == 0.5:
        if ccLD1 > 0:
            L1Speed = -0.2
        else:
            L1Speed == 0.0
            MrY = 1
    #   Fine adjustment looking for MrYork around the Manta
    elif MrY == 1:
        L0Speed = 0
        L1Speed = 0
        L2Speed = 0
        # No MrY
        if cXHand < 1:
            leftMotorFSpeed = 1.5
            rightMotorFSpeed = -1.5
            leftMotorBSpeed = 1.5
            rightMotorBSpeed = -1.5
        # Centering momentum in X
        elif 1 < cXHand < 105:
            leftMotorFSpeed = 0.8
            rightMotorFSpeed = -0.8
            leftMotorBSpeed = 0.8
            rightMotorBSpeed = -0.8
        elif cXHand > 145:
            leftMotorFSpeed = -0.8
            rightMotorFSpeed = 0.8
            leftMotorBSpeed = -0.8
            rightMotorBSpeed = 0.8
        # Centering momentun in Y
        elif 1 < cYHand < 90:
            position4 = (cYHand-155)
            delta4 = 2*position4/100
            leftMotorFSpeed = -(0.8-delta4)
            rightMotorFSpeed = -(0.8-delta4)
            leftMotorBSpeed = -(0.8-delta4)
            rightMotorBSpeed = -(0.8-delta4)
        elif cYHand > 135:
            leftMotorFSpeed = 0.8
            rightMotorFSpeed = 0.8
            leftMotorBSpeed = 0.8
            rightMotorBSpeed = 0.8
        # The GroundRobot is ready to deploy
        else:
            leftMotorFSpeed = 0
            rightMotorFSpeed = 0
            leftMotorBSpeed = 0
            rightMotorBSpeed = 0
            MrY = 2
    #   Deploying the arm towards MrYork
    elif MrY == 2:
        if (ccLD2 < 135):
            L2Speed = 0.3
        else:
            L2Speed = 0
        if ccLD0 > -90:
            L0Speed = -0.2
        else:
            L0Speed = 0
            MrY = 3
    # Super Fine adjustment
    elif MrY == 3:
        L0Speed = 0
        L1Speed = 0
        L2Speed = 0
    # Closing the fingers
    elif MrY == 4:
        L0Speed = 0
        L1Speed = 0
        L2Speed = 0
        if(ccLD0 < -35 and ccLD2 > 40):
            F1Speed = -0.5
            F2Speed = -0.5
            MrY = 4
    #   Folding the arm back to the car with Mr York grabbed
    elif (MrY == 4):
        L0Speed = 0
        L1Speed = 0
        L2Speed = 0
        F1Speed = -0.5
        F2Speed = -0.5
        if(ccLD0 < -15):
            L0Speed = 0.3
        if (ccLD1 < 170):
            L1Speed = 0.3
        if(ccLD2 > -10):
            L2Speed = -0.1
        else:
            L0Speed = 0
            L1Speed = 0
            L2Speed = 0
            MrY = 5
    elif (MrY == 5):
        L0Speed = 0
        L1Speed = 0
        L2Speed = 0
        F1Speed = -0.5
        F2Speed = -0.5
        if (cXCar > 0.1 and cYCar > 0.1 and c1 == True):
            print('GO', MrY)
    print('Arm Joints Position', ccLD0, ccLD1, ccLD2)
    # print("Proximity Sensor", ci2[2], c1)
    print('MrY Stage', MrY, ci2[2])
    print("M in Hand", cXHand, cYHand)
    # print('Front Sensor:',cfd2[2])
    cv2.imshow('View in Car', greenCar)
    cv2.imshow('View in Hand', green)

    # Set actuators on mobile robot
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

    cv2.waitKey(1)


if __name__ == "__main__":

    # Creating process and queue...
    drone_queue = multiprocessing.Queue()
    drone_process = multiprocessing.Process(
        target=drone.main, args=(drone_queue,))

    # Starting drone process
    # drone_process.start()

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
        res, FrontDistance = sim.simxGetObjectHandle(
            clientID, 'FrontSensor', sim.simx_opmode_oneshot_wait)
        # res,ForceHand = sim.simxGetObjectHandle(clientID, 'Force', sim.simx_opmode_oneshot_wait)
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
            clientID, 'ResizableFloor_5_25', sim.simx_opmode_oneshot_wait)

        # Start main control loop
        print('Starting control loop...')
        res, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera, 0, sim.simx_opmode_streaming)
        res, resolutionCar, imageCar = sim.simxGetVisionSensorImage(
            clientID, cameraCar, 0, sim.simx_opmode_streaming)
        # While connected to the simulator
        while (sim.simxGetConnectionId(clientID) != -1):
            start_ms = int(round(time.time() * 1000))

            if not path:  # If path list is empty, wait to get a response
                path = drone_queue.get()
                print(path)

            if (robot_state == RobotState.TRAVELLING):
                getOrientationError(clientID, body, 3.1415)
                speedController(clientID, leftMotorF, rightMotorF,
                                leftMotorB, rightMotorB, 1.5)
            elif(robot_state == RobotState.SEARCHING):
                pass
            elif(robot_state == RobotState.PICKING):
                pickBear()
            elif(robot_state == RobotState.RETURNING):
                pass

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

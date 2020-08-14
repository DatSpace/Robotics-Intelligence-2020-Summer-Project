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


class ArmState(Enum):
    RETRACT = auto()
    EXTENT = auto()
    GRAB = auto()
    WAIT = auto()


# Program Constants
SIMULATION_STEP = 50.0  # in milliseconds

# Program Variables
leftMotorFSpeed = 0.0
rightMotorFSpeed = 0.0
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
MrY = 200
kernel = np.ones((5, 5), np.float32)/25
position = 0
path = []
robot_state = RobotState.TRAVELLING
arm_state = ArmState.RETRACT
robot_path_index = 0


def speedController(clientID, leftMotorF, rightMotorF, error):
    gain = 5.0
    default_speed = -1.0

    delta = gain*error

    left_speed = default_speed - delta
    right_speed = default_speed + delta

    sim.simxSetJointTargetVelocity(
        clientID, leftMotorF, left_speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotorF, right_speed, sim.simx_opmode_oneshot)


def getOrientationError(clientID, body, target_orientation):
    # Get the current euler angles for the orientation of the robot
    orientation = (sim.simxGetObjectOrientation(
        clientID, body, -1, sim.simx_opmode_oneshot)[1])[2]

    orientation_error = orientation - target_orientation

    in_min = -np.pi
    in_max = np.pi
    out_min = -1.0
    out_max = 1.0
    return (orientation_error - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def getTargetOrientation(current_point, next_point):
    #print("Target orientation is:", np.arctan2(next_point[1]-current_point[1], next_point[0]-current_point[0]))
    # Angle of the two points in radians
    return np.arctan2(next_point[1]-current_point[1], next_point[0]-current_point[0])


def updateRobotPathIndex(clientID, robot, path, robot_path_index, radius):
    current_point = path[robot_path_index]

    result, current_robot_location = sim.simxGetObjectPosition(
        clientID, robot, -1, sim.simx_opmode_oneshot)
    while (result != sim.simx_return_ok):
        result, current_robot_location = sim.simxGetObjectPosition(
            clientID, robot, -1, sim.simx_opmode_oneshot)

    print("Index is:", robot_path_index)
    print("Current is:", current_point)
    print("Location is:", current_robot_location)
    if (robot_path_index < len(path) - 1):
        next_point = path[robot_path_index + 1]
        if (np.sqrt(((next_point[0] - current_robot_location[0]) ** 2.0) + ((next_point[1] - current_robot_location[1]) ** 2.0)) < radius):
            robot_path_index += 1
        return robot_path_index, current_point, next_point
    return robot_path_index, current_point, [None, None]


def holdBear():
    L0Speed = 0
    L1Speed = 0
    L2Speed = 0
    F1Speed = -0.5
    F2Speed = -0.5


def searchBear(arm_state, robot_state):
    if (arm_state == ArmState.EXTENT):
        if ccLD1 > 90:
            L1Speed = -0.5
        elif 90 > ccLD1 > 0:
            L1Speed = -0.1
        else:
            L1Speed == 0.0
            arm_state = ArmState.WAIT
    elif (arm_state == ArmState.WAIT):
        #   Fine adjustment looking for MrYork around the Manta
        L0Speed = 0
        L1Speed = 0
        L2Speed = 0
        # No MrY
        if cXHand < 1:
            leftMotorFSpeed = 1.5
            rightMotorFSpeed = -1.5
        # Centering momentum in X
        elif 1 < cXHand < 105:
            leftMotorFSpeed = 0.4
            rightMotorFSpeed = -0.4
        elif cXHand > 145:
            leftMotorFSpeed = -0.8
            rightMotorFSpeed = 0.8
        # Centering momentun in Y
        elif 1 < cYHand < 65:
            position4 = (cYHand-155)
            delta4 = 2*position4/100
            leftMotorFSpeed = -(0.8-delta4)
            rightMotorFSpeed = -(0.8-delta4)
        elif cYHand > 100:
            leftMotorFSpeed = 0.8
            rightMotorFSpeed = 0.8
        # The GroundRobot is ready to deploy
        else:
            leftMotorFSpeed = 0
            rightMotorFSpeed = 0
            arm_state = ArmState.GRAB
            robot_state = RobotState.PICKING
    return arm_state, robot_state


def pickBear(clientID, arm_state, camera, cameraCar, body, L0, L1, L2, distance, distanceCar, FrontDistance):
    res, resolution, image = sim.simxGetVisionSensorImage(
        clientID, camera, 0, sim.simx_opmode_buffer)
    res, resolutionCar, imageCar = sim.simxGetVisionSensorImage(
        clientID, cameraCar, 0, sim.simx_opmode_buffer)

    #Camera in Hand
    original = np.array(image, dtype=np.uint8)
    original.resize([resolution[0], resolution[1], 3])
    original = cv2.flip(original, 0)
    original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
    green = cv2.inRange(
        original, (0, 100, 0), (32, 255, 255))
    #Camera in Car
    originalCar = np.array(imageCar, dtype=np.uint8)
    originalCar.resize([resolutionCar[0], resolutionCar[1], 3])
    originalCar = cv2.flip(originalCar, 0)
    originalCar = cv2.cvtColor(originalCar, cv2.COLOR_RGB2BGR)
    greenCar = cv2.inRange(originalCar, (0, 100, 0), (32, 255, 255))

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
    if arm_state == ArmState.RETRACT:
        if ccLD1 < 155:
            L1Speed = 1.0
        else:
            L1Speed = 0.0
            MrY = 0.5

    #   Deploying the arm towards MrYork
    if (arm_state=ArmState.GRAB):
        if (ccLD2 < 125):
            L2Speed = 0.2
        else:
            L2Speed = 0
        if ccLD0 > -83:
            L0Speed = -0.07
        else:
            # Super Fine adjustment to center
            L0Speed = 0
            L1Speed = 0
            L2Speed = 0
            if 1 < cXHand < 95:
                leftMotorFSpeed = 0.1
                rightMotorFSpeed = -0.1
            elif cXHand > 165:
                leftMotorFSpeed = -0.1
                rightMotorFSpeed = 0.1
            else:
                # Super Fine adjustment to reach MrY
                leftMotorFSpeed = 0.0
                rightMotorFSpeed = 0.0
                if (i1 == True and ci2[2] > 0.03):
                    leftMotorFSpeed = -0.08
                    rightMotorFSpeed = -0.08
                else:
                    if(ccLD0 < -35 and ccLD2 > 40):
                        F1Speed = -0.5
                        F2Speed = -0.5
                        arm_state = ArmState.RETRACT
    #   Folding the arm back to the car with Mr York grabbed
    elif (arm_state == ArmState.RETRACT):
        L0Speed = 0
        L1Speed = 0
        L2Speed = 0
        F1Speed = -0.5
        F2Speed = -0.5
        if(ccLD0 < -25):
            L0Speed = 0.3
        if (ccLD1 < 170):
            L1Speed = 0.3
        if(ccLD2 > -10):
            L2Speed = -0.1
        else:
            L0Speed = 0
            L1Speed = 0
            L2Speed = 0
            arm_state = ArmState.WAIT
            robot_state = RobotState.RETURNING

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
    return arm_state, robot_state


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

        # Get handles to simulation objects
        print('Obtaining handles of simulation objects')

        # Vision and proximity sensors in Hand
        camera = sim.simxGetObjectHandle(
            clientID, 'Camera', sim.simx_opmode_oneshot_wait)[1]
        distance = sim.simxGetObjectHandle(
            clientID, 'Proximity', sim.simx_opmode_oneshot_wait)[1]

        # Wheel drive motors
        leftMotor = sim.simxGetObjectHandle(
            clientID, 'MotorA_FL', sim.simx_opmode_oneshot_wait)[1]
        rightMotor = sim.simxGetObjectHandle(
            clientID, 'MotorA_FR', sim.simx_opmode_oneshot_wait)[1]

        # RobotArm
        link = []
        link.append(sim.simxGetObjectHandle(
            clientID, 'L0', sim.simx_opmode_oneshot_wait)[1])
        link.append(sim.simxGetObjectHandle(
            clientID, 'L1', sim.simx_opmode_oneshot_wait)[1])
        link.append(sim.simxGetObjectHandle(
            clientID, 'L2', sim.simx_opmode_oneshot_wait)[1])
        link.append(sim.simxGetObjectHandle(
            clientID, 'L3', sim.simx_opmode_oneshot_wait)[1])

        # Gripper
        finger1 = sim.simxGetObjectHandle(
            clientID, 'Barrett_openCloseJoint0', sim.simx_opmode_oneshot_wait)[1]
        finger2 = sim.simxGetObjectHandle(
            clientID, 'Barrett_openCloseJoint', sim.simx_opmode_oneshot_wait)[1]

        # Robot
        body = sim.simxGetObjectHandle(
            clientID, 'Robot', sim.simx_opmode_oneshot_wait)[1]

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
                print("Path received...")

            if (robot_state == RobotState.TRAVELLING):
                robot_path_index, current_point, next_point = updateRobotPathIndex(
                    clientID, body, path, robot_path_index, 0.75)
                if (next_point[0] != None):  # If we are not yet at the end point
                    target_orientation = getTargetOrientation(
                        current_point, next_point)
                    orientation_error = getOrientationError(
                        clientID, body, target_orientation)
                    speedController(clientID, leftMotorF,
                                    rightMotorF, orientation_error)
                else:
                    robot_state = RobotState.SEARCHING
                    print("Reached destinattion...")
                    print("Searching...")
            elif(robot_state == RobotState.SEARCHING):
                arm_state, robot_state = searchBear(arm_state, robot_state)
            elif(robot_state == RobotState.PICKING):
                arm_state, robot_state = pickBear(
                    clientID, arm_state, robot_state, camera, cameraCar, body, L0, L1, L2, distance, distanceCar, FrontDistance)
            elif(robot_state == RobotState.RETURNING):
                print("I MADE IT !!! I AM RETURNING")

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
    drone_process.join()

    # Both processes finished
    print('Simulation has ended...')

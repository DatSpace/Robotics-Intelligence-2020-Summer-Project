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
    RESCUE = auto()
    RETURNING = auto()


class ArmState(Enum):
    EXTENT = auto()
    SEARCH = auto()
    GRAB = auto()
    RETRACT = auto()


# Program Constants
SIMULATION_STEP = 50.0  # in milliseconds
PATH_POINT_ERROR_RADIUS = 0.2  # in coppelia units
ROBOT_SPEED = -6.0
CONTROLLER_GAIN = 1.0


def speedController(clientID, error):

    if (abs(error) > 1.0):
        if (error > 0):
            leftMotorSpeed = ROBOT_SPEED / 3.0
            rightMotorSpeed = -ROBOT_SPEED / 3.0
        else:
            leftMotorSpeed = -ROBOT_SPEED / 3.0
            rightMotorSpeed = ROBOT_SPEED / 3.0
    else:
        delta = CONTROLLER_GAIN*error
        leftMotorSpeed = ROBOT_SPEED - delta
        rightMotorSpeed = ROBOT_SPEED + delta

    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftMotorSpeed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightMotorSpeed, sim.simx_opmode_oneshot)


def getOrientationError(clientID, body, target_orientation):
    # Get the current angle for the orientation of the robot from 0 to 2pi, anti-clockwise
    current_orientation = (sim.simxGetObjectOrientation(
        clientID, body, -1, sim.simx_opmode_blocking)[1])[2]

    orientation_error = current_orientation - target_orientation

    if (np.abs(orientation_error) > np.pi):
        if (orientation_error < 0):
            orientation_error = np.pi
        else:
            orientation_error = -np.pi

    in_min = -np.pi/12.0  # 15 degrees
    in_max = np.pi/12.0
    out_min = -1.0
    out_max = 1.0

    error = (orientation_error - in_min) * \
        (out_max - out_min) / (in_max - in_min) + out_min
    return error


def getTargetOrientation(robot_position, path, robot_path_index):
    current_point = path[robot_path_index]

    # If the robot is within 0.75 units radius of the red car (last path point)
    if (np.sqrt(((path[len(path) - 1][0] - robot_position[0]) ** 2.0) + ((path[len(path) - 1][1] - robot_position[1]) ** 2.0)) <= 1.0):
        return None, None

    # Else if there are more points available (we are not at the end)
    elif (robot_path_index + 1 < len(path)):
        next_point = path[robot_path_index + 1]
        if (np.sqrt(((next_point[0] - robot_position[0]) ** 2.0) + ((next_point[1] - robot_position[1]) ** 2.0)) <= PATH_POINT_ERROR_RADIUS):
            robot_path_index += 1

        # Angle of the two points in radians
        return robot_path_index, np.arctan2(next_point[1] - current_point[1], next_point[0] - current_point[0])
    # If there are no points available, which means that we are the end, but the first if didnt trigger (which means the car believes it is at the end, but it is not)
    else:
        print("If I was printed, something went terribly wrong!")


def getBearCenter(clientID, camera):
    _, resolution, image = sim.simxGetVisionSensorImage(
        clientID, camera, 0, sim.simx_opmode_buffer)

    # Camera in Hand
    original = np.array(image, dtype=np.uint8)
    original.resize([resolution[0], resolution[1], 3])
    original = cv2.flip(original, 0)
    hsv = cv2.cvtColor(original, cv2.COLOR_RGB2HSV)

    green = cv2.inRange(hsv, np.array([45, 254, 0]), np.array([64, 255, 255]))

    # Moments in the hand
    MHand = cv2.moments(green)
    if MHand["m00"] != 0:
        cXHand = int(MHand["m10"] / MHand["m00"])
        cYHand = int(MHand["m01"] / MHand["m00"])
    else:
        # If there are no green color a 0 will the value be
        cXHand, cYHand = None, None

    return cXHand, cYHand


def getLinksAnglesDegrees(clientID, link):
    L0Angle = sim.simxGetJointPosition(
        clientID, link[0], sim.simx_opmode_blocking)[1]
    L1Angle = sim.simxGetJointPosition(
        clientID, link[1], sim.simx_opmode_blocking)[1]
    L2Angle = sim.simxGetJointPosition(
        clientID, link[2], sim.simx_opmode_blocking)[1]

    # Robotic Arm Joints DOF adjusted to 90
    L0Angle = np.round(np.rad2deg(L0Angle), 1)
    L1Angle = np.round(np.rad2deg(L1Angle), 1)
    L2Angle = np.round(np.rad2deg(L2Angle), 1)

    return L0Angle, L1Angle, L2Angle


def getBladesDegrees(clientID, blade):
    LeftBlade = sim.simxGetJointPosition(
        clientID, blade[0], sim.simx_opmode_blocking)[1]
    RightBlade = sim.simxGetJointPosition(
        clientID, blade[1], sim.simx_opmode_blocking)[1]

    # Robotic Arm Joints DOF adjusted to 90
    LeftBlade = np.round(np.rad2deg(LeftBlade), 1)
    RightBlade = np.round(np.rad2deg(RightBlade), 1)

    return LeftBlade, RightBlade


def removeObstacle(clientID, blade, FrontDistance):

    _, isDetected, distance, _, _ = sim.simxReadProximitySensor(
        clientID, FrontDistance, sim.simx_opmode_blocking)

    if (isDetected):
        start_time = int(time.time())

        if (distance[0] > 0.0):
            LeftBladeSpeed = 2
            sim.simxSetJointTargetVelocity(
                clientID, blade[0], LeftBladeSpeed, sim.simx_opmode_oneshot)
            while(LeftBladeSpeed != 0):
                LeftBlade = getBladesDegrees(clientID, blade)[0]
                if (LeftBlade >= 130):
                    LeftBladeSpeed = 0
                    sim.simxSetJointTargetVelocity(
                        clientID, blade[0], LeftBladeSpeed, sim.simx_opmode_oneshot)
                end_time = int(time.time())
                if (end_time - start_time > 10):
                    break

            LeftBladeSpeed = -1
            sim.simxSetJointTargetVelocity(
                clientID, blade[0], LeftBladeSpeed, sim.simx_opmode_oneshot)
            while(LeftBladeSpeed != 0):
                LeftBlade = getBladesDegrees(clientID, blade)[0]
                if (LeftBlade <= 0):
                    LeftBladeSpeed = 0
                    sim.simxSetJointTargetVelocity(
                        clientID, blade[0], LeftBladeSpeed, sim.simx_opmode_oneshot)
        else:
            RightBladeSpeed = -2
            sim.simxSetJointTargetVelocity(
                clientID, blade[1], RightBladeSpeed, sim.simx_opmode_oneshot)
            while(RightBladeSpeed != 0):
                RightBlade = getBladesDegrees(clientID, blade)[1]
                if(RightBlade <= -130):
                    RightBladeSpeed = 0
                    sim.simxSetJointTargetVelocity(
                        clientID, blade[1], RightBladeSpeed, sim.simx_opmode_oneshot)
                end_time = int(time.time())
                if (end_time - start_time > 10):
                    break

            RightBladeSpeed = 1
            sim.simxSetJointTargetVelocity(
                clientID, blade[1], RightBladeSpeed, sim.simx_opmode_oneshot)
            while(RightBladeSpeed != 0):
                RightBlade = getBladesDegrees(clientID, blade)[1]
                if (RightBlade >= 0):
                    RightBladeSpeed = 0
                    sim.simxSetJointTargetVelocity(
                        clientID, blade[1], RightBladeSpeed, sim.simx_opmode_oneshot)


def retractArm(clientID, link):
    L0Speed = 0.2

    sim.simxSetJointTargetVelocity(
        clientID, link[0], L0Speed, sim.simx_opmode_oneshot)

    # Set Arm to initial Position
    L0Angle = getLinksAnglesDegrees(clientID, link)[0]
    while (L0Angle < 90):
        L0Angle = getLinksAnglesDegrees(clientID, link)[0]

    L0Speed = 0.0
    sim.simxSetJointTargetVelocity(
        clientID, link[0], L0Speed, sim.simx_opmode_oneshot)


def changeScale(point, in_min, in_max, out_min, out_max):
    return (point - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def rescueBear(clientID, camera, leftMotor, rightMotor, finger1, finger2, link, blade, FrontDistance, arm_state, robot_state):
    L0Speed = 0
    L1Speed = 0
    L2Speed = 0
    leftMotorSpeed = 0
    rightMotorSpeed = 0
    F1Speed = 0
    F2Speed = 0
    if (arm_state == ArmState.EXTENT):
        L0Angle = getLinksAnglesDegrees(clientID, link)[0]
        L2Angle = getLinksAnglesDegrees(clientID, link)[2]
        if L0Angle > 0:
            L0Speed = -0.2
        elif L2Angle < 35:
            L2Speed = 0.2
        else:
            arm_state = ArmState.SEARCH
            print("Searching for bear...")
    elif (arm_state == ArmState.SEARCH):
        cXHand = getBearCenter(clientID, camera)[0]
        if (cXHand != None):
            sim.simxSetJointTargetVelocity(
                clientID, leftMotor, leftMotorSpeed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, rightMotor, rightMotorSpeed, sim.simx_opmode_oneshot)

            removeObstacle(clientID, blade, FrontDistance)

            L2Speed = 0.2
            L0Speed = -0.2

            sim.simxSetJointTargetVelocity(
                clientID, leftMotor, leftMotorSpeed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, rightMotor, rightMotorSpeed, sim.simx_opmode_oneshot)

            sim.simxSetJointTargetVelocity(
                clientID, link[2], L2Speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, link[0], L0Speed, sim.simx_opmode_oneshot)

            while (L0Speed != 0 and L2Speed != 0):
                L0Angle, L1Angle, L2Angle = getLinksAnglesDegrees(
                    clientID, link)
                if (L2Angle >= 125):
                    L2Speed = 0
                    sim.simxSetJointTargetVelocity(
                        clientID, link[2], L2Speed, sim.simx_opmode_oneshot)
                if L0Angle <= -83:
                    L0Speed = 0
                    sim.simxSetJointTargetVelocity(
                        clientID, link[0], L0Speed, sim.simx_opmode_oneshot)
            arm_state = ArmState.GRAB
            print("Preparing to grab...")
        else:
            leftMotorSpeed = ROBOT_SPEED / 3.0
            rightMotorSpeed = -ROBOT_SPEED / 3.0
    elif (arm_state == ArmState.GRAB):  # Deploying the arm towards MrYork
        _, isDetected, proximity2, _, _ = sim.simxReadProximitySensor(
            clientID, distance, sim.simx_opmode_blocking)

        if (isDetected):
            if (proximity2[2] > 0.03):
                leftMotorSpeed = (ROBOT_SPEED / 3.0) * proximity2[2]
                rightMotorSpeed = (ROBOT_SPEED / 3.0) * proximity2[2]
            else:
                sim.simxSetJointTargetVelocity(
                    clientID, leftMotor, leftMotorSpeed, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(
                    clientID, rightMotor, rightMotorSpeed, sim.simx_opmode_oneshot)

                F1Speed = -0.1
                F2Speed = -0.1
                sim.simxSetJointTargetVelocity(
                    clientID, finger1, F1Speed, sim.simx_opmode_oneshot)
                sim.simxSetJointTargetVelocity(
                    clientID, finger2, F2Speed, sim.simx_opmode_oneshot)
                # Wait for 5 seconds for fingers to close, while slowly moving forward
                time.sleep(5)
                arm_state = ArmState.RETRACT
                print("Bear grabbed and retracting...")
        else:
            cXHand = getBearCenter(clientID, camera)[0]

            if (cXHand != None):
                errorX = changeScale(cXHand, 0.0, 255.0, -1.0, 1.0)

                _, isDetected, proximity2, _, _ = sim.simxReadProximitySensor(
                    clientID, distance, sim.simx_opmode_blocking)

                leftMotorSpeed = ROBOT_SPEED - (CONTROLLER_GAIN*errorX)
                rightMotorSpeed = ROBOT_SPEED + (CONTROLLER_GAIN*errorX)
            else:
                leftMotorSpeed = ROBOT_SPEED / 3.0
                rightMotorSpeed = -ROBOT_SPEED / 3.0
    # Folding the arm back to the car with Mr York grabbed
    elif (arm_state == ArmState.RETRACT):
        L0Angle, L1Angle, L2Angle = getLinksAnglesDegrees(clientID, link)

        if (L0Angle < 90):
            L0Speed = 0.2
        if (L2Angle > 0):
            L2Speed = -0.2

        if (L0Speed == 0 and L2Speed == 0):
            robot_state = RobotState.RETURNING
            print("Arm retracted...")

    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftMotorSpeed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightMotorSpeed, sim.simx_opmode_oneshot)

    sim.simxSetJointTargetVelocity(
        clientID, link[0], L0Speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, link[1], L1Speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, link[2], L2Speed, sim.simx_opmode_oneshot)

    return arm_state, robot_state


if __name__ == "__main__":

    path = []
    robot_state = RobotState.TRAVELLING
    arm_state = ArmState.EXTENT
    robot_path_index = 0

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
            clientID, 'Camera', sim.simx_opmode_blocking)[1]
        distance = sim.simxGetObjectHandle(
            clientID, 'Proximity', sim.simx_opmode_blocking)[1]

        # Proximity Sensor in front of the chasis
        FrontDistance = sim.simxGetObjectHandle(
            clientID, 'FrontSensor', sim.simx_opmode_oneshot_wait)[1]

        # Wheel drive motors
        leftMotor = sim.simxGetObjectHandle(
            clientID, 'MotorA_FL', sim.simx_opmode_blocking)[1]
        rightMotor = sim.simxGetObjectHandle(
            clientID, 'MotorA_FR', sim.simx_opmode_blocking)[1]

        # Robot Arm
        link = []
        link.append(sim.simxGetObjectHandle(
            clientID, 'L0', sim.simx_opmode_blocking)[1])
        link.append(sim.simxGetObjectHandle(
            clientID, 'L1', sim.simx_opmode_blocking)[1])
        link.append(sim.simxGetObjectHandle(
            clientID, 'L2', sim.simx_opmode_blocking)[1])

        # Gripper
        finger1 = sim.simxGetObjectHandle(
            clientID, 'Barrett_openCloseJoint0', sim.simx_opmode_blocking)[1]
        finger2 = sim.simxGetObjectHandle(
            clientID, 'Barrett_openCloseJoint', sim.simx_opmode_blocking)[1]

        # Blades
        blade = []
        blade.append(sim.simxGetObjectHandle(
            clientID, 'BladeL', sim.simx_opmode_blocking)[1])
        blade.append(sim.simxGetObjectHandle(
            clientID, 'BladeR', sim.simx_opmode_blocking)[1])

        # Robot
        body = sim.simxGetObjectHandle(
            clientID, 'Robot', sim.simx_opmode_blocking)[1]

        # PROBABLY DONT NEED ANY OF VARIABLES
        res, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera, 0, sim.simx_opmode_streaming)

        path = drone_queue.get()  # If path list is empty, wait to get a response
        print("Path received...")
        retractArm(clientID, link)

        # Start main control loop
        print('Starting ground control loop...')

        # While connected to the simulator
        while (sim.simxGetConnectionId(clientID) != -1):
            start_ms = int(round(time.time() * 1000))

            if (robot_state == RobotState.TRAVELLING):
                result, robot_position = sim.simxGetObjectPosition(
                    clientID, body, -1, sim.simx_opmode_blocking)

                if (result == sim.simx_return_ok):
                    temp_index = robot_path_index
                    robot_path_index, target_orientation = getTargetOrientation(
                        robot_position, path, robot_path_index)

                    if (robot_path_index != None):
                        orientation_error = getOrientationError(
                            clientID, body, target_orientation)
                        speedController(clientID, orientation_error)
                    else:
                        # Prepare path for returning
                        del path[temp_index: len(path)]
                        path.reverse()
                        path.insert(0, [robot_position[0], robot_position[1]])
                        robot_state = RobotState.RESCUE
                        print("Reached destination...")
                        print("Searching...")
            elif(robot_state == RobotState.RESCUE):
                arm_state, robot_state = rescueBear(
                    clientID, camera, leftMotor, rightMotor, finger1, finger2, link, blade, FrontDistance, arm_state, robot_state)
            elif(robot_state == RobotState.RETURNING):
                result, robot_position = sim.simxGetObjectPosition(
                    clientID, body, -1, sim.simx_opmode_blocking)

                if (result == sim.simx_return_ok):
                    if (robot_path_index == None):
                        path.insert(0, [robot_position[0], robot_position[1]])
                        robot_path_index = 0
                    else:
                        robot_path_index, target_orientation = getTargetOrientation(
                            robot_position, path, robot_path_index)

                        if (robot_path_index != None):
                            orientation_error = getOrientationError(
                                clientID, body, target_orientation)
                            speedController(
                                clientID, orientation_error)
                        else:
                            print("The patient has been safely returned...")
                            sim.simxSetJointTargetVelocity(
                                clientID, leftMotor, 0.0, sim.simx_opmode_oneshot)
                            sim.simxSetJointTargetVelocity(
                                clientID, rightMotor, 0.0, sim.simx_opmode_oneshot)
                            break

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

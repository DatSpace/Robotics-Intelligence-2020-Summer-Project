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
PATH_POINT_ERROR_RADIUS = 0.5  # in coppelia units
DEFAULT_SPEED = -1.0
CONTROLLER_GAIN = 5.0

# Program Variables
leftMotorSpeed = 0.0
rightMotorSpeed = 0.0

L0Speed = L1Speed = L2Speed = L3Speed = 0.0
F1Speed = F2Speed = 0.0

path = []
robot_state = RobotState.TRAVELLING
arm_state = ArmState.RETRACT
robot_path_index = 0


def speedController(clientID, leftMotorSpeed, rightMotorSpeed, gain, default_speed, error):

    delta = gain*error
    leftMotorSpeed = default_speed - delta
    rightMotorSpeed = default_speed + delta

    return leftMotorSpeed, rightMotorSpeed


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


def getBearCenter(clientID, camera):
    res, resolution, image = sim.simxGetVisionSensorImage(
        clientID, camera, 0, sim.simx_opmode_buffer)

    #Camera in Hand
    original = np.array(image, dtype=np.uint8)
    original.resize([resolution[0], resolution[1], 3])
    original = cv2.flip(original, 0)
    original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
    green = cv2.inRange(original, (0, 100, 0), (32, 255, 255))

    # Moments in the hand
    MHand = cv2.moments(green)
    if MHand["m00"] != 0:
        cXHand = int(MHand["m10"] / MHand["m00"])
        cYHand = int(MHand["m01"] / MHand["m00"])
    else:
        # If there are no green color a 0 will the value be
        cXHand, cYHand = 0, 0

    return cXHand, cYHand


def getLinksAnglesDegrees(clientID, link):
    L0Angle = sim.simxGetJointPosition(
        clientID, link[0], sim.simx_opmode_oneshot_wait)[1]
    L1Angle = sim.simxGetJointPosition(
        clientID, link[1], sim.simx_opmode_oneshot_wait)[1]
    L2Angle = sim.simxGetJointPosition(
        clientID, link[2], sim.simx_opmode_oneshot_wait)[1]

    # Robotic Arm Joints DOF adjusted to 90
    L0Angle = np.round(np.rad2deg(L0Angle), 1)
    L1Angle = np.round(np.rad2deg(L1Angle), 1)
    L2Angle = np.round(np.rad2deg(L2Angle), 1)

    return L0Angle, L1Angle, L2Angle


def retractArm(clientID, link):
    L1Speed = 1.0
    sim.simxSetJointTargetVelocity(
        clientID, link[1], L1Speed, sim.simx_opmode_oneshot)

    # Set Arm to initial Position
    L1Angle = getLinksAnglesDegrees(clientID, link)
    while (L1Angle < 155):
        L1Angle = getLinksAnglesDegrees(clientID, link)

    L1Speed = 0.0
    sim.simxSetJointTargetVelocity(
        clientID, link[1], L1Speed, sim.simx_opmode_oneshot)


def searchBear(clientID, link, arm_state, robot_state, leftMotorSpeed, rightMotorSpeed, L0Speed, L1Speed, L2Speed):
    if (arm_state == ArmState.EXTENT):
        L1Angle = getLinksAnglesDegrees(clientID, link)[1]
        if L1Angle > 90:
            L1Speed = -0.5
        elif 90 > L1Angle > 0:
            L1Speed = -0.1
        else:
            L1Speed == 0.0
            arm_state = ArmState.WAIT
    elif (arm_state == ArmState.WAIT):
        #   Fine adjustment looking for MrYork around the Manta
        L0Speed = 0
        L1Speed = 0
        L2Speed = 0
        cXHand, cYHand = getBearCenter(clientID, camera)
        # No MrY
        if cXHand < 1:
            leftMotorSpeed = 1.5
            rightMotorSpeed = -1.5
        # Centering momentum in X
        elif 1 < cXHand < 105:
            leftMotorSpeed = 0.4
            rightMotorSpeed = -0.4
        elif cXHand > 145:
            leftMotorSpeed = -0.8
            rightMotorSpeed = 0.8
        # Centering momentun in Y
        elif 1 < cYHand < 65:
            position4 = (cYHand-155)
            delta4 = 2*position4/100
            leftMotorSpeed = -(0.8-delta4)
            rightMotorSpeed = -(0.8-delta4)
        elif cYHand > 100:
            leftMotorSpeed = 0.8
            rightMotorSpeed = 0.8
        # The GroundRobot is ready to deploy
        else:
            leftMotorSpeed = 0
            rightMotorFSpeed = 0
            arm_state = ArmState.GRAB
            robot_state = RobotState.PICKING
    return arm_state, robot_state, leftMotorSpeed, rightMotorSpeed, L0Speed, L1Speed, L2Speed


def pickBear(clientID, arm_state, robot_state, camera, body, link, distance, F1, F2, leftMotorSpeed, rightMotorSpeed, L0Speed, L1Speed, L2Speed, F1Speed, F2Speed):
    L0Angle, L1Angle, L2Angle = getLinksAnglesDegrees(clientID, link)

    #   Deploying the arm towards MrYork
    if (arm_state == ArmState.GRAB):
        if (L2Angle < 125):
            L2Speed = 0.2
        else:
            L2Speed = 0
        if L0Angle > -83:
            L0Speed = -0.07
        else:
            # Super Fine adjustment to center
            L0Speed = 0
            L1Speed = 0
            L2Speed = 0
            cXHand = getBearCenter(clientID, camera)[0]
            if 1 < cXHand < 95:
                leftMotorSpeed = 0.1
                rightMotorSpeed = -0.1
            elif cXHand > 165:
                leftMotorSpeed = -0.1
                rightMotorSpeed = 0.1
            else:
                # Super Fine adjustment to reach MrY
                leftMotorSpeed = 0.0
                rightMotorSpeed = 0.0

                res, proximity1, proximity2, proximity3, proximity4 = sim.simxReadProximitySensor(
                    clientID, distance, sim.simx_opmode_oneshot_wait)
                proximity2 = np.round(proximity2, 2)

                if (proximity1 == True and proximity2[2] > 0.03):
                    leftMotorSpeed = -0.08
                    rightMotorSpeed = -0.08
                else:
                    if(L0Angle < -35 and L2Angle > 40):
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
        if(L0Angle < -25):
            L0Speed = 0.3
        if (L1Angle < 170):
            L1Speed = 0.3
        if(L2Angle > -10):
            L2Speed = -0.1
        else:
            L0Speed = 0
            L1Speed = 0
            L2Speed = 0
            arm_state = ArmState.WAIT
            robot_state = RobotState.RETURNING
    return arm_state, robot_state, leftMotorSpeed, rightMotorSpeed, L0Speed, L1Speed, L2Speed, F1Speed, F2Speed


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

        # Gripper
        finger1 = sim.simxGetObjectHandle(
            clientID, 'Barrett_openCloseJoint0', sim.simx_opmode_oneshot_wait)[1]
        finger2 = sim.simxGetObjectHandle(
            clientID, 'Barrett_openCloseJoint', sim.simx_opmode_oneshot_wait)[1]

        # Robot
        body = sim.simxGetObjectHandle(
            clientID, 'Robot', sim.simx_opmode_oneshot_wait)[1]

        # PROBABLY DONT NEED ANY OF VARIABLES
        res, resolution, image = sim.simxGetVisionSensorImage(
            clientID, camera, 0, sim.simx_opmode_streaming)

        # Start main control loop
        print('Starting ground control loop...')

        # While connected to the simulator
        while (sim.simxGetConnectionId(clientID) != -1):
            start_ms = int(round(time.time() * 1000))

            if not path:  # If path list is empty, wait to get a response
                path = drone_queue.get()
                print("Path received...")

            retractArm(clientID, link)

            if (robot_state == RobotState.TRAVELLING):
                robot_path_index, current_point, next_point = updateRobotPathIndex(
                    clientID, body, path, robot_path_index, PATH_POINT_ERROR_RADIUS)
                if (next_point[0] != None):  # If we are not yet at the end point
                    target_orientation = getTargetOrientation(
                        current_point, next_point)
                    orientation_error = getOrientationError(
                        clientID, body, target_orientation)
                    leftMotorSpeed, rightMotorSpeed = speedController(
                        clientID leftMotorSpeed, rightMotorSpeed, CONTROLLER_GAIN, DEFAULT_SPEED, orientation_error)
                else:
                    robot_state = RobotState.SEARCHING
                    print("Reached destinattion...")
                    print("Searching...")
            elif(robot_state == RobotState.SEARCHING):
                arm_state, robot_state, leftMotorSpeed, rightMotorSpeed, L0Speed, L1Speed, L2Speed = searchBear(
                    clientID, link, arm_state, robot_state, leftMotorSpeed, rightMotorSpeed, L0Speed, L1Speed, L2Speed)
            elif(robot_state == RobotState.PICKING):
                arm_state, robot_state, leftMotorSpeed, rightMotorSpeed, L0Speed, L1Speed, L2Speed, F1Speed, F2Speed = pickBear(
                    clientID, arm_state, robot_state, camera, body, link, distance, F1, F2, leftMotorSpeed, rightMotorSpeed, L0Speed, L1Speed, L2Speed, F1Speed, F2Speed)
            elif(robot_state == RobotState.RETURNING):
                print("I MADE IT !!! I AM RETURNING")

            sim.simxSetJointTargetVelocity(
                clientID, leftMotor, leftMotorSpeed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, rightMotor, rightMotorSpeed, sim.simx_opmode_oneshot)
            # Set actuators on mobile robot
            sim.simxSetJointTargetVelocity(
                clientID, link[0], L0Speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, link[1], L1Speed, sim.simx_opmode_oneshot)
            sim.simxSetJointTargetVelocity(
                clientID, link[2], L2Speed, sim.simx_opmode_oneshot)
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
    drone_process.join()

    # Both processes finished
    print('Simulation has ended...')

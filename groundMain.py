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
PATH_POINT_ERROR_RADIUS = 0.5  # in coppelia units
DEFAULT_SPEED = -1.0
CONTROLLER_GAIN = 5.0

# Program Variables
path = []
robot_state = RobotState.TRAVELLING
arm_state = ArmState.EXTENT
robot_path_index = 0


def speedController(clientID, gain, default_speed, error):

    delta = gain*error
    leftMotorSpeed = default_speed - delta
    rightMotorSpeed = default_speed + delta

    sim.simxSetJointTargetVelocity(
        clientID, leftMotor, leftMotorSpeed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, rightMotor, rightMotorSpeed, sim.simx_opmode_oneshot)


def getOrientationError(clientID, body, target_orientation):
    # Get the current angle for the orientation of the robot from 0 to 2pi, anti-clockwise
    orientation = euler2rotation((sim.simxGetObjectOrientation(
        clientID, body, -1, sim.simx_opmode_oneshot)[1])[2])

    orientation_error = orientation - target_orientation

    in_min = 0
    in_max = 2.0*np.pi
    out_min = -1.0
    out_max = 1.0
    return (orientation_error - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def getTargetOrientation(current_point, next_point):
    # Angle of the two points in radians
    return np.arctan2(next_point[1]-current_point[1], next_point[0]-current_point[0])


def updateRobotPathIndex(clientID, robot, path, robot_path_index, radius):
    current_point = path[robot_path_index]

    result, current_robot_location = sim.simxGetObjectPosition(
        clientID, robot, -1, sim.simx_opmode_oneshot)
    while (result != sim.simx_return_ok):
        result, current_robot_location = sim.simxGetObjectPosition(
            clientID, robot, -1, sim.simx_opmode_oneshot)

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
    hsv = cv2.cvtColor(original, cv2.COLOR_RGB2HSV)

    green = cv2.inRange(hsv, np.array(
        [45, 254, 0]), np.array([64, 255, 255]))

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
    L1Speed = 0.5
    sim.simxSetJointTargetVelocity(
        clientID, link[1], L1Speed, sim.simx_opmode_oneshot)

    # Set Arm to initial Position
    L1Angle = getLinksAnglesDegrees(clientID, link)[1]
    while (L1Angle < 170):
        L1Angle = getLinksAnglesDegrees(clientID, link)[1]

    L1Speed = 0.0
    sim.simxSetJointTargetVelocity(
        clientID, link[1], L1Speed, sim.simx_opmode_oneshot)
    print("Arm retracted...")


def changeScale(point, in_min, in_max, out_min, out_max):
    return (point - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def euler2rotation(euler_angle):
    if (euler_angle < 0):
        positive = -euler_angle
        return (2*np.pi) - positive
    return euler_angle


def rescueBear(clientID, link, arm_state, robot_state):
    L0Speed = 0
    L1Speed = 0
    L2Speed = 0
    leftMotorSpeed = 0
    rightMotorSpeed = 0
    F1Speed = 0
    F2Speed = 0
    if (arm_state == ArmState.EXTENT):
        L1Angle = getLinksAnglesDegrees(clientID, link)[1]
        if L1Angle > 0:
            L1Speed = -0.5
        else:
            arm_state = ArmState.SEARCH
            print("Searching for bear...")
    elif (arm_state == ArmState.SEARCH):
        cXHand = getBearCenter(clientID, camera)[0]
        if (cXHand != None):
            L2Speed = 0.2
            L0Speed = -0.2
            leftMotorSpeed = 0
            rightMotorSpeed = 0

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
            leftMotorSpeed = DEFAULT_SPEED
            rightMotorSpeed = -DEFAULT_SPEED

    elif (arm_state == ArmState.GRAB):  # Deploying the arm towards MrYork
        cXHand = getBearCenter(clientID, camera)[0]

        gain = 1.0
        errorX = changeScale(cXHand, 0.0, 256.0, -1.0, 1.0)

        _, isDetected, proximity2, _, _ = sim.simxReadProximitySensor(
            clientID, distance, sim.simx_opmode_oneshot_wait)

        leftMotorSpeed = DEFAULT_SPEED - (gain*errorX)
        rightMotorSpeed = DEFAULT_SPEED + (gain*errorX)
        if (isDetected == True):
            if (proximity2[2] > 0.03):
                leftMotorSpeed = DEFAULT_SPEED*proximity2[2]
                rightMotorSpeed = DEFAULT_SPEED*proximity2[2]
            else:
                F1Speed = -0.2
                F2Speed = -0.2
                arm_state = ArmState.RETRACT
                print("Bear grabbed and retracting...")
    #   Folding the arm back to the car with Mr York grabbed
    elif (arm_state == ArmState.RETRACT):
        L0Angle, L1Angle, L2Angle = getLinksAnglesDegrees(clientID, link)
        F1Speed = -0.2
        F2Speed = -0.2

        if(L0Angle < -25):
            L0Speed = 0.2
        if (L1Angle < 170):
            L1Speed = 0.2
        if(L2Angle > -10):
            L2Speed = -0.2
        else:
            robot_state = RobotState.RETURNING

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
    sim.simxSetJointTargetVelocity(
        clientID, finger1, F1Speed, sim.simx_opmode_oneshot)
    sim.simxSetJointTargetVelocity(
        clientID, finger2, F2Speed, sim.simx_opmode_oneshot)

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

        # Robot Arm
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

        path = drone_queue.get()  # If path list is empty, wait to get a response
        print("Path received...")
        retractArm(clientID, link)

        # Start main control loop
        print('Starting ground control loop...')

        # While connected to the simulator
        while (sim.simxGetConnectionId(clientID) != -1):
            start_ms = int(round(time.time() * 1000))

            if (robot_state == RobotState.TRAVELLING):
                robot_path_index, current_point, next_point = updateRobotPathIndex(
                    clientID, body, path, robot_path_index, PATH_POINT_ERROR_RADIUS)
                if (next_point[0] != None):  # If we are not yet at the end point
                    target_orientation = getTargetOrientation(
                        current_point, next_point)
                    orientation_error = getOrientationError(
                        clientID, body, target_orientation)
                    speedController(clientID, CONTROLLER_GAIN,
                                    DEFAULT_SPEED, orientation_error)
                else:
                    robot_state = RobotState.RESCUE
                    print("Reached destinattion...")
                    print("Searching...")
            elif(robot_state == RobotState.RESCUE):
                arm_state, robot_state = rescueBear(
                    clientID, link, arm_state, robot_state)
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

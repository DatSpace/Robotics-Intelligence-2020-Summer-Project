#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.

import sim
import cv2
import numpy as np


def flyUp(clientID, drone_target, step):
    res, current_target = sim.simxGetObjectPosition(
        clientID, drone_target, -1, sim.simx_opmode_oneshot)
    if res == sim.simx_return_ok:
        new_target = np.array(current_target) + np.array([0.0, 0.0, step])
        sim.simxSetObjectPosition(
            clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)


def flyDown(clientID, drone_target, step):
    res, current_target = sim.simxGetObjectPosition(
        clientID, drone_target, -1, sim.simx_opmode_oneshot)
    if res == sim.simx_return_ok:
        new_target = np.array(current_target) + np.array([0.0, 0.0, -step])
        sim.simxSetObjectPosition(
            clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)


def flyLeft(clientID, drone_target, step):
    res, current_target = sim.simxGetObjectPosition(
        clientID, drone_target, -1, sim.simx_opmode_oneshot)
    if res == sim.simx_return_ok:
        new_target = np.array(current_target) + np.array([0.0, step, 0.0])
        sim.simxSetObjectPosition(
            clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)


def flyRight(clientID, drone_target, step):
    res, current_target = sim.simxGetObjectPosition(
        clientID, drone_target, -1, sim.simx_opmode_oneshot)
    if res == sim.simx_return_ok:
        new_target = np.array(current_target) + np.array([0.0, -step, 0.0])
        sim.simxSetObjectPosition(
            clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)


def flyForward(clientID, drone_target, step):
    res, current_target = sim.simxGetObjectPosition(
        clientID, drone_target, -1, sim.simx_opmode_oneshot)
    if res == sim.simx_return_ok:
        new_target = np.array(current_target) + np.array([step, 0.0, 0.0])
        sim.simxSetObjectPosition(
            clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)


def flyBackward(clientID, drone_target, step):
    res, current_target = sim.simxGetObjectPosition(
        clientID, drone_target, -1, sim.simx_opmode_oneshot)
    if res == sim.simx_return_ok:
        new_target = np.array(current_target) + np.array([-step, 0.0, 0.0])
        sim.simxSetObjectPosition(
            clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)


def main(drone_queue):

    # Start Program and just in case, close all opened connections
    print('Program started...')
    sim.simxFinish(-1)

    # Connect to simulator running on localhost
    # V-REP runs on port 19997, a script opens the API on port 19999
    clientID = sim.simxStart('127.0.0.1', 19998, True, True, 5000, 5)

    # Connect to the simulation
    if clientID != -1:
        print('Connected to remote API server...')
        print('Obtaining handles of simulation objects...')

        # Target object for drone navigation
        res, drone_target = sim.simxGetObjectHandle(
            clientID, 'DroneTarget', sim.simx_opmode_oneshot_wait)
        if res != sim.simx_return_ok:
            print('Could not get handle to quadcopter target object...')

        # Bottom drone camera
        res, drone_camera = sim.simxGetObjectHandle(
            clientID, 'DroneFloorCamera', sim.simx_opmode_oneshot_wait)
        if res != sim.simx_return_ok:
            print('Could not get handle to drone camera object...')

        # Start main control loop
        print('Starting control loop...')
        res, resolution, image = sim.simxGetVisionSensorImage(
            clientID, drone_camera, 0, sim.simx_opmode_streaming)

        # While connected to the simulator
        while (sim.simxGetConnectionId(clientID) != -1):
            # Get the image from the camera
            res, resolution, image = sim.simxGetVisionSensorImage(
                clientID, drone_camera, 0, sim.simx_opmode_buffer)
            flyForward(clientID, drone_target, 0.1)
            if res == sim.simx_return_ok:

                # Convert from V-REP representation to OpenCV
                original_image = np.array(image, dtype=np.uint8)
                original_image.resize([resolution[0], resolution[1], 3])

                original_image = cv2.flip(original_image, 0)
                original_image = cv2.cvtColor(
                    original_image, cv2.COLOR_RGB2BGR)

                cv2.imshow("Drone View", original_image)

            elif res == sim.simx_return_novalue_flag:
                # Camera has not started or is not returning images
                print("No image yet...")
            else:
                # Something else has happened
                print("Unexpected error returned...", res)

            key = cv2.waitKey(1) & 0xFF
            if (key == ord('q')):
                break

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        sim.simxGetPingTime(clientID)
        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
        cv2.destroyAllWindows()
    else:
        print('Failed connecting to remote API server...')
    print('Simulation has ended...')

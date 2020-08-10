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
import time
import numpy as np
from RRT import rapidlyExploringRandomTree

# Program Constants
SCR_WIDTH = 512
SCR_HEIGHT = 512
SIMULATION_STEP = 50.0  # in milliseconds
DRONE_GOAL_HEIGHT = 8.0
ROBOT_RADIUS_PIXELS = 10  # Based on an image of 512x512

# Processes the binary image to account for the size of the ground robot


def proccessToFinalMap(binary_map):
    print("Started processing of binary image...")
    final_map = np.copy(binary_map)

    for i in range(SCR_WIDTH):
        for j in range(SCR_HEIGHT):
            if (binary_map[i][j] == 255):
                cv2.circle(final_map, (j, i), ROBOT_RADIUS_PIXELS, 255)
    return final_map

# Converts the pixel coordinates from the map to world coordinates for the robots


def cameraToWorldCoord2D(current_pos, camera_min, camera_max, world_min, world_max):
    mapX = (current_pos[0] - camera_min[0]) * (world_max[0] -
                                               world_min[0]) / (camera_max[0] - camera_min[0]) + world_min[0]
    mapY = (current_pos[1] - camera_min[1]) * (world_max[1] -
                                               world_min[1]) / (camera_max[1] - camera_min[1]) + world_min[1]
    return np.array([mapX, mapY])


def worldToCameraCoord2D(current_pos, world_min, world_max, camera_min, camera_max):
    mapX = int((current_pos[0] - world_min[0]) * (camera_max[0] -
                                                  camera_min[0]) / (world_max[0] - world_min[0]) + camera_min[0])
    mapY = int((current_pos[1] - world_min[1]) * (camera_max[1] -
                                                  camera_min[1]) / (world_max[1] - world_min[1]) + camera_min[1])
    return np.array([mapX, mapY])


def calculateMoments(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    red_mask1 = cv2.inRange(hsv, np.array(
        [0, 100, 50]), np.array([10, 255, 255]))
    red_mask2 = cv2.inRange(hsv, np.array(
        [170, 100, 50]), np.array([180, 255, 255]))
    red_mask = red_mask1 | red_mask2

    red_filtered = cv2.bitwise_and(image, image, mask=red_mask)

    gray_image = cv2.cvtColor(red_filtered, cv2.COLOR_BGR2GRAY)

    # convert the grayscale image to binary image
    ret, thresh = cv2.threshold(gray_image, 0, 255, 0)

    # calculate moments of binary image
    return cv2.moments(thresh)


def getCarPixelCentre(original):
    M = calculateMoments(original)

    # calculate x,y coordinate of center
    if (M["m00"] != 0):
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX = -1
        cY = -1
    # car_world_location = cameraToWorldCoord2D([cX, cY], [0.0, 0.0], [SCR_WIDTH, SCR_HEIGHT], [-10.0, -10.0], [10.0, 10.0])
    return [cX, cY]


def getFilteredMap(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    green_mask = cv2.inRange(hsv, np.array(
        [45, 0, 0]), np.array([65, 255, 255]))

    white_mask = cv2.inRange(hsv, np.array(
        [0, 0, 0]), np.array([0, 0, 255]))

    mask = green_mask | white_mask

    filtered_image = cv2.bitwise_and(image, image, mask=mask)

    return filtered_image


def getBinaryMap(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_image = cv2.blur(gray_image, (2, 2))
    return cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)[1]


def moveToCentre(clientID, drone_target, drone_target_position):
    if (round(drone_target_position[2], 3) != DRONE_GOAL_HEIGHT):
        drone_target_position = flyUp(
            clientID, drone_target, drone_target_position, 0.01)
    if (round(drone_target_position[1], 3) != 0.0):
        drone_target_position = flyLeft(
            clientID, drone_target, drone_target_position, 0.01)
    if (round(drone_target_position[0], 3) != 0.0):
        drone_target_position = flyForward(
            clientID, drone_target, drone_target_position, 0.01)
    return drone_target_position


def scanMap(clientID, drone_target, drone_target_position):
    drone_target_position = flyForward(
        clientID, drone_target, drone_target_position, 0.01)
    drone_target_position = flyLeft(
        clientID, drone_target, drone_target_position, 0.01)
    return drone_target_position


def flyUp(clientID, drone_target, drone_target_position, step):
    new_target = drone_target_position + np.array([0.0, 0.0, step])
    sim.simxSetObjectPosition(
        clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)
    return new_target


def flyDown(clientID, drone_target, drone_target_position, step):
    new_target = drone_target_position + np.array([0.0, 0.0, -step])
    sim.simxSetObjectPosition(
        clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)
    return new_target


def flyLeft(clientID, drone_target, drone_target_position, step):
    new_target = drone_target_position + np.array([0.0, step, 0.0])
    sim.simxSetObjectPosition(
        clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)
    return new_target


def flyRight(clientID, drone_target, drone_target_position, step):
    new_target = drone_target_position + np.array([0.0, -step, 0.0])
    sim.simxSetObjectPosition(
        clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)
    return new_target


def flyForward(clientID, drone_target, drone_target_position, step):
    new_target = drone_target_position + np.array([step, 0.0, 0.0])
    sim.simxSetObjectPosition(
        clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)
    return new_target


def flyBackward(clientID, drone_target, drone_target_position, step):
    new_target = drone_target_position + np.array([-step, 0.0, 0.0])
    sim.simxSetObjectPosition(
        clientID, drone_target, -1, new_target, sim.simx_opmode_oneshot)
    return new_target


def main(drone_queue):
    drone_target_position = np.array([0, 0, 0])
    binary_map = np.array([])
    final_map = np.array([])
    start_point = [None, None]
    end_point = [None, None]

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

        # Drone object
        res, drone = sim.simxGetObjectHandle(
            clientID, 'Drone', sim.simx_opmode_oneshot_wait)
        if res != sim.simx_return_ok:
            print('Could not get handle to drone object...')

        # Bottom drone camera
        res, drone_camera = sim.simxGetObjectHandle(
            clientID, 'DroneFloorCamera', sim.simx_opmode_oneshot_wait)
        if res != sim.simx_return_ok:
            print('Could not get handle to drone camera object...')

        # Start main control loop
        print('Starting control loop...')
        res, resolution, image = sim.simxGetVisionSensorImage(
            clientID, drone_camera, 0, sim.simx_opmode_streaming)

        drone_target_res = sim.simx_return_novalue_flag

        # While connected to the simulator
        while (sim.simxGetConnectionId(clientID) != -1):
            start_ms = int(round(time.time() * 1000))

            if (final_map.size != 0):
                cv2.imshow("Pre-processed Map", binary_map)
                cv2.imshow("Post-processed Map", final_map)
                pass
            else:
                # Elevate drone and start scanning movement
                if (drone_target_res is sim.simx_return_ok):
                    if (not np.allclose(drone_target_position, [0.0, 0.0, DRONE_GOAL_HEIGHT])):
                        drone_target_position = moveToCentre(
                            clientID, drone_target, drone_target_position)
                    else:
                        # Wait for 10 seconds to allow drone to stabilize
                        time.sleep(10)
                        # Get the image from the camera
                        res, resolution, image = sim.simxGetVisionSensorImage(
                            clientID, drone_camera, 0, sim.simx_opmode_buffer)
                        if res == sim.simx_return_ok:
                            print("Captured snapshot of map...")

                            # Convert from V-REP representation to OpenCV
                            original_image = np.array(image, dtype=np.uint8)
                            original_image.resize(
                                [resolution[0], resolution[1], 3])

                            original_image = cv2.flip(original_image, 0)
                            original_image = cv2.cvtColor(
                                original_image, cv2.COLOR_RGB2BGR)

                            end_point = getCarPixelCentre(original_image)

                            filtered_map = getFilteredMap(original_image)
                            binary_map = getBinaryMap(filtered_map)
                            cv2.imwrite("original_map.png", binary_map)
                            final_map = proccessToFinalMap(binary_map)

                            end_point[1] += 20
                            path = rapidlyExploringRandomTree(
                                image, start_point, end_point)

                            # TEMPORARY
                            print(path)

                            drawn_map = np.copy(final_map)
                            drawn_map = cv2.cvtColor(
                                drawn_map, cv2.COLOR_GRAY2BGR)
                            cv2.circle(drawn_map, tuple(
                                start_point), 10, (255, 0, 0), -1)
                            cv2.circle(drawn_map, tuple(
                                end_point), 10, (0, 0, 255), -1)
                            cv2.imshow("Drawn Map", drawn_map)
                            #cv2.imshow("Drawn Map", draw_path(drawn_map, solution_map))
                else:
                    drone_target_res, drone_target_position = sim.simxGetObjectPosition(
                        clientID, drone_target, -1, sim.simx_opmode_oneshot)
                    start_point = worldToCameraCoord2D([drone_target_position[0], drone_target_position[1]], [
                                                       10.0, 10.0], [-10.0, -10.0], [0.0, 0.0], [SCR_WIDTH, SCR_HEIGHT])

            key = cv2.waitKey(1) & 0xFF
            if (key == ord('q')):
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
    print('Simulation has ended...')


main(None)

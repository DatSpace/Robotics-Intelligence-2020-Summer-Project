#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Make sure to have the server side running in CoppeliaSim:
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.

from path_planning import path_planning
from remote import *
from utils import *

import sim
import cv2
import time
import numpy as np
import RRT
# Program Constants
SCR_WIDTH = 512
SCR_HEIGHT = 512
SIMULATION_STEP = 50.0  # in milliseconds


def heurestic(start, goal):
    # Manhatan distance
    return abs(start[0] - goal[0]) + abs(start[1] - goal[1])

# Completly custom made shortest path algorithm for the maze
# For each node it goes through the four possible neightbours and filters based
# on if they are valid nodes (visited by the robot), then if they have been visited before (ignores them),
# and if they are dead ends (ignores)
# Then from the left over options for this node, it selects the one which is closer to the goal.
# If there is no possible move, then it is a dead end, so it take as step back, marks the node as dead, and tries again.
# COPIED FROM MY PRACTICAL ROBOTICS PROJECT





def main():
    """Path Planning.
    """
    # Load the maze image.
    img = cv2.imread("maze.png", cv2.binary_map)  # type:np.ndarray
    cv2.imshow("Maze", img)

    # Find the solution.
    start = time.time()
    solution = path_planning(img)
    cost = time.time() - start
    print(f"It costs {cost} seconds to find the path.")

    # Show the solution.
    cv2.imshow("Solution", draw_path(img, solution))
    cv2.imwrite("solution.png", draw_path(img, solution))

    # When we press "q", the program will exit.
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

"""
def find_shortest_path(maze_map, start, goal):
    possible_moves = [[1, 0], [-1, 0], [0, 1], [0, -1]]
    path = np.zeros([21, 21], dtype=int)
    visited_nodes = list(start)
    dead_nodes = list(start)
    grid_position = start
    path[grid_position[0]][grid_position[1]] = 1
    while (grid_position != goal):
        min_h = 400  # The highest possible h value
        best_node = grid_position
        for move in possible_moves:
            next_node = [grid_position[0]+move[0], grid_position[1]+move[1]]
            if (maze_map[next_node[0]][next_node[1]] != 0 and (not next_node in visited_nodes) and (not next_node in dead_nodes)):
                h = heurestic(next_node, goal)
                if (h < min_h):
                    min_h = h
                    best_node = next_node
        if (min_h == 400):  # No possible paths
            path[grid_position[0], grid_position[1]] = 0
            dead_nodes.append(grid_position)
            for move in possible_moves:
                next_node = [grid_position[0] +
                             move[0], grid_position[1]+move[1]]
                if ((next_node in visited_nodes) and (not next_node in dead_nodes)):
                    grid_position = next_node
                    break
        else:
            path[grid_position[0]][grid_position[1]] = 1
            grid_position = best_node
            visited_nodes.append(best_node)
    return path
"""

# Converts the pixel coordinates from the map to world coordinates for the robots


def cameraToWorldCord2D(current_pos, camera_min, camera_max, world_min, world_max):
    mapX = (current_pos[0] - camera_min[0]) * (world_max[0] -
                                               world_min[0]) / (camera_max[0] - camera_min[0]) + world_min[0]
    mapY = (current_pos[1] - camera_min[1]) * (world_max[1] -
                                               world_min[1]) / (camera_max[1] - camera_min[1]) + world_min[1]
    return np.array([mapX, mapY, current_pos[2]])


def getFilteredMap(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    red_mask1 = cv2.inRange(hsv, np.array(
        [0, 100, 50]), np.array([15, 255, 255]))
    red_mask2 = cv2.inRange(hsv, np.array(
        [170, 100, 50]), np.array([180, 255, 255]))
    red_mask = red_mask1 | red_mask2

    blue_mask = cv2.inRange(hsv, np.array(
        [100, 100, 50]), np.array([135, 255, 255]))

    green_mask = cv2.inRange(hsv, np.array(
        [45, 0, 0]), np.array([65, 255, 255]))

    white_mask = cv2.inRange(hsv, np.array(
        [0, 0, 0]), np.array([0, 0, 255]))

    mask = red_mask | blue_mask | green_mask | white_mask

    filtered_image = cv2.bitwise_and(image, image, mask=mask)

    return filtered_image


def getBinaryMap(image):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray_image = cv2.blur(gray_image, (2, 2))
    return cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)[1]


def moveToCentre(clientID, drone_target, drone_target_position):
    if (round(drone_target_position[2], 3) != 9.0):
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

        # Testing print statment
        print(cameraToWorldCord2D([128.0, 346.0, 8.0], [0.0, 0.0], [
              512.0, 512.0], [-10.0, -10.0], [10.0, 10.0]))

        # While connected to the simulator
        while (sim.simxGetConnectionId(clientID) != -1):
            start_ms = int(round(time.time() * 1000))

            # Elevate drone and start scanning movement
            if (drone_target_res is sim.simx_return_ok):

                if (not np.allclose(drone_target_position, [0.0, 0.0, 9.0])):
                    drone_target_position = moveToCentre(
                        clientID, drone_target, drone_target_position)
                else:
                    # Get the image from the camera
                    res, resolution, image = sim.simxGetVisionSensorImage(
                        clientID, drone_camera, 0, sim.simx_opmode_buffer)
                    if res == sim.simx_return_ok:
                        # Convert from V-REP representation to OpenCV
                        original_image = np.array(image, dtype=np.uint8)
                        original_image.resize(
                            [resolution[0], resolution[1], 3])

                        original_image = cv2.flip(original_image, 0)
                        original_image = cv2.cvtColor(
                            original_image, cv2.COLOR_RGB2BGR)

                        filtered_map = getFilteredMap(original_image)
                        binary_map = getBinaryMap(filtered_map)

                        cv2.imshow("Drone View", original_image)
                        cv2.imshow("Filtered View", filtered_map)
                        cv2.imshow("Map View", binary_map)
                       
                        
                        
                        solution_map = path_planning(binary_map)
                        cv2.imshow("Solution", draw_path(binary_map, solution_map))
                    
                    
                    
                    elif res == sim.simx_return_novalue_flag:
                        # Camera has not started or is not returning images
                        print("No image yet...")
                    else:
                        # Something else has happened
                        print("Unexpected error returned...", res)

                # if (round(drone_target_position[2], 3) == 5.0):
                #     drone_target_position=scanMap(
                #         clientID, drone_target, drone_target_position)
                # else:
                #     drone_target_position=flyUp(
                #         clientID, drone_target, drone_target_position, 0.01)
            else:
                drone_target_res, drone_target_position = sim.simxGetObjectPosition(
                    clientID, drone_target, -1, sim.simx_opmode_oneshot)

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

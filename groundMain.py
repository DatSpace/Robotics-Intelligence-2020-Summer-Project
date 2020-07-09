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
import droneMain as drone
import multiprocessing

# Program Constants
SIMULATION_STEP = 50.0  # in milliseconds

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

        # Ground robot
        res, drone_target = sim.simxGetObjectHandle(
            clientID, 'youBot#0', sim.simx_opmode_oneshot_wait)
        if res != sim.simx_return_ok:
            print('Could not get handle to ground robot...')

        # Start main control loop
        print('Starting control loop...')

        # While connected to the simulator
        while (sim.simxGetConnectionId(clientID) != -1):
            start_ms = int(round(time.time() * 1000))

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

import vrep

propellers = ['propeller1Vel', 'propeller2Vel', 'propeller3Vel', 'propeller4Vel']

def init_rotors(clientID):
    # Clear all signals
    for i in range(len(propellers)):
        vrep.simxClearFloatSignal(clientID, propellers[i], vrep.simx_opmode_oneshot)

    # Set all propellers to zero
    for i in range(len(propellers)):
        vrep.simxSetFloatSignal(clientID, propellers[i], 0.0, vrep.simx_opmode_oneshot)

def move_rotors(clientID, propeller_vels):
        vrep.simxSetFloatSignal(clientID, propellers[0], propeller_vels[0], vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(clientID, propellers[1], propeller_vels[1], vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(clientID, propellers[2], propeller_vels[2], vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(clientID, propellers[3], propeller_vels[3], vrep.simx_opmode_oneshot)
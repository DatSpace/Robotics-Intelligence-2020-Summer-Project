#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sim
import time
import math


def initialize(droneTarget, drone):
    sim.setObjectParent(droneTarget, -1, true)

    propellerHandles = {-1, -1, -1, -1}

    for i in range(4):
        propellerHandles[i] = sim.getScriptHandle('Quadricopter_propeller_respondable'..i)

    heli = sim.getObjectAssociatedWithScript(sim.handle_self)

    particlesTargetVelocities = {0, 0, 0, 0}

    pParam = 2
    iParam = 0
    dParam = 0
    vParam = -2

    cumul = 0
    lastE = 0
    pAlphaE = 0
    pBetaE = 0
    psp2 = 0
    psp1 = 0

    prevEuler = 0


def controller():
    s = sim.getObjectSizeFactor(drone)

    pos = sim.getObjectPosition(drone, -1)

    '''Vertical control:'''
    targetPos = sim.getObjectPosition(droneTarget, -1)
    pos = sim.getObjectPosition(drone, -1)

    l = sim.getVelocity(heli)
    e = (targetPos[3]-pos[3])
    cumul = cumul + e
    pv = pParam * e
    thrust = 5.335 + pv + iParam * cumul + dParam * (e-lastE) + l[3] * vParam
    lastE = e

    '''Horizontal control:'''
    sp = sim.getObjectPosition(droneTarget, drone)
    m = sim.getObjectMatrix(drone, -1)
    vx = {1, 0, 0}
    vx = sim.multiplyVector(m, vx)
    vy = {0, 1, 0}
    vy = sim.multiplyVector(m, vy)
    alphaE = (vy[3] - m[12])
    alphaCorr = 0.25 * alphaE + 2.1 * (alphaE - pAlphaE)
    betaE = (vx[3] - m[12])
    betaCorr = -0.25 * betaE - 2.1 * (betaE-pBetaE)
    pAlphaE = alphaE
    pBetaE = betaE
    alphaCorr = alphaCorr + sp[2] * 0.005 + 1 * (sp[2] - psp2)
    betaCorr = betaCorr - sp[1] * 0.005 - 1 * (sp[1] - psp1)
    psp2 = sp[2]
    psp1 = sp[1]

    ''''Rotational control:'''
    euler = sim.getObjectOrientation(drone, droneTarget)
    rotCorr = euler[3] * 0.1 + 2 * (euler[3]-prevEuler)
    prevEuler = euler[3]

    ''''Decide of the motor velocities:'''
    particlesTargetVelocities[1] = thrust * \
        (1 - alphaCorr + betaCorr + rotCorr)
    particlesTargetVelocities[2] = thrust * \
        (1 - alphaCorr - betaCorr - rotCorr)
    particlesTargetVelocities[3] = thrust * \
        (1 + alphaCorr - betaCorr + rotCorr)
    particlesTargetVelocities[4] = thrust * \
        (1 + alphaCorr + betaCorr - rotCorr)

    '''Send the desired motor velocities to the 4 rotors:'''
    for i in range(4):
        sim.setScriptSimulationParameter(
            propellerHandles[i], 'particleVelocity', particlesTargetVelocities[i])

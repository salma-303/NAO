# -*- coding: utf-8 -*-
import qi
import argparse
import time
import math
from functools import partial
import numpy as np
import motion as mot

robotIP = "10.1.104.19"
ses = qi.Session()
ses.connect(robotIP)
per = qi.PeriodicTask()
motion = ses.service('ALMotion')
posture = ses.service('ALRobotPosture')

def kickBall(speed=2.0):
    # Activate Whole Body Balancer
    isEnabled = True
    motion.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName = "Fixed"
    supportLeg = "Legs"
    motion.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable = True
    supportLeg = "Legs"
    motion.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration = 1.0
    motion.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName = "Free"
    supportLeg = "RLeg"
    motion.wbFootState(stateName, supportLeg)

    # RLeg is optimized
    effectorName = "RLeg"
    axisMask = 63
    space = mot.FRAME_TORSO

    # Motion of the RLeg
    dx = 0.025  # translation axis X (meters)
    dz = 0.02  # translation axis Z (meters)
    dwy = 5.0 * math.pi / 180.0  # rotation axis Y (radian)

    times = [1.0, 1.4, 2.1]
    isAbsolute = False

    targetList = [
        [-0.7 * dx, 0.0, 1.1 * dz, 0.0, +dwy, 0.0],
        [+2.2 * dx, +dx, dz, 0.0, -dwy, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    ]

    # Adjust duration based on speed
    times = [t / speed for t in times]

    motion.positionInterpolation(effectorName, space, targetList,
                                 axisMask, times, isAbsolute)

    # Example showing how to Enable Effector Control as an Optimization
    isActive = False
    motion.wbEnableEffectorOptimization(effectorName, isActive)

    time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled = False
    motion.wbEnable(isEnabled)

    # send robot to Pose Init
    posture.goToPosture("StandInit", 0.5)

if __name__ == "__main__":
    speed = 2.0  # Adjust speed here (1.0 is the default)
    kickBall(speed)


    # posture_service.goToPosture("Crouch", 1.0)
                # time.sleep(1.0)
                # names = [
                #     "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw",
                #     "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw"
                # ]

                # angles = [
                #     math.radians(0), math.radians(20), math.radians(-90), math.radians(40), math.radians(0),
                #     math.radians(0), math.radians(-20), math.radians(90), math.radians(-40), math.radians(0)
                # ]

                # fractionMaxSpeed = 0.5
                # motion_service.changeAngles(names, angles, fractionMaxSpeed)
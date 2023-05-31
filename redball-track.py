#!/usr/bin/env python
# -*- encoding: UTF-8 -*-

"""Example: Use Tracking Module to Track a Red Ball"""

import qi
import argparse
import sys
import time


def main(session, ball_size):
    """
    This example shows how to use ALTracker with a red ball.
    """
    # Get the services ALTracker, ALMotion, and ALRobotPosture.
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    tracker_service = session.service("ALTracker")

    # First, wake up.
    motion_service.wakeUp()

    fraction_max_speed = 0.8
    # Go to the stand posture.
    posture_service.goToPosture("StandInit", fraction_max_speed)

    # Add a target to track.
    target_name = "RedBall"
    diameter_of_ball = ball_size
    tracker_service.registerTarget(target_name, diameter_of_ball)

    # Set the mode.
    mode = "Move"
    tracker_service.setMode(mode)

    # Then, start the tracker.
    tracker_service.track(target_name)

    print("ALTracker successfully started. Now, show a red ball to the robot!")
    print("Use Ctrl+C to stop this script.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        print("Stopping...")

    # Stop the tracker and go to the Sit posture.
    tracker_service.stopTracker()
    tracker_service.unregisterAllTargets()
    posture_service.goToPosture("Sit", fraction_max_speed)
    motion_service.rest()

    print("ALTracker stopped.")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="10.1.5.47",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    parser.add_argument("--ballsize", type=float, default=0.1,
                        help="Diameter of ball.")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    main(session, args.ballsize)
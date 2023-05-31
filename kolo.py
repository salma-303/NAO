# -*- coding: utf-8 -*-

import cv2
import imutils
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
from naoqi import ALProxy
from vision_definitions import k4VGA, kBGRColorSpace
import threading
import qi
import argparse
import sys
import time
import math
import signal
from functools import partial
import motion as mot

diameter = 0
radius = 0
stopped = False
NAO = "10.1.104.133"

cnt = -1

def kickBall(speed=2.0):
    motion_service.wakeUp()
    # Activate Whole Body Balancer
    isEnabled = True
    motion_service.wbEnable(isEnabled)

    # Legs are constrained fixed
    stateName = "Fixed"
    supportLeg = "Legs"
    motion_service.wbFootState(stateName, supportLeg)

    # Constraint Balance Motion
    isEnable = True
    supportLeg = "Legs"
    motion_service.wbEnableBalanceConstraint(isEnable, supportLeg)

    # Com go to LLeg
    supportLeg = "LLeg"
    duration = 1.0
    motion_service.wbGoToBalance(supportLeg, duration)

    # RLeg is free
    stateName = "Free"
    supportLeg = "RLeg"
    motion_service.wbFootState(stateName, supportLeg)

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

    motion_service.positionInterpolation(effectorName, space, targetList,
                                 axisMask, times, isAbsolute)

    # Example showing how to Enable Effector Control as an Optimization
    isActive = False
    motion_service.wbEnableEffectorOptimization(effectorName, isActive)

    time.sleep(1.0)

    # Deactivate Head tracking
    isEnabled = False
    motion_service.wbEnable(isEnabled)

    # send robot to Pose Init
    posture_service.goToPosture("StandInit", 0.5)


def rotate_head():
    global diameter

    """
    Rotate the NAO's head until the detect_red_ball function starts.
    """

    tts = ALProxy("ALTextToSpeech", NAO, 9559)
    tts.say("Searching")
    motion = ALProxy("ALMotion", NAO, 9559)

    # Specify the desired head rotation range
    min_yaw = -2.0
    max_yaw = 2.0

    # Set the initial angle to the minimum value
    angle = min_yaw

    # Set the initial rotation direction
    direction = 1  # 1 for left to right, -1 for right to left
    try:
        while diameter <= 20:
            print("Ball diameter: {}".format(diameter))
            
            # Rotate the head using the current angle
            motion.setAngles("HeadPitch", 0.5, 0.05)
            motion.setAngles("HeadYaw", angle, 0.04)

            # Increment the angle based on the current direction
            angle += 0.1 * direction

            # Check if the angle exceeds the range limits
            if angle > max_yaw:
                angle = max_yaw
                direction = -1  # Change direction to right to left
            elif angle < min_yaw:
                angle = min_yaw
                direction = 1  # Change direction to left to right

            time.sleep(0.1)

        # Stop the motion of the head joint
        joint_name = "HeadYaw"  # Replace with the actual joint name you want to stop

        # Get the current angle of the joint
        current_angle = motion.getAngles(joint_name, True)

        # Set the current angle to stop the motion
        motion.setAngles(joint_name, current_angle, 0.1)
    except KeyboardInterrupt:
        sys.exit()



def detect_red_ball():
    global diameter
    global radius
    
    # tts = ALProxy("ALTextToSpeech", NAO, 9559)
    # tts.say("Ready to detect a red ball")

    
    # Subscribe to the top and bottom cameras
    camera_index_top = 0
    # camera_index_bottom = 1

    # Create a proxy for ALVideoDevice
    name = "nao_opencv"
    video = ALProxy("ALVideoDevice", NAO, 9559)
    # subscribe to video device on a specific camera # BGR for opencv
    name_top = video.subscribeCamera("nao_opencv_top", camera_index_top, k4VGA, kBGRColorSpace, 30)
    print("Subscribed to top camera: ", name_top)

    # Subscribe to the bottom camera
    # name_bottom = video.subscribeCamera("nao_opencv_bottom", camera_index_bottom, k4VGA, kBGRColorSpace, 30)
    # print("Subscribed to bottom camera: ", name_bottom)

    try:
        frame_top = None
        # frame_bottom = None
        # ball_detected = False

        while True:
            key = cv2.waitKey(33) & 0xFF
            if key == ord("q") or key == 27:
                break

            # Obtain images from the top and bottom cameras
            alimg_top = video.getImageRemote(name_top)
            # alimg_bottom = video.getImageRemote(name_bottom)

            # Extract image fields
            width_top = alimg_top[0]
            height_top = alimg_top[1]
            imgbuffer_top = alimg_top[6]

            # width_bottom = alimg_bottom[0]
            # height_bottom = alimg_bottom[1]
            # imgbuffer_bottom = alimg_bottom[6]

            # Build OpenCV images
            if frame_top is None:
                frame_top = np.asarray(bytearray(imgbuffer_top), dtype=np.uint8)
                frame_top = frame_top.reshape((height_top, width_top, 3))
            else:
                frame_top.data = bytearray(imgbuffer_top)

            # if frame_bottom is None:
            #     frame_bottom = np.asarray(bytearray(imgbuffer_bottom), dtype=np.uint8)
            #     frame_bottom = frame_bottom.reshape((height_bottom, width_bottom, 3))
            # else:
            #     frame_bottom.data = bytearray(imgbuffer_bottom)

            # Convert BGR image to HSV color space
            hsv = cv2.cvtColor(frame_top, cv2.COLOR_BGR2HSV)
            # hsv = cv2.cvtColor(frame_bottom, cv2.COLOR_BGR2HSV)
            # Define lower and upper bounds for red color in HSV color space
            lower_red = np.array([0, 70, 50])
            upper_red = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            # Create a mask to filter out only red pixels
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            # Apply morphological operations to remove noise and fill in gaps
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.erode(mask, kernel, iterations=2)
            mask = cv2.dilate(mask, kernel, iterations=2)
            # Find contours in the mask and initialize the current (x, y) center of the ball
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            center = None
            # If at least one contour is found
            if len(contours) > 0:
                # Find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                diameter = radius * 2
            else:
                diameter = 0

            if radius > 0:
                cv2.circle(frame_top, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame_top, (int(x), int(y)), 5, (0, 0, 255), -1)
                # cv2.circle(frame_bottom, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # cv2.circle(frame_bottom, (int(x), int(y)), 5, (0, 0, 255), -1)
                center = (int(x), int(y))

            

            # Display the top and bottom camera images
            cv2.imshow("Top Camera", frame_top)
            # cv2.imshow("Bottom Camera", frame_top)

        # Check if the ball is in the center of the image
        if center is not None:
            # Calculate the difference between the center of the image and the center of the ball
            x_diff = center[0] - (width_top / 2)
            y_diff = center[1] - (height_top / 2)

            # If the ball is not in the center of the image, move the NAO's head to center the ball
            if abs(x_diff) > 20 or abs(y_diff) > 20:
                # Calculate the desired position for the NAO's head based on the difference between the center of the image and the center of the ball
                head_yaw = -0.3 * x_diff / (width_top / 2)
                head_pitch = -0.3 * y_diff / (height_top / 2)

                # Move the NAO's head to the desired position
                motion = ALProxy("ALMotion", NAO, 9559)
                names = ["HeadYaw", "HeadPitch"]
                angles = [head_yaw, head_pitch]
                fractionMaxSpeed = 0.1
                motion.setAngles(names, angles, fractionMaxSpeed)

        # Clear buffer and release camera subscriptions
        video.releaseImage(name_top)
        # video.releaseImage(name_bottom)
        tts.say("Stopping Camera")
        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        pass

    finally:
        # Unsubscribe from the cameras
        video.unsubscribe(name_top)
        # video.unsubscribe(name_bottom)

        diameter = 0
# # Call the detect_red_ball function
# detect_red_ball()
def detect_red_ball_bottom():
    global diameter
    global radius
    
    # tts = ALProxy("ALTextToSpeech", NAO, 9559)
    # tts.say("Ready to detect a red ball")

    # Subscribe to the bottom camera
    camera_index_bottom = 1

    # Create a proxy for ALVideoDevice
    name = "nao_opencv"
    video = ALProxy("ALVideoDevice", NAO, 9559)
    # subscribe to video device on the bottom camera # BGR for opencv
    name_bottom = video.subscribeCamera("nao_opencv_bottom", camera_index_bottom, k4VGA, kBGRColorSpace, 30)
    print("Subscribed to bottom camera: ", name_bottom)

    try:
        frame_bottom = None
        # ball_detected = False

        while True:
            key = cv2.waitKey(33) & 0xFF
            if key == ord("q") or key == 27:
                break

            # Obtain image from the bottom camera
            alimg_bottom = video.getImageRemote(name_bottom)

            # Extract image fields
            width_bottom = alimg_bottom[0]
            height_bottom = alimg_bottom[1]
            imgbuffer_bottom = alimg_bottom[6]

            # Build OpenCV image
            if frame_bottom is None:
                frame_bottom = np.asarray(bytearray(imgbuffer_bottom), dtype=np.uint8)
                frame_bottom = frame_bottom.reshape((height_bottom, width_bottom, 3))
            else:
                frame_bottom.data = bytearray(imgbuffer_bottom)

            # Convert BGR image to HSV color space
            hsv = cv2.cvtColor(frame_bottom, cv2.COLOR_BGR2HSV)

            # Define lower and upper bounds for red color in HSV color space
            lower_red = np.array([0, 70, 50])
            upper_red = np.array([10, 255, 255])
            lower_red2 = np.array([170, 70, 50])
            upper_red2 = np.array([180, 255, 255])
            # Create a mask to filter out only red pixels
            mask1 = cv2.inRange(hsv, lower_red, upper_red)
            mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask = cv2.bitwise_or(mask1, mask2)
            # Apply morphological operations to remove noise and fill in gaps
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.erode(mask, kernel, iterations=2)
            mask = cv2.dilate(mask, kernel, iterations=2)
            # Find contours in the mask and initialize the current (x, y) center of the ball
            contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = imutils.grab_contours(contours)
            center = None
            # If at least one contour is found
            if len(contours) > 0:
                # Find the largest contour in the mask, then use it to compute the minimum enclosing circle and centroid
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                M = cv2.moments(c)
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                diameter = radius * 2
            else:
                diameter = 0

            if radius > 5:
                cv2.circle(frame_bottom, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame_bottom, (int(x), int(y)), 5, (0, 0, 255), -1)
                center = (int(x), int(y))

            # Display the bottom camera image
            cv2.imshow("Bottom Camera", frame_bottom)

        # Check if the ball is in the center of the image
        if center is not None:
            # Calculate the difference between the center of the image and the center of the ball
            x_diff = center[0] - (width_bottom / 2)
            y_diff = center[1] - (height_bottom / 2)

            # If the ball is not in the center of the image, move the NAO's head to center the ball
            if abs(x_diff) > 20 or abs(y_diff) > 20:
                # Calculate the desired position for the NAO's head based on the difference between the center of the image and the center of the ball
                head_yaw = -0.3 * x_diff / (width_bottom / 2)
                head_pitch = -0.3 * y_diff / (height_bottom / 2)

                # Move the NAO's head to the desired position
                motion = ALProxy("ALMotion", NAO, 9559)
                names = ["HeadYaw", "HeadPitch"]
                angles = [head_yaw, head_pitch]
                fractionMaxSpeed = 0.1
                motion.setAngles(names, angles, fractionMaxSpeed)

        # Clear buffer and release camera subscription
        video.releaseImage(name_bottom)
        tts.say("Stopping Camera")
        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        pass

    finally:
        # Unsubscribe from the camera
        video.unsubscribe(name_bottom)

        diameter = 0

def track(session, ball_size):
    global diameter
    global radius
    # Get the services ALTracker, ALMotion, and ALRobotPosture.
    # motion_service = session.service("ALMotion")
    # posture_service = session.service("ALRobotPosture")
    tracker_service = session.service("ALTracker")

    # Add a target to track.
    target_name = "RedBall"
    diameter_of_ball = ball_size
    tracker_service.registerTarget(target_name, diameter_of_ball)

    # Set the mode.
    mode = "Move"

    tracker_service.setMode(mode)
    tracker_service.track(target_name)

    print("ALTracker successfully started. Now, show a red ball to the robot!")
    print("Use Ctrl+C to stop this script.")
    try:
        while True:
            if diameter == 0:
                tracker_service.stopTracker()
                tracker_service.unregisterAllTargets()
                thread0 = threading.Thread(target=rotate_head)
                thread0.start()
                break
            elif diameter < 320:
                tts = ALProxy("ALTextToSpeech", NAO, 9559)
                tts.say("ball detected")
                print("Ball tracker diameter: {}".format(diameter))
                time.sleep(1)
            else:
                tracker_service.stopTracker()
                tracker_service.unregisterAllTargets()
                tts = ALProxy("ALTextToSpeech", NAO, 9559)
                
                # Set the walking parameters
                x = 0.2  # Distance in meters
                y = 0.0
                theta = 0.0  # Angle in radians
                frequency = 0.8  # Speed in fraction of maximum speed

                # Make NAO walk
                motion_service.moveTo(x, y, theta, _async=True)  # Start walking asynchronously

                # Wait for the first step to complete
                motion_service.waitUntilMoveIsFinished()

                tts.say("kicking the ball")
                kickBall()
                time.sleep(2)
                if diameter < 50:
                    diameter=0
                    tracker_service.stopTracker()
                    tracker_service.unregisterAllTargets()
                    thread0 = threading.Thread(target=rotate_head)
                    thread0.start()
                    break

                # Restart rotating the head
                thread0 = threading.Thread(target=rotate_head)
                thread0.start()

                break

    except KeyboardInterrupt:
        print("\nInterrupted by user")
        print("Stopping...")

        tracker_service.stopTracker()
        tracker_service.unregisterAllTargets()
        tts = ALProxy("ALTextToSpeech", NAO, 9559)
        tts.say("Stop detecting")
        print("ALTracker stopped.")
        sys.exit()
    diameter = 0
    time.sleep(5)
    track(session, ball_size)


def signal_handler(signal, frame):
    global stopped
    global diameter
    stopped = True
    diameter = 0
    print("Stopping...")
    sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=NAO,
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    parser.add_argument("--ballsize", type=float, default=0.15,
                        help="Diameter of ball.")

    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    

    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    tracker_service = session.service("ALTracker")

    # First, wake up.
    motion_service.wakeUp()

    fraction_max_speed = 0.8
    # Go to the stand posture.
    posture_service.goToPosture("StandInit", fraction_max_speed)

    signal.signal(signal.SIGINT, signal_handler)

try:
    thread1 = threading.Thread(target=detect_red_ball)
    thread1.start()
    time.sleep(3)
    thread0 = threading.Thread(target=rotate_head)
    thread0.start()
    time.sleep(7)
    thread2 = threading.Thread(target=track, args=(session, args.ballsize, ))
    thread2.start()

    thread1.join()
    thread0.join()
    thread2.join()

    while not stopped:
        pass

except:
    pass

finally:
    tracker_service.stopTracker()
    tracker_service.unregisterAllTargets()
    # posture_service.goToPosture("Sit", fraction_max_speed)
    # motion_service.rest()
    tts = ALProxy("ALTextToSpeech", NAO, 9559)
    tts.say("Stop detecting")
    print("ALTracker stopped.")
    sys.exit(1)



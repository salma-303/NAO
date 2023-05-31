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
import random
import signal

diameter = 0
radius = 0
stopped = False
NAO = "10.1.5.45"

def rotate_head():
    global diameter
    """
    Rotate the NAO's head until the detect_red_ball function starts.
    """
    
    tts = ALProxy("ALTextToSpeech", NAO, 9559)
    tts.say("Rotating head")
    motion = ALProxy("ALMotion", NAO, 9559)
    while True:
        angle = random.uniform(-2, 2)
        # print("Ball diameter: {}".format(diameter))
        if diameter > 0:
            break
        else:
            motion.setAngles("HeadPitch", 0.3 , 0.05)
            motion.setAngles("HeadYaw", angle, 0.05)
        time.sleep(0.1) 

def detect_red_ball():
    global diameter
    global radius
    
    tts = ALProxy("ALTextToSpeech", NAO, 9559)
    tts.say("Ready to detect a red ball")

    
    camera_index = 1  

    # Create a proxy for ALVideoDevice
    name = "nao_opencv"
    video = ALProxy("ALVideoDevice", NAO, 9559)
    # subscribe to video device on a specific camera # BGR for opencv
    name = video.subscribeCamera(name, camera_index, k4VGA, kBGRColorSpace, 60)
    print("Subscribed name: ", name)

    try:
        frame = None
        # keep looping
        while True:
            key = cv2.waitKey(33) & 0xFF
            if key == ord("q") or key == 27:
                break
            # obtain image
            alimg = video.getImageRemote(name)
            # extract fields
            
            width = alimg[0]
            height = alimg[1]
            nchannels = alimg[2]
            imgbuffer = alimg[6]
            # build opencv image ( allocate on first pass )
            if frame is None:
                # print("Grabbed image: ", width, "x", height, "nchannels =", nchannels)
                frame = np.asarray(bytearray(imgbuffer), dtype=np.uint8)
                frame = frame.reshape((height, width, 3))
            else:
                frame.data = bytearray(imgbuffer)
            
            # Convert BGR image to HSV color space
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
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
                # print("Ball diameter: {}".format(diameter))

            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
                center = (int(x), int(y))

            # Display the resulting frame
            cv2.imshow("NAO Top Camera", frame)

        # Check if the ball is in the center of the image
        if center is not None:
            # Calculate the difference between the center of the image and the center of the ball
            x_diff = center[0] - (width / 2)
            y_diff = center[1] - (height / 2)

            # If the ball is not in the center of the image, move the NAO's head to center the ball
            if abs(x_diff) > 20 or abs(y_diff) > 20:
                # Calculate the desired position for the NAO's head based on the difference between the center of the image and the center of the ball
                head_yaw = -0.3 * x_diff / (width / 2)
                head_pitch = -0.3 * y_diff / (height / 2)

                # Move the NAO's head to the desired position
                motion = ALProxy("ALMotion", NAO, 9559)
                names = ["HeadYaw", "HeadPitch"]
                angles = [head_yaw, head_pitch]
                fractionMaxSpeed = 0.1
                motion.setAngles(names, angles, fractionMaxSpeed)

        # clear buffer
        video.releaseImage(name)
        # set frame to None for next iteration
        frame = None

    except KeyboardInterrupt:
        pass

    finally:
        # Unsubscribe from the video device
        video.unsubscribe(name)
        tts.say("Stopping Camera")
        cv2.destroyAllWindows()

def track (session, ball_size):
    global diameter
    global radius
        # Get the services ALTracker, ALMotion, and ALRobotPosture.
    motion_service = session.service("ALMotion")
    posture_service = session.service("ALRobotPosture")
    tracker_service = session.service("ALTracker")
    
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
            if diameter < 490:    
                print("Ball diameter: {}".format(diameter))
                time.sleep(1)
            else:
                tracker_service.stopTracker()
                tracker_service.unregisterAllTargets()
                # posture_service.goToPosture("StandInit", fraction_max_speed)
                tts = ALProxy("ALTextToSpeech", NAO, 9559)
                tts.say("Kicking the ball!")
               # Set the move configuration
                move_config = [
                    ["MaxStepX", 0.04],  # Max step size in meters for forward/backward movement
                    ["MaxStepTheta", 0.4],  # Max step size in radians for turning
                    ["StepHeight", 0.02],  # Height of each step in meters
                    ["TorsoWx", 0.0],  # Turn angle of the torso around the X-axis in radians
                    ["TorsoWy", 0.0],  # Turn angle of the torso around the Y-axis in radians
                ]

                
                backdistance = -0.04
                angle = 0.0  # Don't turn
                motion_service.moveToward(1, 0, 0.1)
                time.sleep(3)
                motion_service.move(backdistance, 0.0, angle, move_config)
                time.sleep(5)
                break
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        print("Stopping...")
    
        tracker_service.stopTracker()
        tracker_service.unregisterAllTargets()
        posture_service.goToPosture("Sit", fraction_max_speed)
        motion_service.rest()
        tts = ALProxy("ALTextToSpeech", NAO, 9559)
        tts.say("Stop detecting")
        print("ALTracker stopped.")
    time.sleep(5)
    track(session, args.ballsize)

def signal_handler(signal, frame):
    global stopped
    stopped = True
    print("Stopping...")
    sys.exit(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default=NAO,
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

    thread0 = threading.Thread(target=rotate_head)
    thread0.start()
    
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



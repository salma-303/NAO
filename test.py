import argparse
import cv2
import numpy as np
import qi
import sys
import threading
from vision_definitions import kVGA, kBGRColorSpace
from naoqi import ALProxy

def camera_thread(video, name):
    camera_index = 0  # top camera

    # subscribe to video device on a specific camera # BGR for opencv
    name = video.subscribeCamera(name, camera_index, kVGA, kBGRColorSpace, 30)
    print("Subscribed name: ", name)

    while True:
        alimg = video.getImageRemote(name)
        width = alimg[0]
        height = alimg[1]
        nchannels = alimg[2]
        imgbuffer = alimg[6]
        frame = np.asarray(bytearray(imgbuffer), dtype=np.uint8)
        frame = frame.reshape((height, width, 3))
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        cv2.imshow("NAO Top Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video.unsubscribe(name)
    cv2.destroyAllWindows()

def main(session, ballsize, effector):
    tracker_service = session.service("ALTracker")

    # set tracking mode
    tracker_service.setMode("Move")
    tracker_service.registerTarget("RedBall", ballsize)

    # set effector
    tracker_service.setEffector(effector)

    # start tracking
    tracker_service.track("RedBall")

    try:
        while True:
            ballPosition = tracker_service.getTargetPosition(0)
            if ballPosition:
                print("Ball position: x={}, y={}, z={}".format(ballPosition[0], ballPosition[1], ballPosition[2]))
                x, y = int(ballPosition[0] * 100), int(ballPosition[1] * 100)
                cv2.circle(frame, (x, y), 10, (0, 0, 255), -1)

            cv2.imshow("NAO Top Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        tracker_service.stopTracker()
        tracker_service.unregisterAllTargets()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="10.1.5.103",
                        help="Robot IP address. On robot or Local Naoqi: use '10.1.5.103'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")
    parser.add_argument("--ballsize", type=float, default=0.06,
                        help="Diameter of ball.")
    parser.add_argument("--effector", type=str, default="LArm",
                        choices=["Arms", "LArm", "RArm"],
                        help="Effector for tracking.")

    args = parser.parse_args()
    session = qi.Session()

    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)

    # Create a proxy for ALVideoDevice
    video_proxy = session.service("ALVideoDevice")

    # Subscribe to the top camera
    camera_index = 0
    resolution = kVGA
    color_space = kBGRColorSpace
    fps = 30
    video_client = "python_client"
    video_subscriber_id = video_proxy.subscribeCamera(
        video_client, camera_index, resolution, color_space, fps)

    # Create a proxy for ALTracker
    tracker_proxy = session.service("ALTracker")

    # Set the tracking mode to "Head"
    tracker_mode = "Head"
    tracker_proxy.setMode(tracker_mode)

    # Add target to track
    target_size = args.ballsize
    target_color = [255, 0, 0]  # Red in BGR format
    target_name = "RedBall"
    tracker_proxy.registerTarget(target_name, target_size)

    # Set the effector to track the target with
    effector_name = args.effector
    tracker_proxy.setEffector(effector_name)

    try:
        while True:
            key = cv2.waitKey(33) & 0xFF
            if key == ord("q") or key == 27:
                break

            # Get the latest image from the camera
            video_frame = video_proxy.getImageRemote(video_subscriber_id)

            # Extract the image properties
            width = video_frame[0]
            height = video_frame[1]
            channels = video_frame[2]
            imgbuffer = video_frame[6]

            # Convert the image to a numpy array
            img = np.ndarray(shape=(height, width, channels), dtype=np.uint8, buffer=imgbuffer)

            # Convert the image from BGR to HSV color space
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

            # Track the target
            success, target_position = tracker_proxy.getTargetPosition(target_name)
            if success:
                x, y, z = target_position
                print("Target position: x={}, y={}, z={}".format(x, y, z))

            # Display the image
            cv2.imshow("NAO Top Camera", img)

    finally:
        # Unsubscribe from the camera
        video_proxy.unsubscribe(video_subscriber_id)

        # Stop tracking the target
        tracker_proxy.stopTracker()
        tracker_proxy.unregisterAllTargets()
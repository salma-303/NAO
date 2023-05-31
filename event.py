import time
from naoqi import ALProxy
import cv2
import numpy as np
import imutils

NAO = "10.1.104.47"
k4VGA = 2
kBGRColorSpace = 13
radius=0

def detect_red_ball():
    global diameter
    global radius
    
    # tts = ALProxy("ALTextToSpeech", NAO, 9559)
    # tts.say("Ready to detect a red ball")

    
    # Subscribe to the top and bottom cameras
    camera_index_top = 0
    camera_index_bottom = 1

    # Create a proxy for ALVideoDevice
    name = "nao_opencv"
    video = ALProxy("ALVideoDevice", NAO, 9559)
    # subscribe to video device on a specific camera # BGR for opencv
    name_top = video.subscribeCamera("nao_opencv_top", camera_index_top, k4VGA, kBGRColorSpace, 30)
    print("Subscribed to top camera: ", name_top)

    # Subscribe to the bottom camera
    name_bottom = video.subscribeCamera("nao_opencv_bottom", camera_index_bottom, k4VGA, kBGRColorSpace, 30)
    print("Subscribed to bottom camera: ", name_bottom)

    try:
        frame_top = None
        frame_bottom = None
        ball_detected = False
        using_top_camera = True

        while True:
            key = cv2.waitKey(33) & 0xFF
            if key == ord("q") or key == 27:
                break

            # Obtain images from the top or bottom camera based on the current mode
            if using_top_camera:
                alimg = video.getImageRemote(name_top)
                camera_name = "Top Camera"
            else:
                alimg = video.getImageRemote(name_bottom)
                camera_name = "Bottom Camera"

            # Extract image fields
            width = alimg[0]
            height = alimg[1]
            imgbuffer = alimg[6]

            # Build OpenCV image
            if using_top_camera:
                if frame_top is None:
                    frame_top = np.asarray(bytearray(imgbuffer), dtype=np.uint8)
                    frame_top = frame_top.reshape((height, width, 3))
                else:
                    frame_top.data = bytearray(imgbuffer)
            else:
                if frame_bottom is None:
                    frame_bottom = np.asarray(bytearray(imgbuffer), dtype=np.uint8)
                    frame_bottom = frame_bottom.reshape((height, width, 3))
                else:
                    frame_bottom.data = bytearray(imgbuffer)

            # Convert BGR image to HSV color space
            if using_top_camera:
                frame = frame_top
            else:
                frame = frame_bottom
            
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
                ball_detected = True
            else:
                diameter = 0
                ball_detected = False

            if radius > 10:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
                center = (int(x), int(y))

            # Display the camera image
            cv2.imshow(camera_name, frame)

            # Check if the ball goes out of the frame
            if ball_detected and (center[0] < 0 or center[0] > width or center[1] < 0 or center[1] > height):
                # Switch to the other camera
                using_top_camera = not using_top_camera
                ball_detected = False
                if using_top_camera:
                    print("Switching to top camera")
                else:
                    print("Switching to bottom camera")

        # Clear buffer and release camera subscriptions
        video.releaseImage(name_top)
        video.releaseImage(name_bottom)
        # tts.say("Stopping Camera")
        cv2.destroyAllWindows()

    except KeyboardInterrupt:
        pass

    finally:
        # Unsubscribe from the cameras
        video.unsubscribe(name_top)
        video.unsubscribe(name_bottom)

        diameter = 0

# Call the detect_red_ball function
detect_red_ball()

import cv2
import imutils
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
from naoqi import ALProxy
from vision_definitions import kVGA, kBGRColorSpace

def detect_red_ball():
    tts = ALProxy("ALTextToSpeech", "10.1.5.142", 9559)
    tts.say("Ready to detect a red ball")

    NAO = "10.1.5.142"
    camera_index = 0  # top camera

    # Create a proxy for ALVideoDevice
    name = "nao_opencv"
    video = ALProxy("ALVideoDevice", NAO, 9559)
    # subscribe to video device on a specific camera # BGR for opencv
    name = video.subscribeCamera(name, camera_index, kVGA, kBGRColorSpace, 15)
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
                print("Ball diameter: {}".format(diameter))

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
        tts.say("Stopping detection")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    detect_red_ball()
# -*- coding: utf-8 -*-

import cv2
import imutils
from imutils.object_detection import non_max_suppression
from imutils import paths
import numpy as np
from naoqi import ALProxy
from vision_definitions import kVGA, kBGRColorSpace


NAO = "10.1.5.142"

if __name__ == "__main__":
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
            
            # show the frame
            cv2.imshow("NAO Top Camera", frame)
            frame = None
    finally:
        # unsubscribe
        video.unsubscribe(name)
        # Destroy all windows
        cv2.destroyAllWindows()

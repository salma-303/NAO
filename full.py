from naoqi import ALProxy
import cv2
import mediapipe as mp

# Set up Naoqi proxies
motion = ALProxy("ALMotion", "10.1.5.43", 9559)
posture = ALProxy("ALRobotPosture", "10.1.5.43", 9559)
camera = ALProxy("ALVideoDevice", "10.1.5.43", 9559)

# Set up MediaPipe Hands
mp_hands = mp.solutions.hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7, min_tracking_confidence=0.7)

# Set the resolution and color space of the video stream
resolution = 2    # VGA (640x480)
color_space = 11  # RGB

# Create a video module with a name and subscribe to the camera
module_name = "my_video_module"
camera_id = 0  # Top camera
camera.subscribeCamera(module_name, camera_id, resolution, color_space, 15)  # 15 fps

# Set up the OpenCV window
cv2.namedWindow("Nao Top Camera", cv2.WINDOW_NORMAL)

try:
    # Go to a standing posture
    posture.goToPosture("StandInit", 0.5)

    while True:
        # Retrieve a frame from the camera
        image = camera.getImageRemote(module_name)

        # Convert the image to a NumPy array
        image_width = image[0]
        image_height = image[1]
        image_channels = image[2]
        image_data = image[6]
        image_array = np.frombuffer(image_data, dtype=np.uint8).reshape((image_height, image_width, image_channels))

        # Run MediaPipe Hands on the image to detect the football
        results = mp_hands.process(cv2.cvtColor(image_array, cv2.COLOR_BGR2RGB))

        if results.multi_hand_landmarks:
            # Extract the coordinates of the first hand landmark
            hand_landmarks = results.multi_hand_landmarks[0]
            x, y, z = hand_landmarks.landmark[0].x, hand_landmarks.landmark[0].y, hand_landmarks.landmark[0].z

            # Print the coordinates of the football
            print("Football coordinates: ({:.2f}, {:.2f}, {:.2f})".format(x, y, z))

            # Move the robot's arms based on the coordinates of the football
            # Your code here

        # Display the frame in the OpenCV window
        cv2.imshow("Nao Top Camera", image_array)

        # Wait for a key press
        key = cv2.waitKey(1)
        if key == 27:  # Escape key
            break

finally:
    # Unsubscribe from the camera and destroy the OpenCV window
    camera.unsubscribe(module_name)
    cv2.destroyAllWindows()

import time
from naoqi import ALProxy


IP = "10.1.5.142" # Replace with your Nao's IP address
PORT = 9559

video = ALProxy("ALVideoDevice", IP, PORT)
motion = ALProxy("ALMotion", IP, PORT)

# Subscribe to the top camera
camera = video.subscribeCamera("test", 0, 2, 11, 500)

# Set the head pitch and yaw limits
motion.setStiffnesses("Head", 1.0)
motion.setAngles("Head", [0.0, 0.0], 0.1)
motion.setAngles("HeadYaw", [-90, 90], 0.1)
motion.setAngles("HeadPitch", [-45, 15], 0.1)

# Loop until the ball is found
while True:
    # Get the current image from the camera
    image = video.getImageRemote(camera)
    
    # Detect the ball in the image
    # Replace this with your own ball detection code
    ball_found = detect_ball(image)
    
    # If the ball is found, stop moving the head
    if ball_found:
        motion.stop("Head")
        break
    
    # Move the head in a random direction
    motion.setAngles("HeadYaw", [90, -90], 0.5)
    motion.setAngles("HeadPitch", [-30, 10], 0.5)
    time.sleep(1.0)

# Unsubscribe from the camera
video.unsubscribe(camera)

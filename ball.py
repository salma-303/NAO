# Import necessary libraries and modules
import time
from naoqi import ALProxy

# Connect to the robot
ip = "10.1.5.43" # Replace with your robot's IP address
port = 9559
redball_proxy = ALProxy("ALRedBallDetection", ip, port)

# Set detection parameters
redball_proxy.subscribe("Test_RedBall", 500, 0.0)
redball_proxy.setParameter("TrackingEnabled", 1)

# Start detection loop
try:
    while True:
        time.sleep(1)
        balls = redball_proxy.getBallPosition()
        if balls:
            print("Detected %d balls:" % len(balls))
            for ball in balls:
                print(ball)
        else:
            print("No balls detected")
except KeyboardInterrupt:
    print("Detection stopped by user")

# Unsubscribe from detection
redball_proxy.unsubscribe("Test_RedBall")

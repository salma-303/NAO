import cv2

# Create a VideoCapture object
cap = cv2.VideoCapture(0)

# Check if camera is opened correctly
if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Loop over the frames from the camera
while True:
    # Read a frame from the camera
    ret, frame = cap.read()

    # If frame is read correctly, display it
    if ret:
        cv2.imshow("Camera", frame)

    # Wait for key press
    key = cv2.waitKey(1)

    # If "q" is pressed, exit the loop
    if key == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

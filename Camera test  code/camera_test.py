import cv2
import numpy as np
import time
import argparse

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=480,
    capture_height=270,
    display_width=480,
    display_height=270,
    framerate=15,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

# Ball detection using color filtering (for blue color)
def detect_blue_ball(frame):
    # Convert the frame to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define the lower and upper range for the blue color in HSV
    lower_blue = np.array([0, 187, 53])
    upper_blue = np.array([75, 255, 176])

    frame_HSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Create a mask that isolates the blue colors in the frame
    mask = cv2.inRange(frame_HSV, lower_blue, upper_blue)

    # Bitwise-AND mask and original frame to show the blue ball
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('masked',mask)

    # Detect circles using HoughCircles on the masked image
    gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (15, 15), 0)

    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50, param1=100, param2=30, minRadius=20, maxRadius=100)

    num_circles = 0
    # If circles are detected, draw them on the frame
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            # Draw the circle
            cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
            # Draw a rectangle to show the center of the circle
            cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)
        num_circles = len(circles)

    return frame, num_circles

# Initialize video capture from the webcam
# cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)
parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
parser.add_argument('--camera', help='Camera divide number.', default=0, type=int)
args = parser.parse_args()
cap = cv2.VideoCapture(args.camera)

# Variable to keep track of how many frames with balls were saved
image_counter = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect the ball in the frame
    frame_with_ball, balls_detected = detect_blue_ball(frame)
    print(balls_detected)
    # If balls are detected, save the frame as an image
    if balls_detected > 0:
        image_filename = f"photos/detected_balls_{image_counter}.jpg"
        # cv2.imwrite(image_filename, frame_with_ball)
        print(f"Saved {image_filename} with {balls_detected} ball(s) detected.")
        image_counter += 1

    # Display the result (optional, you can uncomment this if needed)
    cv2.imshow('Ball Detection', frame_with_ball)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(1)

# Release the video capture and close windows
cap.release()
cv2.destroyAllWindows()

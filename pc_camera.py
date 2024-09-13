import cv2
from ultralytics import YOLO

# Load the pre-trained YOLOv8 model
model = YOLO('arkmodel3.pt')  # or 'path/to/your/custom_model.pt' if using a custom model

# Open a video capture feed (0 for webcam or use a path for a video file)
cap = cv2.VideoCapture(0)  # Use a path like 'video.mp4' to load a video file

# Check if the video capture feed is open
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while cap.isOpened():
    ret, frame = cap.read()
    
    if not ret:
        print("Error: Failed to read the frame.")
        break
    
    # Run YOLOv8 detection on the current frame
    results = model(frame)
    
    # Draw bounding boxes and labels on the frame
    annotated_frame = results[0].plot()

    # Display the frame with detections
    cv2.imshow('YOLOv8 Object Detection', annotated_frame)

    # Press 'q' to quit the video stream
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()

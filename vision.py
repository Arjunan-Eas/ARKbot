import tensorflow as tf
import numpy as np
import cv2

# Load the pre-trained SSD-MobileNet model from TensorFlow Hub
model_dir = 'ssd_mobilenet_v2_coco_2018_03_29'
model = tf.saved_model.load(f'{model_dir}/saved_model')

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

# Define the inference function
def run_inference_for_single_image(image):
    # Convert the image to a tensor and prepare it for the model
    input_tensor = tf.convert_to_tensor(image)
    input_tensor = input_tensor[tf.newaxis, ...]

    # Perform inference
    model_fn = model.signatures['serving_default']
    output_dict = model_fn(input_tensor)

    # Convert output to numpy arrays
    num_detections = int(output_dict['num_detections'].numpy())
    output_dict = {key:value.numpy() for key,value in output_dict.items()}
    output_dict['num_detections'] = num_detections
    output_dict['detection_classes'] = output_dict['detection_classes'][0].astype(np.uint8)
    output_dict['detection_boxes'] = output_dict['detection_boxes'][0]
    output_dict['detection_scores'] = output_dict['detection_scores'][0]

    return output_dict

# Function to visualize the results
def visualize_results(image, output_dict):
    for i in range(output_dict['num_detections']):
        if output_dict['detection_scores'][i] > 0.5:
            # Extract box coordinates
            box = output_dict['detection_boxes'][i]
            ymin, xmin, ymax, xmax = box
            (left, right, top, bottom) = (xmin * image.shape[1], xmax * image.shape[1], ymin * image.shape[0], ymax * image.shape[0])

            # Draw bounding box on the image
            cv2.rectangle(image, (int(left), int(top)), (int(right), int(bottom)), (0, 255, 0), 2)
            label = f'Class: {output_dict["detection_classes"][i]}, Score: {output_dict["detection_scores"][i]:.2f}'
            cv2.putText(image, label, (int(left), int(top) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return image

# Capture video from webcam
cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=0), cv2.CAP_GSTREAMER)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference
    output_dict = run_inference_for_single_image(frame)

    # Visualize the results
    frame_with_detections = visualize_results(frame, output_dict)

    # Display the result
    cv2.imshow('SSD-MobileNet Object Detection', frame_with_detections)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

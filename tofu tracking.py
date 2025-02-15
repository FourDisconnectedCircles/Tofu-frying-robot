from ultralytics import YOLO
import cv2
import numpy as np
import serial
import time

# Initialize YOLO model with the provided .pt file for tofu detection
model = YOLO('best.pt')

# Define the known width of the tofu in millimeters
known_width_mm = 40.0

# Initialize video capture using webcam
cap = cv2.VideoCapture(1)  # Change to 0 if camera index 1 doesn't work

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Get frame dimensions
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

# Calculate the center of the frame
frame_center_x = frame_width // 2
frame_center_y = frame_height // 2

# Define the size of the blue square
blue_square_size = 20

# Calculate the top-left and bottom-right coordinates of the blue square
blue_square_top_left = (
    frame_center_x - blue_square_size // 2,
    frame_center_y - blue_square_size // 2,
)
blue_square_bottom_right = (
    frame_center_x + blue_square_size // 2,
    frame_center_y + blue_square_size // 2,
)

# Attempt to initialize the serial connection with Arduino
try:
    ser = serial.Serial('COM5', 115200)  # Use the correct COM port for your setup
    time.sleep(2)  # Wait for the serial connection to initialize
    arduino_connected = True
    print("Arduino connected")
except serial.SerialException:
    arduino_connected = False
    print("Arduino not connected")

# Function to send coordinates to Arduino
def send_coordinates(x_mm, y_mm, z):
    coords = f"{x_mm:.2f},{y_mm:.2f},{z}\n"
    if arduino_connected:
        ser.write(coords.encode())
    print(f"Sent to Arduino: {coords}")  # Print the coordinates being sent

# Variables to track lock state and timing
lock_coordinates = False
lock_start_time = 0
lock_duration = 4  # Duration to check if tofu is inside the blue square in seconds
action_duration = 7  # Duration to hold z = 1 after being set
post_action_duration = 5  # Duration to hold z = 0 after action in seconds
post_action_start_time = 0

# Initialize z to a default value
z = 0
in_action_phase = False
in_post_action_phase = False

# Main loop for processing frames
while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        break

    # Perform object detection
    results = model(frame)

    # Variables to track the largest detected tofu
    max_confidence = 0
    max_box = None
    max_class_id = None

    # Process detection results
    for result in results:
        boxes = result.boxes
        for box in boxes:
            # Extract class ID, confidence, and box coordinates
            class_id = int(box.cls)
            confidence = box.conf
            b = box.xyxy[0]  # Bounding box coordinates (xmin, ymin, xmax, ymax)

            # Update max confidence and box if this is the most confident detection
            if confidence > max_confidence:
                max_confidence = confidence
                max_box = b
                max_class_id = class_id

    # Check the current time for timing calculations
    current_time = time.time()

    if max_box is not None:
        # Display only the bounding box with the highest confidence score
        class_name = model.names[max_class_id]

        # Draw bounding box around the detected tofu
        x_min, y_min, x_max, y_max = map(int, max_box)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        # Draw class name above the bounding box
        cv2.putText(frame, class_name, (x_min, y_min - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Calculate the pixel width of the bounding box
        pixel_width = x_max - x_min

        # Calculate the conversion factor from pixels to millimeters
        conversion_factor = known_width_mm / pixel_width

        # Calculate the center of the bounding box
        tofu_center_x = (x_min + x_max) // 2
        tofu_center_y = (y_min + y_max) // 2

        # Draw a circle at the centroid of the bounding box
        cv2.circle(frame, (tofu_center_x, tofu_center_y), 5, (255, 255, 255), -1)

        # Calculate offsets from the center of the frame
        offset_x = tofu_center_x - frame_center_x
        offset_y = tofu_center_y - frame_center_y

        # Convert offsets from pixels to millimeters
        offset_x_mm = offset_x * conversion_factor
        offset_y_mm = offset_y * conversion_factor

        # Print coordinates for debugging
        print(f'Offset X: {offset_x_mm:.2f} mm, Offset Y: {offset_y_mm:.2f} mm')

        # Determine if the centroid is within the blue square
        in_blue_square = (blue_square_top_left[0] <= tofu_center_x <= blue_square_bottom_right[0] and
                          blue_square_top_left[1] <= tofu_center_y <= blue_square_bottom_right[1])

        if in_blue_square:
            if not lock_coordinates:
                # Start timing when tofu first enters the square
                lock_start_time = current_time
                lock_coordinates = True
            elif current_time - lock_start_time >= lock_duration:
                # Determine if the centroid is within the green square
                # Placeholder for green square check
                # Define your green square coordinates here
                green_square_top_left = (frame_center_x - 40, frame_center_y - 40)
                green_square_bottom_right = (frame_center_x + 40, frame_center_y + 40)
                in_green_square = (green_square_top_left[0] <= tofu_center_x <= green_square_bottom_right[0] and
                                   green_square_top_left[1] <= tofu_center_y <= green_square_bottom_right[1])
                if in_green_square:
                    if not in_action_phase:
                        z = 1
                        send_coordinates(0, 0, z)  # Send grab command to Arduino
                        in_action_phase = True  # Enter action phase
                        lock_start_time = current_time  # Restart the timer for action phase
                        post_action_start_time = 0  # Reset post-action timing
        else:
            # Reset the lock if tofu moves out of the square
            lock_coordinates = False

        # Manage the action and post-action phases
        if in_action_phase:
            # If we're in the action phase, check if the action duration is complete
            if current_time - lock_start_time >= action_duration:
                # Action duration complete, switch to post-action phase
                z = 0
                send_coordinates(0, 0, z)  # Send rest state to Arduino
                post_action_start_time = current_time  # Start the post-action rest period
                in_action_phase = False  # Exit the action phase
                in_post_action_phase = True  # Enter post-action phase

        if in_post_action_phase:
            # In post-action phase, check if the rest duration is complete
            if current_time - post_action_start_time >= post_action_duration:
                # Post-action duration complete, exit post-action phase
                in_post_action_phase = False
                # Continue tracking tofu with the rest state
                send_coordinates(offset_x_mm, offset_y_mm, 0)

        # Send coordinates and z value to Arduino continuously when not locked or in post-action phase
        if not lock_coordinates and not in_action_phase and not in_post_action_phase:
            send_coordinates(offset_x_mm, offset_y_mm, z)

    else:
        offset_x_mm, offset_y_mm = 0, 0
        print('Tofu not detected')
        cv2.putText(frame, 'Tofu not detected', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Draw a blue square in the center of the frame
    cv2.rectangle(frame, blue_square_top_left, blue_square_bottom_right, (255, 0, 0), 2)

    # Draw a red dot at the center of the frame
    cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 0, 255), -1)

    # Display the offset values on the image (always show)
    cv2.putText(frame, f'Offset X: {offset_x_mm:.2f} mm', (10, frame_height - 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f'Offset Y: {offset_y_mm:.2f} mm', (10, frame_height - 40),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f'Z: {z}', (10, frame_height - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Display the original frame with annotations
    cv2.imshow('Tofu Detection', frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture object and close all windows
cap.release()
cv2.destroyAllWindows()

# Close the serial connection if it was opened
if arduino_connected:
    ser.close()

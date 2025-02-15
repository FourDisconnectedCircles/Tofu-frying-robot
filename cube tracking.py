import cv2
import numpy as np
import serial
import time

# Attempt to initialize the serial connection with Arduino
try:
    ser = serial.Serial('COM5', 115200)  # Use the correct COM port for your setup
    time.sleep(2)  # Wait for the serial connection to initialize
    arduino_connected = True
    print("Arduino connected")
except serial.SerialException:
    arduino_connected = False
    print("Arduino not connected")

# Define the known width of the cube in millimeters
known_width_mm = 20.0

# Initialize the camera
cap = cv2.VideoCapture(2)  # Change to 0 if camera index 1 doesn't work

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
blue_square_size = 30

# Calculate the top-left and bottom-right coordinates of the blue square
blue_square_top_left = (
    frame_center_x - blue_square_size // 2,
    frame_center_y - blue_square_size // 2,
)
blue_square_bottom_right = (
    frame_center_x + blue_square_size // 2,
    frame_center_y + blue_square_size // 2,
)

# Function to send coordinates to Arduino
def send_coordinates(x_mm, y_mm, z):
    coords = f"{x_mm:.2f},{y_mm:.2f},{z}\n"
    if arduino_connected:
        ser.write(coords.encode())
    print(f"Sent to Arduino: {coords}")  # Print the coordinates being sent

# Rate limit settings
rate_limit_time = 0.01  # Minimum time between sends in seconds
last_send_time = time.time()

# Create trackbars for dynamic threshold adjustment (optional, for real-time tuning)
def nothing(x):
    pass

cv2.namedWindow('Settings')
cv2.createTrackbar('Lower Black', 'Settings', 0, 255, nothing)
cv2.createTrackbar('Upper Black', 'Settings', 150, 255, nothing)

# Main loop for processing frames
while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        break

    # Convert the image to grayscale
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    gray_img = cv2.GaussianBlur(gray_img, (5, 5), 0)

    # Use adaptive thresholding to handle variable lighting
    adaptive_thresh = cv2.adaptiveThreshold(gray_img, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

    # Optionally adjust thresholds using trackbars
    lower_black = cv2.getTrackbarPos('Lower Black', 'Settings')
    upper_black = cv2.getTrackbarPos('Upper Black', 'Settings')

    # Create a mask to isolate the black color
    black_mask = cv2.inRange(gray_img, lower_black, upper_black)

    # Combine the masks for better results
    combined_mask = cv2.bitwise_and(adaptive_thresh, black_mask)

    # Use morphological operations to clean up the mask
    kernel = np.ones((3, 3), np.uint8)
    cleaned_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    cleaned_mask = cv2.morphologyEx(cleaned_mask, cv2.MORPH_OPEN, kernel, iterations=1)

    # Find contours in the cleaned mask
    contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Variable to track if the cube is detected
    cube_detected = False

    # Process each contour
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)
        if area > 300:  # Adjust this threshold based on your cube size
            # Approximate the contour to a polygon
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the approximated polygon is a rectangle
            if 4 <= len(approx) <= 6:  # Allow for some margin of error
                # Get the bounding rectangle for the contour
                x, y, w, h = cv2.boundingRect(approx)

                # Ensure the contour is approximately square and size is reasonable
                if 0.8 < w / float(h) < 1.2:  # Use aspect ratio check
                    # Draw the detected square on the frame
                    cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)

                    # Calculate the pixel width of the cube
                    pixel_width = w

                    # Calculate the conversion factor from pixels to millimeters
                    conversion_factor = known_width_mm / pixel_width

                    # Display the pixel width on the image
                    cv2.putText(frame, f'Width: {pixel_width} px', (x, y - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # Display the calculated conversion factor
                    cv2.putText(frame, f'1 mm = {conversion_factor:.2f} px', (x, y - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

                    # Calculate moments of the contour to find its centroid
                    moments_data = cv2.moments(contour)
                    if moments_data["m00"] != 0:
                        cube_center_x = int(moments_data['m10'] / moments_data['m00'])
                        cube_center_y = int(moments_data['m01'] / moments_data['m00'])

                        # Draw a circle at the centroid of the contour
                        cv2.circle(frame, (cube_center_x, cube_center_y), 5, (255, 255, 255), -1)

                        # Calculate offsets from the center of the frame
                        offset_x = cube_center_x - frame_center_x
                        offset_y = cube_center_y - frame_center_y

                        # Convert offsets from pixels to millimeters
                        offset_x_mm = offset_x / conversion_factor
                        offset_y_mm = offset_y / conversion_factor

                        # Print coordinates for debugging
                        print(f'Offset X: {offset_x_mm:.2f} mm, Offset Y: {offset_y_mm:.2f} mm')

                        # Determine if the centroid is within the blue square
                        if (blue_square_top_left[0] <= cube_center_x <= blue_square_bottom_right[0] and
                            blue_square_top_left[1] <= cube_center_y <= blue_square_bottom_right[1]):
                            # Set z to 1 if the cube is inside the blue square
                            z = 1
                            # Send (0, 0) if the cube is inside the blue square
                            offset_x_mm = 0
                            offset_y_mm = 0
                        else:
                            # Set z to 0 if the cube is outside the blue square
                            z = 0

                        # Send coordinates and z value to Arduino if enough time has passed
                        current_time = time.time()
                        if current_time - last_send_time > rate_limit_time:
                            send_coordinates(offset_x_mm, offset_y_mm, z)
                            last_send_time = current_time

                        # Display the offset values on the image
                        cv2.putText(frame, f'Offset X: {offset_x_mm:.2f} mm', (10, frame_height - 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(frame, f'Offset Y: {offset_y_mm:.2f} mm', (10, frame_height - 40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(frame, f'Z: {z}', (10, frame_height - 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # Mark the cube as detected
                    cube_detected = True

                    break  # Assuming there's only one cube, break after finding it

    if not cube_detected:
        cv2.putText(frame, 'Cube not detected', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Draw a blue square in the center of the frame
    cv2.rectangle(frame, blue_square_top_left, blue_square_bottom_right, (255, 0, 0), 2)

    # **Draw a red dot at the center of the frame**
    cv2.circle(frame, (frame_center_x, frame_center_y), 5, (0, 0, 255), -1)

    # Display the original frame with drawn contours and text
    cv2.imshow("Frame", frame)

    # Display the mask for debugging purposes
    cv2.imshow("Mask", cleaned_mask)

    # Check for user input to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

# Close the serial connection if it was opened
if arduino_connected:
    ser.close()

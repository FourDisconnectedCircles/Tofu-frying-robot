import cv2
import numpy as np

# Define the known width of the cube in millimeters
known_width_mm = 20.0

# Initialize the camera
cap = cv2.VideoCapture(1)  # Change to 0 if camera index 1 doesn't work

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Main loop for processing frames
while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read frame.")
        break

    # Convert the image to grayscale
    gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Define lower and upper bounds for white color in grayscale
    lower_white = 200  # Adjust this value according to your threshold
    upper_white = 255  # Adjust this value according to your threshold

    # Create a mask to isolate the area surrounded by white color
    white_mask = cv2.inRange(gray_img, lower_white, upper_white)

    # Invert the mask to focus on the white areas
    white_mask = cv2.bitwise_not(white_mask)

    # Apply the mask to the grayscale image
    masked_gray_img = cv2.bitwise_and(gray_img, gray_img, mask=white_mask)

    # Define lower and upper bounds for black color in grayscale
    lower_black = 5  # Adjust this value according to your threshold
    upper_black = 150  # Adjust this value according to your threshold

    # Create a mask to isolate the black color within the masked region
    black_mask = cv2.inRange(masked_gray_img, lower_black, upper_black)

    # Find contours in the black mask
    contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Variable to track if the cube is detected
    cube_detected = False

    # Process each contour
    for contour in contours:
        # Calculate the area of the contour
        area = cv2.contourArea(contour)
        if area > 500:  # Adjust this threshold based on your cube size
            # Approximate the contour to a polygon
            epsilon = 0.04 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            # Check if the approximated polygon is a rectangle
            if len(approx) == 4:
                # Get the bounding rectangle for the contour
                x, y, w, h = cv2.boundingRect(approx)

                # Ensure the contour is approximately square and size is reasonable
                if abs(w - h) < 10:  # Allow for some margin
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
                        center_x = int(moments_data['m10'] / moments_data['m00'])
                        center_y = int(moments_data['m01'] / moments_data['m00'])

                        # Draw a circle at the centroid of the contour
                        cv2.circle(frame, (center_x, center_y), 5, (255, 255, 255), -1)

                    # Mark the cube as detected
                    cube_detected = True

                    break  # Assuming there's only one cube, break after finding it

    if not cube_detected:
        cv2.putText(frame, 'Cube not detected', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Display the masks for debugging
    cv2.imshow('White Mask', white_mask)
    #cv2.imshow('Masked Gray Image', masked_gray_img)
    cv2.imshow('Black Mask', black_mask)

    # Show the frame with the detected cube
    cv2.imshow('Cube Tracking', frame)

    # Exit loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cap.release()
cv2.destroyAllWindows()

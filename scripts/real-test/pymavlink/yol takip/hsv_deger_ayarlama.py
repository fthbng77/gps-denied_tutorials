import cv2
import numpy as np

def nothing(x):
    pass

# Create a window
cv2.namedWindow('HSV Adjustments')

# create trackbars for color change
cv2.createTrackbar('H Lower', 'HSV Adjustments', 0, 179, nothing)
cv2.createTrackbar('H Upper', 'HSV Adjustments', 179, 179, nothing)
cv2.createTrackbar('S Lower', 'HSV Adjustments', 0, 255, nothing)
cv2.createTrackbar('S Upper', 'HSV Adjustments', 255, 255, nothing)
cv2.createTrackbar('V Lower', 'HSV Adjustments', 0, 255, nothing)
cv2.createTrackbar('V Upper', 'HSV Adjustments', 255, 255, nothing)

# Read an image or capture video
cap = cv2.VideoCapture(2)  # Use 0 for webcam or provide video file path

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    if not ret:
        break

    # Convert the frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Get the current positions of the trackbars
    hL = cv2.getTrackbarPos('H Lower', 'HSV Adjustments')
    hU = cv2.getTrackbarPos('H Upper', 'HSV Adjustments')
    sL = cv2.getTrackbarPos('S Lower', 'HSV Adjustments')
    sU = cv2.getTrackbarPos('S Upper', 'HSV Adjustments')
    vL = cv2.getTrackbarPos('V Lower', 'HSV Adjustments')
    vU = cv2.getTrackbarPos('V Upper', 'HSV Adjustments')

    # Define the range of colors in HSV
    lower_hsv = np.array([hL, sL, vL])
    upper_hsv = np.array([hU, sU, vU])

    # Create a mask with the defined HSV range
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
    res = cv2.bitwise_and(frame, frame, mask=mask)

    # Display the result
    cv2.imshow('HSV Adjustments', res)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()


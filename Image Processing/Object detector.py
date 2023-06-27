import cv2
import numpy as np

def nothing(x):
    pass


cam = cv2.VideoCapture(0)


detalization = 10
approximation_coef = 0.0
morphology_coefficient = 11


# Create a window
cv2.namedWindow('image')

# create trackbars for color change
cv2.createTrackbar('HMin','image',0,179,nothing) # Hue is from 0-179 for Opencv
cv2.createTrackbar('SMin','image',0,255,nothing)
cv2.createTrackbar('VMin','image',0,255,nothing)
cv2.createTrackbar('HMax','image',0,179,nothing)
cv2.createTrackbar('SMax','image',0,255,nothing)
cv2.createTrackbar('VMax','image',0,255,nothing)

#lower = np.array([17, 71, 139])
#upper = np.array([40, 182, 255])
# Set default value for MIN HSV trackbars.
cv2.setTrackbarPos('HMin', 'image', 17)
cv2.setTrackbarPos('SMin', 'image', 71)
cv2.setTrackbarPos('VMin', 'image', 139)
# Set default value for MAX HSV trackbars.
cv2.setTrackbarPos('HMax', 'image', 40)
cv2.setTrackbarPos('SMax', 'image', 182)
cv2.setTrackbarPos('VMax', 'image', 255)

# Initialize to check if HSV min/max value changes
hMin = sMin = vMin = hMax = sMax = vMax = 0
phMin = psMin = pvMin = phMax = psMax = pvMax = 0


while (True):

    # Capture the video frame
    # by frame
    ret, image = cam.read()
    frame = image

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # get current positions of all trackbars
    hMin = cv2.getTrackbarPos('HMin', 'image')
    sMin = cv2.getTrackbarPos('SMin', 'image')
    vMin = cv2.getTrackbarPos('VMin', 'image')

    hMax = cv2.getTrackbarPos('HMax', 'image')
    sMax = cv2.getTrackbarPos('SMax', 'image')
    vMax = cv2.getTrackbarPos('VMax', 'image')

    # Set minimum and max HSV values to display
    lower = np.array([hMin, sMin, vMin])
    upper = np.array([hMax, sMax, vMax])




    mask = cv2.inRange(hsv, lower, upper)


    # convert the grayscale image to binary image
    ret, thresh = cv2.threshold(mask, 127, 255, 0)
    contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    all_areas = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        all_areas.append(area)


    largest_item = sorted(contours, key=cv2.contourArea, reverse=True)[0]


    # calculate moments for each contour
    M = cv2.moments(largest_item)

    # calculate x,y coordinate of center
    if M["m00"] != 0:
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])
    else:
        cX, cY = 0, 0
    cv2.circle(frame, (cX, cY), 5, (255, 255, 255), -1)
    cv2.putText(frame, "centroid", (cX - 25, cY - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)


    cv2.drawContours(frame, contours, -1, (0, 255, 0), 3)

    # Display the resulting frame
    cv2.imshow('initial frame', frame)
    cv2.imshow("hsv", hsv)
    cv2.imshow('mask', mask)

    filename = 'example 1'

    if cv2.waitKey(1) & 0xFF == ord('w'):
        cv2.imwrite(filename + '-initial_picture.png', image)
        cv2.imwrite(filename + '-hsv.png', hsv)
        cv2.imwrite(filename + '-mask.png', mask)

    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# After the loop release the cap object
cam .release()
# Destroy all the windows
cv2.destroyAllWindows()
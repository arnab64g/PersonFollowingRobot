import cv2


def MarkerBorder(image, top_left, top_right, bottom_left, bottom_right):

    cv2.line(image, top_left, top_right, (0, 255, 0), 2)
    cv2.line(image, top_right, bottom_right, (0, 255, 0), 2)
    cv2.line(image, bottom_right, bottom_left, (0, 255, 0), 2)
    cv2.line(image, bottom_left, top_left, (0, 255, 0), 2)


def MarkerCenter(image, c_x, c_y):
    cv2.circle(image, (c_x, c_y), 4, (0, 0, 255), -1)
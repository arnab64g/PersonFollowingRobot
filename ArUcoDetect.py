import cv2
import math

ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
}


def get_max_size(top_left, top_right, bottom_left, bottom_right):
    a = math.sqrt(pow(top_left[0] - top_right[0], 2) + pow(top_left[1] - top_right[1], 2))
    b = math.sqrt(pow(bottom_left[0] - bottom_right[0], 2) + pow(bottom_left[1] - bottom_right[1], 2))
    c = math.sqrt(pow(top_left[0] - bottom_left[0], 2) + pow(top_left[1] - bottom_left[1], 2))
    d = math.sqrt(pow(top_right[0] - bottom_right[0], 2) + pow(top_right[1] - bottom_right[1], 2))
    ar = [int(a), int(b), int(c), int(d)]
    ar.sort()
    return ar[3]


def aruco_display(corners, ids, image):
    if len(corners) > 0:
        ids = ids.flatten()
        
        for (markerCorner, markerID) in zip(corners, ids):
            corners = markerCorner.reshape((4, 2))
            (top_left, top_right, bottom_right, bottom_left) = corners
            top_right = (int(top_right[0]), int(top_right[1]))
            bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
            bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
            top_left = (int(top_left[0]), int(top_left[1]))
            maxl = get_max_size(top_left, top_right, bottom_left, bottom_right)
            cv2.line(image, top_left, top_right, (0, 255, 0), 2)
            cv2.line(image, top_right, bottom_right, (0, 255, 0), 2)
            cv2.line(image, bottom_right, bottom_left, (0, 255, 0), 2)
            cv2.line(image, bottom_left, top_left, (0, 255, 0), 2)
            c_x = int((top_left[0] + bottom_right[0]) / 2.0)
            c_y = int((top_left[1] + bottom_right[1]) / 2.0)
            cv2.circle(image, (c_x, c_y), 4, (0, 0, 255), -1)
            cv2.putText(image, str(markerID) + str(maxl), (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(markerID))
            
    return image


def webcam_read():
    aruco_type = "DICT_6X6_1000"
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
    aruco_params = cv2.aruco.DetectorParameters_create()
    vid = cv2.VideoCapture(0)
    
    while vid.isOpened():
        ret, img = vid.read()
        h, w, d = img.shape
        width = 1000
        height = int(width*(h/w))
      #  img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)
        corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)
        detected_markers = aruco_display(corners, ids, img)
        cv2.imshow('WebCam', detected_markers)
    
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    vid.release()
    cv2.distroyAllWindows()


webcam_read()


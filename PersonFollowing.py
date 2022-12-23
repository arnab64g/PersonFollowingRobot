#!/usr/bin/env python

import cv2
import rospy
import math
from geometry_msgs.msg import Twist

import sys, select, os
if os.name == 'nt':
  import msvcrt, time
else:
  import tty, termios

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

e = """
Communications Failed
"""


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
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}


def get_max_size(top_left, top_right, bottom_left, bottom_right):
    a = math.sqrt(pow(top_left[0] - top_right[0], 2) + pow(top_left[1] - top_right[1], 2))
    b = math.sqrt(pow(bottom_left[0] - bottom_right[0], 2) + pow(bottom_left[1] - bottom_right[1], 2))
    c = math.sqrt(pow(top_left[0] - bottom_left[0], 2) + pow(top_left[1] - bottom_left[1], 2))
    d = math.sqrt(pow(top_right[0] - bottom_right[0], 2) + pow(top_right[1] - bottom_right[1], 2))
    ar = [int(a), int(b), int(c), int(d)]
    ar.sort()
    return ar[3]


def aruco_display(corners, ids, image, w):
    c_x = 0
    c_y = 0

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
            cv2.putText(image, str(markerID), (top_left[0], top_left[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)
            cv2.putText(image, str(w), (0, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)
            print("[Inference] ArUco marker ID: {}".format(markerID))
            
            return image, c_x, c_y, maxl
    else: 
        return image, 0, 0, 0


def webcam_read():
    aruco_type = "DICT_4X4_100"
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
    aruco_params = cv2.aruco.DetectorParameters_create()
    vid = cv2.VideoCapture('http://192.168.1.105:8080/video')
    
    while vid.isOpened():
        ret, img = vid.read()
        h, w, d = img.shape
        width = 1000
        height = int(width*(h/w))
        #img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)
        corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)
        detected_markers = aruco_display(corners, ids, img, w)
        cv2.imshow('WebCam', detected_markers)

        if cv2.waitKey(10) & 0xFF == ord('q'):
            break
    
    vid.release()
    cv2.distroyAllWindows()


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)


def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input


def checkLinearLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel):
    if turtlebot3_model == "burger":
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == "waffle" or turtlebot3_model == "waffle_pi":
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", "burger")

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        aruco_type = "DICT_4X4_100"
        aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_type])
        aruco_params = cv2.aruco.DetectorParameters_create()
        vid = cv2.VideoCapture('http://192.168.1.112:8080/video')
        while not rospy.is_shutdown():
            ret, img = vid.read()
            h, w, d = img.shape
            width = 1000
            height = int(width*(h/w))
            #img = cv2.resize(img, (width, height), interpolation=cv2.INTER_CUBIC)
            corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)
            detected_markers, x, y, l = aruco_display(corners, ids, img, w)

            cv2.imshow('WebCam', detected_markers)

            if cv2.waitKey(10) & 0xFF == ord('q'):
              break
              
            if l == 0:
                target_linear_vel = 0.0
                target_angular_vel = 0.5
            
            if(x >= 280 & x <= 360):
                target_angular_vel = 0.0
            
            if x < 280:
                target_linear_vel = 0.0
                target_angular_vel = 0.2
            elif x > 360:
                target_linear_vel = 0.0
                target_angular_vel = -0.2
            elif(l < 50 & l > 1):
                target_linear_vel = 0.5
                target_angular_vel = 0.0
            elif l <= 50:
                target_linear_vel = 0.25
                target_angular_vel = 0.0
            elif l > 60 & l <=70:
                target_linear_vel = -0.25
                target_angular_vel = 0.0
            elif l > 70:
                target_angular_vel = 0.0
                target_linear_vel = 0.5
            else:
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
            
            print(vels(target_linear_vel,target_angular_vel))
            
            if status == 20 :
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

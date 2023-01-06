#!/usr/bin/env python

import cv2
import rospy
import Calculate
import ArUcoDict
import Draw
import DisplayText
import Message
import Turtlebot3_Model
import StringM
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


def aruco_display(corners, ids, image, w, targetMarker):

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

            maxl = Calculate.get_max_size(top_left, top_right, bottom_left, bottom_right)

            Draw.MarkerBorder(image, top_left, top_right, bottom_left, bottom_right)
    
            c_x = Calculate.get_center(top_left[0], bottom_right[0]) 
            c_y = Calculate.get_center(top_left[1], bottom_right[1])
    
            Draw.MarkerCenter(image, c_x, c_y)

            DisplayText.MarkerID(image, markerID, top_left)
            DisplayText.ImageText(image, w)
            

            print("[Inference] ArUco marker ID: {}".format(markerID))

            if targetMarker != markerID:

                return image, 0, 0, 0, markerID
            
            return image, c_x, c_y, maxl, markerID
    else: 

        return image, 0, 0, 0, -1


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

    if turtlebot3_model == Turtlebot3_Model.turtlebot3_burger:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    elif turtlebot3_model == Turtlebot3_Model.turtlebot3_waffle or turtlebot3_model == Turtlebot3_Model.turtlebot3_waffle_pi:
      vel = constrain(vel, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)

    return vel


def checkAngularLimitVelocity(vel):

    if turtlebot3_model == Turtlebot3_Model.turtlebot3_burger:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    elif turtlebot3_model == Turtlebot3_Model.turtlebot3_waffle or turtlebot3_model == Turtlebot3_Model.turtlebot3_waffle_pi:
      vel = constrain(vel, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)

    return vel


if __name__=="__main__":

    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('turtlebot3_teleop')

    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    turtlebot3_model = rospy.get_param("model", Turtlebot3_Model.turtlebot3_burger)

    status = 0

    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    aruco_type = "DICT_6X6_1000"

    aruco_dict = cv2.aruco.Dictionary_get(ArUcoDict.ARUCO_DICT[aruco_type])
    aruco_params = cv2.aruco.DetectorParameters_create()

    vid = cv2.VideoCapture('http://192.168.1.100:8080/video')

    waiting_time = 0

    marker_id = int(input("Enter Marker ID: "))

    try:
        while not rospy.is_shutdown():
            ret, img = vid.read()

            h, w, d = img.shape

            left_line, right_line = Calculate.get_lef_line_right_line(w)
            
            corners, ids, rejected = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)

            detected_markers, x, y, l, markerId = aruco_display(corners, ids, img, w, marker_id)

            cv2.imshow('WebCam', detected_markers)

            if cv2.waitKey(10) & 0xFF == ord('q'):
                break

            if  marker_id != markerId: 

                print('Waiting.... ')

                waiting_time += 10

                if(waiting_time < 1000):
                    continue
                

            if marker_id == markerId:
                waiting_time = 0

            if(x >= left_line and x <= right_line):
                target_angular_vel = 0.0
            
            if x < left_line:
                target_linear_vel = 0.0
                target_angular_vel = 0.25
            elif x > right_line:
                target_linear_vel = 0.0
                target_angular_vel = -0.25
            elif(l <= 40 and l > 1):
                target_linear_vel = 0.22
                target_angular_vel = 0.0
            elif l <= 50 and l < 40:
                target_linear_vel = 0.05
                target_angular_vel = 0.0
            elif l > 60 and l <=70:
                target_linear_vel = -0.05
                target_angular_vel = 0.0
            elif l > 70:
                target_angular_vel = 0.0
                target_linear_vel = -0.2
            else:
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
            
            print( StringM.vels(target_linear_vel,target_angular_vel))

            print("Target Marker ID: ", marker_id, ", Marker ID: ", markerId)
            
            if status == 20 :
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))

            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            pub.publish(twist)

    except:
        print(Message.e)

    finally:
        twist = Twist()

        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0

        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

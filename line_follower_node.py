#!/usr/bin/env python
# coding=utf8
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
# from rgb_hsv import BGR_HSV


class LineFollower(object):
    # def __init__(self, rgb_to_track, colour_error_perc = 10.0,colour_cal=False, camera_topic="/morpheus_bot/raspicam_node/image_raw", cmd_vel_topic="/morpheus_bot/cmd_vel"):
    def __init__(self):
        # self._colour_cal = colour_cal
        # self._colour_error_perc = colour_error_perc
        # self.rgb_hsv = BGR_HSV()
        # self.hsv, hsv_numpy_percentage = self.rgb_hsv.rgb_hsv(rgb=rgb_to_track)
        # We check which OpenCV version is installed.
        # (self.major, minor, _) = cv2.__version__.split(".")
        # rospy.logwarn("OpenCV Version Installed==>"+str(self.major))

        # This way we process only half the frames
        self.process_this_frame = True

        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("image_topic", Image, self.camera_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel_topic", Twist, queue_size=1)

    def camera_callback(self, data):

        if self.process_this_frame:
            self.process_this_frame = False
                # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, "bgr8")


            # We get image dimensions and crop the parts of the image we dont need
            # Bare in mind that because its image matrix first value is start and second value is down limit.
            # Select the limits so that it gets the line not too close, not too far and the minimum portion possible
            # To make process faster.
            # TODO: Get multiple lines so that we can generate paths.

            small_frame = cv2.resize(cv_image, (0, 0), fx=0.2, fy=0.2)

            height, width, channels = small_frame.shape

            rospy.loginfo("height=%s, width=%s" % (str(height), str(width)))

            #descentre = 160
            #rows_to_watch = 100
            #crop_img = small_frame[(height) / 2 + descentre:(height) / 2 + (descentre + rows_to_watch)][1:width]
            crop_img = small_frame
            # gray=cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
            #image_blur = cv2.GaussianBlur(small_frame, (5, 5), 0)
            # img_hsv = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)
            # lowgreen = np.array([41, 180, 100])
            # ret,th1 = cv2.threshold(image_blur,35,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)
            # ret1,th2 = cv2.threshold(th1,127,255,cv2.THRESH_BINARY_INV) # invert the pixels of the image frame
            image_blur = cv2.GaussianBlur(small_frame, (5, 5), 0)
            image_hsv = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)

            black_mask = cv2.inRange(image_hsv, np.array([70,90,0]), np.array([180,255,255]))
            mask_filter = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
            #cv2.imshow('MaskFilter', mask_filter)
            edged_img = cv2.Canny(mask_filter.copy(), 35, 125)
            #cv2.imshow('Img', gray)
            #cv2.imshow('black mask', black_mask)
            
            (contours, _) = cv2.findContours(edged_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.drawContours(small_frame, contours, -1, (0,255,0) , 3)
            # cv2.imshow('frame', small_frame)
            #cv2.waitKey(10)
            # Convert from RGB to HSV
            #hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

            # min_hsv = self.hsv * (1.0-(self._colour_error_perc / 100.0))
            # max_hsv = self.hsv * (1.0 + (self._colour_error_perc / 100.0))
            # lower_yellow = np.array(min_hsv)
            # upper_yellow = np.array(max_hsv)

            # Threshold the HSV image to get only yellow colors
            # mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

            # Bitwise-AND mask and original image
            # res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

            # if self.major == '3':
                # If its 3
                # (_, contours, _) = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)

            # else:
                # If its 2 or 4
                # (contours, _) = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_TC89_L1)
            rospy.loginfo("Number of centroids==>" + str(len(contours)))
            # centres = []
            largest = -1
            area = 0
            cx =0
            cy = 0
            if len(contours)>0:
                for i in range(len(contours)):
                    # moments = cv2.moments(contours[i])
                    temp_area = cv2.contourArea(contours[i])
                # try:
                    # centres.append((int(moments['m10'] / moments['m00']), int(moments['m01'] / moments['m00'])))
                    # cv2.circle(res, centres[-1], 10, (0, 255, 0), -1)
                # except ZeroDivisionError:
                    # pass
                    # if it is the biggest we have seen, keep it
                    if temp_area > area:
                        area = temp_area
                        largest = i
                coordinates = cv2.moments(contours[largest])
                img_cnts = cv2.drawContours(small_frame.copy(),contours[largest], -1, (40, 255, 255))
                cv2.imshow("Contour", img_cnts)
                # cv2.waitKey(0)
                if coordinates["m00"] != 0:
                    cx = int(coordinates['m10'] / coordinates['m00'])
                    cy = int(coordinates['m01'] / coordinates['m00'])
                else:
                    cx = 0
                    cy = 0

                rospy.loginfo(str(cx))
            # Select the right centroid
            # [(542, 39), (136, 46)], (x, y)
            #most_right_centroid_index = 0
            # index = 0
            # max_x_value = 0

            # centroids_detected = []

            # for candidate in centres:
                # Retrieve the cx value
                # cx = candidate[0]
                # Get the Cx more to the right
                # if cx >= max_x_value:
                    # max_x_value = cx
                    # most_right_centroid_index = index
                # index += 1

                # try:
                    # cx = centres[most_right_centroid_index][0]
                    # cy = centres[most_right_centroid_index][1]
                    # rospy.logwarn("Centroid FOUND ==" + str(cx) + "," + str(cy) + "")
                # except:
                    # cy, cx = height / 2, width / 2

                # centroids_detected.append([cx,cy])
                # Draw the centroid in the result image
                cv2.circle(small_frame, (int(cx), int(cy)), 5, (0, 0, 255), -1)
                cv2.imshow('center', small_frame)
                # cv2.waitKey(0)
            #if self._colour_cal:
            #    cv2.imshow("Original", small_frame)
            #else:
            #    cv2.imshow("HSV", hsv)
            #    cv2.imshow("MASK", mask)
            #    cv2.imshow("RES", res)
            
            # We send data from the first cetroid we get
            # if len(centroids_detected) > 0:
                
                #cx_final = width
                #cy_final = height
                
                #for centroid in centroids_detected:
                    # We get the values of the centroid closer to us
                    #print(centroid)
                    #if centroid[1]< cy_final:
                        #cx_final = centroid[0]
                        #cy_final = centroid[1]
                        #print("Selected CENTROID AS FINAL")
            # else:
                # cx_final = None
                #cy_final = None
            cx_final = width
            cy_final = height
            if cx != 0 and cy != 0:
                cx_final = cx
                cy_final = cy
            else:
                cx_final = None
                cy_final = None

            self.move_robot(height, width, cx_final, cy_final)

            cv2.waitKey(1)
        else:
            self.process_this_frame = True
            
            
            
    def move_robot(self, image_dim_y, image_dim_x, cx, cy, linear_vel_base = 0.1, angular_vel_base = 0.1):
        """
        It move the Robot based on the Centroid Data
        image_dim_x=96, image_dim_y=128
        cx, cy = [(77, 71)]
        """
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        
        FACTOR_LINEAR = 0.001
        FACTOR_ANGULAR = 0.1
        
        
        if cx is not None and cy is not None:
            origin = [image_dim_x / 2.0, image_dim_y / 2.0]
            centroid = [cx, cy]
            delta = [centroid[0] - origin[0], centroid[1]]
            
            print("origin="+str(origin))
            print("centroid="+str(centroid))
            print("delta="+str(delta))
            
            # -1 because when delta is positive we want to turn right, which means sending a negative angular
            cmd_vel.angular.z = angular_vel_base * delta[0] * FACTOR_ANGULAR * -1
            # If its further away it has to go faster, closer then slower
            cmd_vel.linear.x = linear_vel_base - delta[1] * FACTOR_LINEAR
            
        else:
            cmd_vel.angular.z = angular_vel_base * 2
            cmd_vel.linear.x = linear_vel_base * 0.5
            print("NO CENTROID DETECTED...SEARCHING...")
        
        print("SPEED==>["+str(cmd_vel.linear.x)+","+str(cmd_vel.angular.z)+"]")
        self.cmd_vel_pub.publish(cmd_vel)

    def loop(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('line_follower', anonymous=True)
    # rgb_to_track = [255,255,255]
    robot_mover = LineFollower()
    # robot_mover = LineFollower(rgb_to_track=rgb_to_track, colour_error_perc= 20.0, colour_cal=False)
    robot_mover.loop()

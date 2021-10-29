#!/usr/bin/env python
"""
* Id : 5917
* Author : Atharva Karpate
* Theme: Pollinator Bee
* Filename: task_2_ip.py
* Functions: __init__,image_callback,Colorno
* Global Variables: red(Stores number of red Contours),redC(Stores the BGR value of red)
					green(Stores number of green Contours),greemC(Stores the BGR value of green)
					blue(Stores number of blue Contours),blueC(Stores the BGR value of blue)
* Description: Image processing for single waypoint(flower). 
"""

import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64

red = 0
green = 0
blue = 0

redC = (0, 0, 255)
greenC = (0, 255, 0)
blueC = (255, 0, 0)


class ColorDetect:
    """
    * Function Name: __init__
    * Input: None(self)
    * Output: Provides whycon/image_out and initialises ros_bridge
    * Logic: Subscribes to whycon/image_out via ros_bridge
    * Example Call: As soon as object is decalared.
    """

    def __init__(self):
        self.red = 0
        self.green = 0
        self.blue = 0

        rospy.init_node("ros_bridge")
        # self.img= np.zeros((1,2,3),np.uint8)

        rospy.sleep(0.1)
        # Create a ROS Bridge
        self.ros_bridge = cv_bridge.CvBridge()

        # Subscribe to whycon image_out
        self.image_sub = rospy.Subscriber(
            "/usb_cam/image_rect_color", Image, self.image_callback
        )
        self.pub = rospy.Publisher("red", Float64, queue_size=1000)

    """
        * Function Name:image_callback
        * Input: msg
        * Output: Provides openCv frame from ros image message 
        * Logic: Converts ros image message to openCv frame via ros_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 
        * Example Call: image_callback(msg)
    """

    def image_callback(self, msg):

        # 'image' is now an opencv frame
        # You can run opencv operations on 'image'
        self.image = self.ros_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.img = self.image  # self.image is stored in self.img

    """
        * Function Name:DetectColor
        * Input: None(self)
        * Output: Stores the number of red,green and blue patches in their respective variables 
        * Logic: First blurs the image ,then converts in to hsv . Calls function colorno to detect the number of squares of the particular number 
        * Example Call: DetectColor()
    """

    def DetectColor(self):

        redC = (0, 0, 255)
        greenC = (0, 255, 0)
        blueC = (255, 0, 0)

        self.blur = cv2.medianBlur(self.img, 5)  # To blur the image to reduce noise
        self.hsv = cv2.cvtColor(self.blur, cv2.COLOR_BGR2HSV)  # conversion to hsv

        lower_blue = np.array([97, 51, 227])
        upper_blue = np.array([143, 198, 253])

        lower_red = np.array([168, 44, 187])
        upper_red = np.array([192, 147, 255])

        lower_green = np.array([30, 61, 190])
        upper_green = np.array([80, 153, 255])

        # Provides with the number of color squares and stores in red,green and blue respectively.
        red = self.Colorno(lower_red, upper_red, redC)
        green = self.Colorno(lower_green, upper_green, greenC)
        blue = self.Colorno(lower_blue, upper_blue, blueC)

        return red, green, blue

    """
        * Function Name: Colorno
        * Input: lower,upper,color
        * Output: Returns the number of contours of the 'color' provided . Also draws rectanlges of that color around that contour.
        * Logic: Extracts the required color from 'lower' and 'upper' followed by erosion and then closing. Then contour is found using
        		findContours() fucntion and a rectangle is drawn around the centroid. 
        * Example Call: Colorno([97,51,227],[143,198,253],(255,0,0))
    """

    def Colorno(self, lower, upper, color):
        self.mask = cv2.inRange(
            self.hsv, lower, upper
        )  # Extraction from the lower-upper range

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

        erode = cv2.erode(
            self.mask, kernel, iterations=2
        )  # Eroded to remove white noise as well as to stop the detection of blue from whycon coordinates in whycon/image_out.

        dilation = cv2.dilate(
            erode, kernel, iterations=16
        )  # To join diffrent detected segments of the same petal in one segment to draw rectangle.
        self.closing = cv2.erode(dilation, kernel, iterations=1)

        abc, contours, hierarchy = cv2.findContours(
            self.closing, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        # To find contour centroid
        for x in range(0, len(contours)):
            M = cv2.moments(contours[x])
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # print "Centroid = ", cx, a", ", cy
            # cv2.circle(img,(cx,cy), 3, (127,127,127), -1)
            cv2.rectangle(self.img, (cx - 15, cy - 15), (cx + 15, cy + 15), color, 2)

        return len(contours)


if __name__ == "__main__":
    test = ColorDetect()
    rospy.sleep(2)
    red = 0
    blue = 0
    green = 0
    flag = 0

    while True:
        red, green, blue = test.DetectColor()
        if flag == 0:
            test.pub.publish(red)
            if red == 1:
                flag = 1

        if flag == 1:
            red = 1
            test.pub.publish(red)
            pass

        cv2.imshow("image", test.image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()
    rospy.spin()


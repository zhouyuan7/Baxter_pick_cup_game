#!/usr/bin/env python
"""
A cup using haar-cascade method to box cup target on kinect RGB frame
input topic:
    /kinect2/qhd/image_color_rect/compressed
        kinect RGB image stream
        data structure: ros CompressedImage
output topic:
    /cup_pixel_coordinates
        cup center point in RGB frame
        data structure: ros custom point array message

"""

import rospy
import time
import cv2
from cv_bridge import CvBridge, CvBridgeError
import math
import numpy as np
import sys
from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Point
from position_calibration.msg import pointarray

rospy.init_node('cup_detector', anonymous=True)
# make sure you change the .xml file path
cups_detector = cv2.CascadeClassifier('/home/yuan/catkin_ws/src/baxter_pickup_cup/src/cup_detector.xml')

class detector:
    def __init__(self):
        
        self.coordinate_pub = rospy.Publisher("/cup_pixel_coordinates", pointarray,queue_size=10)

        self.image_sub = rospy.Subscriber("/kinect2/qhd/image_color_rect/compressed", CompressedImage, self.callback, queue_size=1, buff_size=2**24)

        self.bridge = CvBridge()
  
    def callback(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Try to twick the parameters to get better performance in your environment
        cups = cups_detector.detectMultiScale(gray, 1.1, 2, 0, (80,100), (80,100))

        msg = pointarray()

        i = 0
        for(x,y,w,h) in cups:
            point = Point()
            point.x = (w/2+x)
            point.y = (h/2+y)
            point.z = 0
            msg.points.append(point)
            i += 1

            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,255,0),2)

            cv2.imshow('img',cv_image)
            k = cv2.waitKey(30) & 0xff
            if k == 27:
                break
        
        msg.index = i
        print("cups detected number:", i)

        self.coordinate_pub.publish(msg)

def main(args):
    
    detector()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)

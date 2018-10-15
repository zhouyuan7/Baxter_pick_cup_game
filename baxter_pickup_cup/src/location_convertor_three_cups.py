#!/usr/bin/env python
"""
Access kinect point cloud to get cups pose in Baxter coordinate
input topic:
    /cup_pixel_coordinates
        cup center point in RGB frame
        data structure: ros custom point array message
    /kinect2/qhd/points
        kinect point cloud stream
        data structure: ros PointCloud2 message
output topic:
    /cup_baxter_coordinates
        cup center pose in Baxter coordinate
        data structure: ros custom point array message
"""
import rospy
import numpy as np
import math
import sys
import tf
import cv2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from geometry_msgs.msg import Point
from position_calibration.msg import pointarray

rospy.init_node('location_convertor_three_cups', anonymous=True)

cupCoordinates = np.array([[0,0,0],[0,0,0]])
index = 0

class Convertor:
    def __init__ (self):
        # set point cloud frame resolution variables(qhd or sd)
        self.width = 960
        self.height = 540
        # receive target pixel coordiante generated from detector
        rospy.Subscriber("/cup_pixel_coordinates", pointarray, self.callback_processed)

        # receive kinect point cloud message
        rospy.Subscriber("/kinect2/qhd/points", PointCloud2, self.point_cloud_callback,queue_size=1)

        # output target translation messages to motion control module
        self.coordinate_pub = rospy.Publisher("/cup_baxter_coordinates", pointarray,queue_size=10)

        self.bridge = CvBridge()

    def callback_processed(self, data):
        global cupCoordinates
        global index
        index = data.index
        for i in range(0, index):
            point = Point()
            point = data.points[i]
            cupCoordinates[0][i] = point.x
            cupCoordinates[1][i] = point.y

    def cup_translation(self,cloud_points,cupCoordinates,index):
        msg = pointarray()
        i = 0
        for i in range(0, index):
            y = cupCoordinates[1][i]
            x = cupCoordinates[0][i]
            tmp = y*self.width+(x+1)
            raw_location_for_kinect = list(cloud_points[tmp])
            
            raw_location_for_kinect = np.array([[raw_location_for_kinect[0]],
                                                [raw_location_for_kinect[1]],
                                                [raw_location_for_kinect[2]]])
            rotation_matrix = np.array([[-0.019595637770269, -0.053189313003922, 0.998392161408805],
                                        [-0.998846575227872, 0.044823069193251,  -0.017216608947398],
                                        [-0.043835261330657, -0.997577961590104, -0.054006299760353]])

            translation_vector = np.array([[0.221156969513543],
                                            [-0.088928287270474],
                                            [0.2559913378498813]])
            target_location_baxter = np.add(np.dot(rotation_matrix, raw_location_for_kinect),translation_vector)
            point = Point()
            point.x = target_location_baxter[0]
            point.y = target_location_baxter[1]
            point.z = target_location_baxter[2]
            msg.points.append(point)
            i += 1
            # use tf to visualize the target pose(translation)
            br = tf.TransformBroadcaster()
            br.sendTransform((target_location_baxter[0], target_location_baxter[1], target_location_baxter[2]),
                                (0,0,0,1),
                                rospy.Time(0),
                                str(i),
                                "base")
        msg.index = i
        self.coordinate_pub.publish(msg)

    def point_cloud_callback(self,data):
        '''
        Kinect cooridnate inside Baxter coordinate 
        transformation[x,y,z]: [0.221156969513543, -0.088928287270474, 0.2559913378498813]
        rotation matrix: [-0.019595637770269 -0.053189313003922 0.998392161408805
                          -0.998846575227872 0.044823069193251  -0.017216608947398
                          -0.043835261330657 -0.997577961590104 -0.054006299760353]
        '''
        # store point cloud data into a list
        cloud_points = list(point_cloud2.read_points(data, skip_nans=False, field_names = ("x", "y", "z")))
        #print('point cloud length', len(cloud_points))
        self.cup_translation(cloud_points,cupCoordinates,index)
        
        
def main(args):
    
    Convertor()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
#!/usr/bin/env python3
"""
Created on Thu May 19 12:06:18 2022
@author: Fiona
"""
from __future__ import print_function
import sys
import math
import numpy as np




# Import do_mpc package:
import do_mpc


#ROS Imports
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

from geometry_msgs.msg import Point
from std_msgs.msg import String, ColorRGBA
from visualization_msgs.msg import Marker



class Visualise:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        self.points=[]
        #Topics & Subs, Pubs
        odometry_topic= '/odom'
        drive_topic = '/nav'
        debug_topic= '/debug'
        pathline_topic='/waypoint_vis'

        
        self.pathline_pub=rospy.Publisher(pathline_topic, Marker, queue_size=1)
        #self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)#TODO: Subscribe to LIDAR
        self.odom_sub=rospy.Subscriber(odometry_topic, Odometry, self.odometry_callback)#TODO: Subscribe to LIDAR
        self.debug_pub=rospy.Publisher(debug_topic, String, queue_size=1)
        

    def get_line(self, points):
        point_msg=Marker()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id= "map"
        point_msg.color=  ColorRGBA()
        point_msg.color.r=255
        point_msg.color.g=0
        point_msg.color.b=0
        point_msg.color.a=100
        point_msg.type=point_msg.LINE_STRIP
        point_msg.action = point_msg.ADD
        point_msg.points=points
        #point_msg.pose.position.x = point[0]
        #point_msg.pose.position.y = point[1]
        point_msg.lifetime.secs=1
        return point_msg
        
        #self.pathline_pub.publish(point_msg)
        #self.debug_pub.publish("Published point")

      

    

    def odometry_callback(self, data):
        """
        """
        rospy.loginfo(data)
        new_pos=data.pose.pose.position
        new_marker= Point()
        new_marker.x= new_pos.x
        new_marker.y= new_pos.y
        self.points.append(new_marker)
        if len(self.points)>10:
            self.points.pop(0)
        #rospy.loginfo(self.points)
        #rospy.loginfo(String(self.points[0].x-self.points[-1].x))
        line=self.get_line(self.points)
        self.pathline_pub.publish(line)


        

        

def main(args):
    
    
    rospy.init_node("visualise_node", anonymous=True)
    rospy.loginfo("starting up visualising node")
    vis = Visualise()
    rospy.sleep(1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)





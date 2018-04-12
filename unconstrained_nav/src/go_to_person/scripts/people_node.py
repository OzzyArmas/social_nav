#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from go_to_person.msg import People
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import random



def generatePeople(x = 3, y = -2, th = 0):
    people = []
    #listener = tf.TransformListener()
    #print listener.allFramesAsString()
    t = quaternion_from_euler(th, 0, 0)
    head = Header(0,rospy.Time(0), '/map') #/kinect_rgb_optical_frame
    pose = PoseStamped(head, Pose(Point(x,y,0),Quaternion(t[0],t[1],t[2],t[3])))
    #point = listener.transformPoint('/map', point)
    people.append(pose)

    pub = rospy.Publisher('/foo/People', People, queue_size=1)
    pub.publish(People(people))


if __name__ == '__main__':
    rospy.init_node('people_node')
    rate = rospy.Rate(1)
    n = 0
    while(True):
        generatePeople(-1,2,math.pi/2)
        #generatePeople()
        #generatePeople(0,0,0)

        rate.sleep()
        n += 1
    rospy.spin()

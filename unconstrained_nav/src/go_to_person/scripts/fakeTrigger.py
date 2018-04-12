#!/usr/bin/env python

import sys
from std_srvs.srv import Trigger
import rospy
import math
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
from go_to_person.msg import People
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
import random


def generatePeople(x = 3, y = -2):
    people = []
    #listener = tf.TransformListener()
    #print listener.allFramesAsString()
    t = quaternion_from_euler(math.pi, 0, 0)
    head = Header(0,rospy.Time(0), '/map') #/kinect_rgb_optical_frame
    person_x = x
    person_y = y
    pose = PoseStamped(head, Pose(Point(person_x,person_y,0),Quaternion(t[0],t[1],t[2],t[3])))
    #point = listener.transformPoint('/map', point)
    people.append(pose)

    pub = rospy.Publisher('/foo/People', People, queue_size=1)
    pub.publish(People(people))

def trigger_move():
    rospy.wait_for_service('go_to_person')
    try:
        serv = rospy.ServiceProxy('go_to_person', Trigger)
        print serv()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == '__main__':
    #rospy.init_node('people_node')
    # if len(sys.argv) > 1:
    #     generatePeople(sys.argv[1], sys.argv[2])
    # else:
    #     generatePeople()
    trigger_move()

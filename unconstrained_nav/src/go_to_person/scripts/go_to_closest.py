#!/usr/bin/env python
# Something something module tells the
# robot which person to go based on trigger
import math
import actionlib
import rospy
import tf
import ast
from copy import deepcopy
from geometry_msgs.msg import Point, PoseStamped, Pose, Quaternion
from go_to_person.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetPlan
from rospy.exceptions import ROSException
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class GoToPerson():

    def __init__(self, zero_base_link = True):

        self.node = rospy.init_node('go_to_person_server')

        self.person_subscriber = rospy.Subscriber('/foo/People', People, self._handlePeople, queue_size=1)

        rospy.Service('go_to_person', Trigger, self._handle_service)

        self.tf_listener = tf.TransformListener()
        self._id = 0
        self.robot_footprint = ast.literal_eval(rospy.get_param('/move_base/global_costmap/footprint'))
        #bubble_size = 0.65
        #self.bubble_size = rospy.get_param('bubble_size')
        print ('Foot print: {0}'.format(self.robot_footprint))
        self.closes_person = None


    def _handlePeople(self, people):

        closest_distance = float('inf')

        for person in people.people:
            stamped_pose = self.tf_listener.transformPose('/base_link', person)
            dist = math.sqrt(stamped_pose.pose.position.x ** 2 + stamped_pose.pose.position.y ** 2)
            if dist < closest_distance:
                closest_distance = dist
                self.closes_person = stamped_pose

    def _handle_service(self, data):

        final_pose = self._plan_approach(self.closes_person)
        if final_pose is None:
            return TriggerResponse(False, 'No goal found\n')
        goal = MoveBaseGoal(final_pose)
        client = actionlib.SimpleActionClient('move_base_navi', MoveBaseAction)
        client.wait_for_server()
        client.send_goal(goal)

        return TriggerResponse(True, '\nGoal Found at:\n ' + str(goal))




    def _plan_approach(self, person_pose):

        base_transform = self.tf_listener.lookupTransform('/map','/base_link', rospy.Time(0))

        header = Header(None, rospy.Time(0), '/map')
        # Create a base_link pose
        base_link_pose = Pose(Point(base_transform[0][0],
                                    base_transform[0][1],
                                    base_transform[0][2]),
                              Quaternion(base_transform[1][0],
                                         base_transform[1][1],
                                         base_transform[1][2],
                                         base_transform[1][3]))
        base_link_pose = PoseStamped(header, base_link_pose)
        #person_pose = self.tf_listener.transformPose('/map', person_pose)
        if person_pose is None:
            return None

        # Based on Robot Foot Print from:
        # vector_ws/src/vector_v1/vector_common/vector_navigation/vector_navigation_apps/config/common
        # [Corner][x y]
        robot_front = self.robot_footprint[0][0] #That's a corner bro
        bubble_size = 0.65
        try:
            poses = []
            final_pose = None
            angle_shift_right = math.pi
            angle_shift_left = math.pi
            while angle_shift_right > 0 and len(poses) == 0:
                # The loop checks if the base_link pose is reachable

                p_orientation = person_pose.pose.orientation

                theta_right = euler_from_quaternion((p_orientation.x,
                                                     p_orientation.y,
                                                     p_orientation.z,
                                                     p_orientation.w))[2] + angle_shift_right

                target_x_right = -(bubble_size + robot_front) * math.cos(theta_right)\
                                 + person_pose.pose.position.x

                target_y_right = -(bubble_size) * math.sin(theta_right)\
                                 + person_pose.pose.position.y

                z = theta_right
                theta_right = quaternion_from_euler(0, 0, theta_right)

                header = person_pose.header
                pose = Pose(Point(target_x_right, target_y_right, 0), Quaternion(theta_right[0],
                                                                                 theta_right[1],
                                                                                 theta_right[2],
                                                                                 theta_right[3]))
                final_pose = PoseStamped(header, pose)
                final_pose = self.tf_listener.transformPose('/map', final_pose)
                rospy.wait_for_service('move_base/make_plan', timeout=5)
                get_plan_service = rospy.ServiceProxy('move_base/make_plan', GetPlan)
                resp = get_plan_service(base_link_pose, final_pose, 0.1)

                angle_shift_right -= math.pi / 8
                if (len(resp.plan.poses) == 0):
                    theta_left = euler_from_quaternion((p_orientation.x,
                                                        p_orientation.y,
                                                        p_orientation.z,
                                                        p_orientation.w))[2] - angle_shift_left

                    target_x = -(bubble_size + robot_front) * math.cos(theta_left)\
                               + person_pose.pose.position.x

                    target_y = -(bubble_size) * math.sin(theta_left)\
                               + person_pose.pose.position.y

                    z = theta_left
                    theta_left = quaternion_from_euler(0, 0, theta_left)

                    header = person_pose.header
                    pose = Pose(Point(target_x, target_y, 0), Quaternion(theta_left[0],
                                                                         theta_left[1],
                                                                         theta_left[2],
                                                                         theta_left[3]))
                    final_pose = PoseStamped(header, pose)
                    final_pose = self.tf_listener.transformPose('/map', final_pose)
                    rospy.wait_for_service('move_base/make_plan', timeout=5)
                    get_plan_service = rospy.ServiceProxy('move_base/make_plan', GetPlan)
                    resp = get_plan_service(base_link_pose, final_pose, 0.1)

                    angle_shift_left += math.pi / 8

                # The loop ensure that the foot print of the robot fits at the desination
                footprint_check = True
                for corner in self.robot_footprint:
                    final_pose_corner = deepcopy(final_pose)

                    final_pose_corner.pose.position.x = final_pose.pose.position.x + corner[0]
                    final_pose_corner.pose.position.y = final_pose.pose.position.y + corner[1]
                    final_pose_corner = self.tf_listener.transformPose('/map', final_pose_corner)

                    rospy.wait_for_service('move_base/make_plan',
                                           timeout=5)
                    get_plan_service = rospy.ServiceProxy('move_base/make_plan', GetPlan)
                    corner_resp = get_plan_service(base_link_pose, final_pose_corner, 0.1)
                    if len(corner_resp.plan.poses) == 0:
                        footprint_check = False
                        poses = []
                        break
                if not footprint_check:
                    final_pose = None
                else:
                    poses = resp.plan.poses

            return final_pose
        except ROSException as rosExc:
            print rosExc.message

        return None

'''
orient = person_pose.pose.orientation

'''

if __name__ == '__main__':
    goToPersonNode = GoToPerson()
    rospy.spin()

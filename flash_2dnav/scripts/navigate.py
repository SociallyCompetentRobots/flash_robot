#!/usr/bin/env python

import rospy
import actionlib

import tf.transformations as tr

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Quaternion

from flash_behaviors.msg import BehaveAction
from flash_behaviors.msg import BehaveGoal

from flash_behaviors.msg import SpeakAction
from flash_behaviors.msg import SpeakGoal

from datetime import datetime


if __name__ == '__main__':

    rospy.init_node('move_base_client_node')

    move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    behave_client = actionlib.SimpleActionClient('behavior_server', BehaveAction)
    speak_client = actionlib.SimpleActionClient('speech_server', SpeakAction)

    rospy.loginfo("Waiting for servers...")
    move_client.wait_for_server()
    behave_client.wait_for_server()
    speak_client.wait_for_server()
    rospy.loginfo("Server found!")

    # Node sleeps for 5 minutes.
    rospy.sleep(3.)

    # Define goal -> visit the user
    # move_goal = MoveBaseGoal()

    # move_goal.target_pose.header.frame_id = 'map'
    # move_goal.target_pose.header.stamp = rospy.Time.now()

    # move_goal.target_pose.pose.position.x = 1.87
    # move_goal.target_pose.pose.position.y = -1.54
    # move_goal.target_pose.pose.position.z = 0.0

    # # orientation = tr.quaternion_from_euler(0.0, 0.0, -1.5708)
    # # move_goal.target_pose.pose.orientation.x = orientation[0]
    # # move_goal.target_pose.pose.orientation.y = orientation[1]
    # # move_goal.target_pose.pose.orientation.z = orientation[2]
    # # move_goal.target_pose.pose.orientation.w = orientation[3]
    # move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14/2))

    # rospy.loginfo("Sending move goal...")

    # move_client.send_goal(move_goal)
    # move_client.wait_for_result()

    rospy.loginfo("Current time: %s", str(datetime.now()))

    # Perform behavior.
    behave_goal = BehaveGoal()

    behave_goal.behavior = 'Joy'
    behave_goal.duration = 2
    behave_goal.intensity = 8

    speak_goal = SpeakGoal()

    speak_goal.text = 'Hello. My name is Alyx!'
    speak_goal.duration = 2

    rospy.loginfo("Sending behavior goal...")

    behave_client.send_goal(behave_goal)
    behave_client.wait_for_result()

    rospy.loginfo("Sending speech goal...")

    speak_client.send_goal(speak_goal)
    speak_client.wait_for_result()

    # Define goal -> go home
    # move_goal = MoveBaseGoal()

    # move_goal.target_pose.header.frame_id = 'map'
    # move_goal.target_pose.header.stamp = rospy.Time.now()

    # move_goal.target_pose.pose.position.x = 0.0
    # move_goal.target_pose.pose.position.y = 0.0
    # move_goal.target_pose.pose.position.z = 0.0
    # move_goal.target_pose.pose.orientation.x = 0.0
    # move_goal.target_pose.pose.orientation.y = 0.0
    # move_goal.target_pose.pose.orientation.z = 0.0
    # move_goal.target_pose.pose.orientation.w = 1.0

    # rospy.loginfo("Sending move goal...")

    # move_client.send_goal(move_goal)
    # move_client.wait_for_result()

    rospy.loginfo("All done!")
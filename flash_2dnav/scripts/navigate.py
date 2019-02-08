#!/usr/bin/env python

import rospy
import actionlib

import tf.transformations as tr

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Quaternion

from flash_behaviors.msg import ActAction
from flash_behaviors.msg import ActGoal
from flash_behaviors.msg import Speech

from datetime import datetime


if __name__ == '__main__':

    rospy.init_node('move_base_client_node')

    speak_pub = rospy.Publisher('/flash_robot/say', Speech, queue_size=1)

    move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    act_client = actionlib.SimpleActionClient('action_server', ActAction)

    rospy.loginfo("Waiting for servers...")
    move_client.wait_for_server()
    act_client.wait_for_server()
    rospy.loginfo("Servers found!")

    # Start the look alive behavior.
    act_goal = ActGoal()

    act_goal.action = 'robot.body.neck.head.ActAlive(5, 2, 5, 2, 2, 5, 2),'

    rospy.loginfo("Sending act goal (act alive)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    # Node sleeps for 5 minutes.
    # rospy.sleep(300.)
    rospy.sleep(3.)

    # Stop head behavior.
    act_goal.action = 'robot.body.neck.head.Stop'

    rospy.loginfo("Sending act goal (stop head)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    # Go to relaxed position.
    act_goal.action = 'GoToRelaxedPose()'

    rospy.loginfo("Sending act goal (relaxed pose)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    # Send blinking behavior.
    act_goal.action = 'robot.body.neck.head.ActBlinking(20,1),'

    rospy.loginfo("Sending act goal (blink behavior)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    # Waypoints definition:
    move_goal = MoveBaseGoal()

    move_goal.target_pose.header.frame_id = 'map'

    # Home.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x = 0.0
    move_goal.target_pose.pose.position.y = 0.0
    move_goal.target_pose.pose.position.z = 0.0

    move_goal.target_pose.pose.orientation = Quaternion(0., 0., 0., 1.)

    rospy.loginfo("Sending waypoint 1...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Move forward 1.4 meters.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x = 1.4
    move_goal.target_pose.pose.position.y = 0.0
    move_goal.target_pose.pose.position.z = 0.0

    move_goal.target_pose.pose.orientation = Quaternion(0., 0., 0., 1.)

    rospy.loginfo("Sending waypoint 2...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Rotate 90 degrees right.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x = 1.4
    move_goal.target_pose.pose.position.y = 0.0
    move_goal.target_pose.pose.position.z = 0.0

    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14/2))

    rospy.loginfo("Sending waypoint 3...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Move forward 1.4 meters.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x =  1.4
    move_goal.target_pose.pose.position.y = -1.4
    move_goal.target_pose.pose.position.z =  0.0

    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14/2))

    rospy.loginfo("Sending waypoint 4...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Define goal -> visit the user (this is in replacement of the waypoints if we want the robot to plan to go straight to the user instead of predefined waypoints)

    # move_goal = MoveBaseGoal()

    # move_goal.target_pose.header.frame_id = 'map'
    # move_goal.target_pose.header.stamp = rospy.Time.now()

    # move_goal.target_pose.pose.position.x = 1.18
    # move_goal.target_pose.pose.position.y = -1.4
    # move_goal.target_pose.pose.position.z = 0.0

    # move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14/2))

    # rospy.loginfo("Sending move goal...")

    # move_client.send_goal(move_goal)
    # move_client.wait_for_result()

    # Stop head behavior.
    act_goal.action = 'robot.body.neck.head.Stop'

    rospy.loginfo("Sending act goal (stop head)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    rospy.loginfo("Current time: %s", str(datetime.now()))

    # Perform interaction behavior.
    act_goal.action = 'PointLeft(4)'

    rospy.loginfo("Sending act goal (interaction behavior)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    speech = Speech()
    speech.text = 'You are fired!'
    speech.intensity = 2

    speak_pub.publish(speech)

    # Go back to relaxed position.
    act_goal.action = 'GoToRelaxedPose()'

    rospy.loginfo("Sending act goal (relaxed pose)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    # Send blinking behavior.
    act_goal.action = 'robot.body.neck.head.ActBlinking(20,1),'

    rospy.loginfo("Sending act goal (blinking behavior)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    # Turn 180 degrees.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x =  1.4
    move_goal.target_pose.pose.position.y = -1.4
    move_goal.target_pose.pose.position.z =  0.0

    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14*3/2))

    rospy.loginfo("Sending waypoint 5...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Go back 1.4 meters.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x = 1.4
    move_goal.target_pose.pose.position.y = 0.0
    move_goal.target_pose.pose.position.z = 0.0

    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14*3/2))

    rospy.loginfo("Sending waypoint 6...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Turn 90 degrees left.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x = 1.4
    move_goal.target_pose.pose.position.y = 0.0
    move_goal.target_pose.pose.position.z = 0.0

    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14))

    rospy.loginfo("Sending waypoint 7...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Go back 1.4 meters.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x = 0.0
    move_goal.target_pose.pose.position.y = 0.0
    move_goal.target_pose.pose.position.z = 0.0

    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14))

    rospy.loginfo("Sending waypoint 8...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Start the look alive behavior.
    act_goal.action = 'robot.body.neck.head.ActAlive(5, 2, 5, 2, 2, 5, 2),'

    rospy.loginfo("Sending act goal (act alive)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    # Shutdown hook.
    def stop_head():
        # Stop head behavior.
        act_goal.action = 'robot.body.neck.head.Stop'

        rospy.loginfo("Sending act goal (stop head)...")

        act_client.send_goal(act_goal)
        act_client.wait_for_result()

    rospy.on_shutdown(stop_head)

    # Keep the node alive.
    rospy.spin()

    # Define goal -> go home (this is instead of the waypoints if we want the robot to plan to go straight home instead of predefined waypoints)

    # move_goal = MoveBaseGoal()

    # move_goal.target_pose.header.frame_id = 'map'
    # move_goal.target_pose.header.stamp = rospy.Time.now()

    # move_goal.target_pose.pose.position.x = 0.0
    # move_goal.target_pose.pose.position.y = 0.0
    # move_goal.target_pose.pose.position.z = 0.0

    # move_goal.target_pose.pose.orientation = Quaternion(0., 0., 0., 1.)

    # rospy.loginfo("Sending move goal...")

    # move_client.send_goal(move_goal)
    # move_client.wait_for_result()

    rospy.loginfo("All done!")
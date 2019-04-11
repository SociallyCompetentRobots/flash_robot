#!/usr/bin/env python

import rospy
import actionlib

import tf.transformations as tr

from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool
from std_srvs.srv import Empty, EmptyRequest

from flash_behaviors.msg import ActAction
from flash_behaviors.msg import ActGoal
from flash_behaviors.msg import Speech

from datetime import datetime


if __name__ == '__main__':

    rospy.init_node('move_base_client_node')

    # Process status.
    status = False

    # Subscriber callback.
    def process_cb(data):
        global status
        status = data.data

    # Process signal subscriber.
    process_sub = rospy.Subscriber('/system/ps/psychopy', Bool, process_cb)

    # Speech publisher.
    speak_pub = rospy.Publisher('/flash_robot/say', Speech, queue_size=1)

    # Action Clients.
    move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    act_client = actionlib.SimpleActionClient('action_server', ActAction)

    rospy.loginfo("Waiting for servers...")
    move_client.wait_for_server()
    act_client.wait_for_server()
    rospy.wait_for_service('/move_base/clear_costmaps')
    rospy.loginfo("Servers found!")

    clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)

    # Action goal.
    act_goal = ActGoal()

    # Convenience functions.
    def relaxed_pose():
        # Go to relaxed pose.
        act_goal.action = 'GoToRelaxedPose()'

        rospy.loginfo("Sending act goal (relaxed pose)...")

        act_client.send_goal(act_goal)
        act_client.wait_for_result()

    def act_alive():
        # Start the look alive behavior.
        act_goal.action = 'robot.body.neck.head.ActAlive(5, 2, 5, 2, 2, 10, 2),'

        rospy.loginfo("Sending act goal (act alive)...")

        act_client.send_goal(act_goal)
        act_client.wait_for_result()

    def act_blinking():
        # Start blinking behavior (during the forward motion).
        act_goal.action = 'robot.body.neck.head.ActBlinking(10,2),'

        rospy.loginfo("Sending act goal (blinking behavior)...")

        act_client.send_goal(act_goal)
        act_client.wait_for_result()

    def stop_head():
        # Stop head behavior.
        act_goal.action = 'robot.body.neck.head.Stop'

        rospy.loginfo("Sending act goal (stop head)...")

        act_client.send_goal(act_goal)
        act_client.wait_for_result()

        # Make a normal face expression.
        act_goal.action = 'robot.body.neck.head.BehaveNormal(2)'

        rospy.loginfo("Sending act goal (Express normal)...")

        act_client.send_goal(act_goal)
        act_client.wait_for_result()

    # Go to relaxed pose.
    relaxed_pose()
    # Start the look alive behavior.
    act_alive()

    # Node sleeps for 5 minutes.
    # rospy.sleep(300.)
    # rospy.sleep(10.)

    # Stop head behavior.
    stop_head()

    # Start the timer.
    begin = rospy.get_time()

    # Announce start of the navigation behavior.
    speech = Speech()

    speech.text = "That is me done for now."
    speech.intensity = 2

    speak_pub.publish(speech)

    # Wait for 2 seconds before starting the navigation.
    rospy.sleep(2.)

    # Start the look alive behavior (during the turn in place motion).
    act_alive()

    ###########################################################################
    # Waypoints definition.
    ###########################################################################
    move_goal = MoveBaseGoal()

    move_goal.target_pose.header.frame_id = 'map'

    # Home.
    # move_goal.target_pose.header.stamp = rospy.Time.now()

    # move_goal.target_pose.pose.position.x = 0.0
    # move_goal.target_pose.pose.position.y = 0.0
    # move_goal.target_pose.pose.position.z = 0.0

    # move_goal.target_pose.pose.orientation = Quaternion(0., 0., 0., 1.)

    # rospy.loginfo("Sending waypoint 1...")

    # move_client.send_goal(move_goal)
    # move_client.wait_for_result()

    # # Stop the look alive behavior (after turn in place motion).
    # stop_head() 
    # # Start blinking behavior (during the forward motion).
    # act_blinking()

    # # Move forward 1.6 meters.
    # move_goal.target_pose.header.stamp = rospy.Time.now()

    # move_goal.target_pose.pose.position.x = 1.6
    # move_goal.target_pose.pose.position.y = 0.0
    # move_goal.target_pose.pose.position.z = 0.0

    # move_goal.target_pose.pose.orientation = Quaternion(0., 0., 0., 1.)

    # rospy.loginfo("Sending waypoint 2...")

    # move_client.send_goal(move_goal)
    # move_client.wait_for_result()

    # # Stop the look blinking behavior.
    # stop_head()
    # # Start the look alive behavior (during the turn in place motion).
    # act_alive()

    # Rotate 90 degrees right.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x = 1.6
    move_goal.target_pose.pose.position.y = 0.0
    move_goal.target_pose.pose.position.z = 0.0

    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14/2))

    rospy.loginfo("Sending waypoint 3...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Stop the look alive behavior (after turn in place motion).
    stop_head()
    # Start blinking behavior (during the forward motion).
    act_blinking()

    # Move forward 1.6 meters and rotate approximately -80 degrees.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x =  1.6
    move_goal.target_pose.pose.position.y = -2.0
    move_goal.target_pose.pose.position.z =  0.0

    # move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -11*3.14/25))
    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -1.1))

    rospy.loginfo("Sending waypoint 4...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Stop the look blinking behavior.
    stop_head()

    # Report the time it took for the robot to reach the middle goal position.
    middle = rospy.get_time() - begin
    rospy.loginfo("Time to the goal: %s seconds", str(middle))

    ###########################################################################
    # Interaction.
    ###########################################################################
    # Initiate interruption.
    speech.text = 'Hi there. Sorry to disturb you.'
    speech.intensity = 2

    speak_pub.publish(speech)

    # Separate arms from body.
    act_goal.action = 'robot.body.arm.MoveCenterDown(3)'

    act_client.send_goal(act_goal)
    act_client.wait_for_result()
    
    rospy.sleep(1.)

    # Move hands forward.
    act_goal.action = 'robot.body.arm.MoveCenterDown2(3)'

    rospy.loginfo("Sending act goal (interaction behavior)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    # Speak.
    speech.text = 'Could you open the word task on the desktop please?'
    speech.intensity = 2

    speak_pub.publish(speech)

    # Initialize timer to log how much time the user takes to respond to the request.
    interruption_start = rospy.get_time()

    rospy.sleep(2.)

    # Wait until the user has opened the word task.
    counter = 1
    while not (status or rospy.is_shutdown()):
        rospy.Rate(10).sleep()

        # Reiterate request.
        if not (counter % 100):
            speech.text = 'Could you open the word task on the desktop please?'
            speech.intensity = 2

            speak_pub.publish(speech)

        counter += 1
    
    # Log reaction time.
    interruption_end = rospy.get_time() - interruption_start
    rospy.loginfo("Reaction time: %s seconds", str(interruption_end))

    speech.text = 'Thank you.'
    speech.intensity = 2

    speak_pub.publish(speech)

    # Go back to relaxed pose.
    act_goal.action = 'robot.body.arm.MoveCenterDown(3)'
    
    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    rospy.sleep(2.)

    relaxed_pose()

    # Start the look alive behavior (during the turn in place motion).
    act_goal = ActGoal()

    act_goal.action = 'robot.body.neck.head.ActAlive(5, 2, 5, 2, 2, 10, 2),'

    rospy.loginfo("Sending act goal (act alive)...")

    act_client.send_goal(act_goal)
    act_client.wait_for_result()

    # Clear costmaps.
    clear_costmaps(EmptyRequest())

    ###########################################################################
    # Waypoints definition.
    ###########################################################################
    # # Turn 180 degrees.
    # move_goal.target_pose.header.stamp = rospy.Time.now()

    # move_goal.target_pose.pose.position.x =  1.6
    # move_goal.target_pose.pose.position.y = -2.0
    # move_goal.target_pose.pose.position.z =  0.0

    # move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14*3/2))

    # rospy.loginfo("Sending waypoint 5...")

    # move_client.send_goal(move_goal)
    # move_client.wait_for_result()

    # # Stop the look alive behavior (after turn in place motion).
    # stop_head()
    # # Start blinking behavior (during the forward motion).
    # act_blinking()

    # # Go back 1.6 meters.
    # move_goal.target_pose.header.stamp = rospy.Time.now()

    # move_goal.target_pose.pose.position.x = 1.6
    # move_goal.target_pose.pose.position.y = 0.0
    # move_goal.target_pose.pose.position.z = 0.0

    # move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14*3/2))

    # rospy.loginfo("Sending waypoint 6...")

    # move_client.send_goal(move_goal)
    # move_client.wait_for_result()

    # # Stop the look blinking behavior.
    # stop_head()
    # # Start the look alive behavior (during the turn in place motion).
    # act_alive()

    # Turn 90 degrees left.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x = 1.6
    move_goal.target_pose.pose.position.y = 0.0
    move_goal.target_pose.pose.position.z = 0.0

    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14))

    rospy.loginfo("Sending waypoint 7...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Stop the look alive behavior (after turn in place motion).
    stop_head()
    # Start blinking behavior (during the forward motion).
    act_blinking()

    # Go back 1.6 meters.
    move_goal.target_pose.header.stamp = rospy.Time.now()

    move_goal.target_pose.pose.position.x = 0.2
    move_goal.target_pose.pose.position.y = 0.0
    move_goal.target_pose.pose.position.z = 0.0

    move_goal.target_pose.pose.orientation = Quaternion(*tr.quaternion_from_euler(0.0, 0.0, -3.14))

    rospy.loginfo("Sending waypoint 8...")

    move_client.send_goal(move_goal)
    move_client.wait_for_result()

    # Stop the look blinking behavior.
    stop_head()
    # Start the look alive behavior (during the turn in place motion).
    act_alive()

    ###########################################################################

    # Finish time.
    end = rospy.get_time() - begin
    rospy.loginfo("Time to the end: %s seconds", str(end))

    # Shutdown hook.
    rospy.on_shutdown(stop_head)

    # Keep the node alive.
    rospy.spin()
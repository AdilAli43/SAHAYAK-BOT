#!/usr/bin/env python


import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def movebase_client():
    x1=0
    y1=0
    i=1
    a = [[0,0],[-9.1,-1.2],[10.7,10.5],[12.6,-1.9],[18.2,-1.4],[-2,4]]
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.orientation.w = 1.0
    while not rospy.is_shutdown():
    	goal.target_pose.pose.position.x = x1
    	goal.target_pose.pose.position.y = y1
        client.send_goal(goal)
        wait = client.wait_for_result()
        if wait :
            if i<6 :
	        [x1,y1]=a[i]
                i=i+1
            else :
               return client.get_result()

        else :
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")

    
   

if __name__ == '__main__':
    try:
        rospy.init_node('ebot_nav')
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


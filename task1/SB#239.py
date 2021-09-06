#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from math import atan2 ,sin
# initialization of variables
x=0
y=0
z=0
x1=0
y1=0
regions = {
        'bright':0,
        'fright':0,
        'front':0,
        'fleft':0,
        'bleft':0,
    }

def Waypoints(x,y):
    # using equation for y variable for tracing sine wave
    x  = x+0.23
    y  = 2*(sin (x))*(sin (x/2))
    return x,y


def control_loop():
    global x
    global y
    global z
    global x1
    global y1
    global regions
    rospy.init_node('ebot_controller')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    
    rate = rospy.Rate(10) 

    velocity_msg = Twist()
    velocity_msg.linear.x =0
    velocity_msg.angular.z =0
    pub.publish(velocity_msg)
    
    

    while not rospy.is_shutdown():
        # tracing sine path
        # x condition to limit 0 to pi
        while x < 6.15 :
            theta_goal=atan2((y1-y),(x1-x))
            e_theta=z-theta_goal
            velocity_msg.linear.x =0.37
            velocity_msg.angular.z =-7*e_theta
            pub.publish(velocity_msg)
            x1,y1=Waypoints(x,y)                   #generating waypoints
            
        
        # to avoid concave obstacle and reach goal
        to_goal=atan2((0-y),(12.5-x))
        theta=z-to_goal
        if regions['front']<2 or ( regions['fright']<0.7 or regions['bright'] <0.5 ):
            velocity_msg.linear.x =0
            velocity_msg.angular.z =0.6
        else :
            if x>11 :  # just to increase the accuracy to reach goal
               velocity_msg.linear.x =0.3
               velocity_msg.angular.z =-6*theta
            else :
                velocity_msg.linear.x =0.9
                velocity_msg.angular.z =-3*theta
    	pub.publish(velocity_msg)
       
        if x>=12 and y>=0 :  # to stop the bot at the goal
            velocity_msg.linear.x =0
            velocity_msg.angular.z =0
            pub.publish(velocity_msg)
            break 

        print("Controller message pushed at {}".format(rospy.get_time()))
        
    	rate.sleep()
    
def odom_callback(data):
    global z
    global x
    global y
    x = data.pose.pose.position.x;
    y = data.pose.pose.position.y;
    z = data.pose.pose.orientation.z;
    
    
def laser_callback(msg):
    global regions
    regions = {
        'bright':min(min(msg.ranges[50:144]),10),
        'fright':min(min(msg.ranges[145:288]),10),
        'front':min(min(msg.ranges[289:432]),10),
        'fleft':min(min(msg.ranges[433:576]),10),
        'bleft':min(min(msg.ranges[577:720]),10),
        }
         
    
    



if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass





















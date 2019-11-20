#!/usr/bin/env python

import rospy
from duckietown_msgs.msg import Twist2DStamped

if __name__ == '__main__':
    rospy.init_node('open_loop', anonymous=True)
    pub = rospy.Publisher('~car_cmd', Twist2DStamped, queue_size=1)
    rate = rospy.Rate(10) # 10hz
    t_start = rospy.get_time()
    while not rospy.is_shutdown():
        t = rospy.get_time()
        msg = Twist2DStamped()
        dt = t - t_start

        if dt > 10 and dt < 12:   # straight line
            msg.v = 0.3   # speed
            msg.omega = 0.0   # no angle  
        elif dt > 12 and dt < 12.7:   # turn around 180
            msg.v = 0.1   
            msg.omega = 5

        if dt > 12.7 and dt < 14.7:   # straight line
            msg.v = 0.3   # speed
            msg.omega = 0.0   # no angle  
        elif dt > 14.7 and dt < 15.4:   # turn around 180
            msg.v = 0.1   
            msg.omega = 5
       
        else:
            msg.v = 0
            msg.omega = 0
        pub.publish(msg)
        rate.sleep()

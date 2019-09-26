#!/usr/bin/env python
# Revision $Id$
# Simple talker demo that published std_msgs/Strings messages to 'ob_det' topic

import rospy # required for ROS node
import random # for random numbers
from std_msgs.msg import String # allows string message container for publishing
from geometry_msgs.msg import Point # standard type, contains x, y, z
from funbotics_hw2.msg import posId # custom msg with string id and Point position


def n1PtGen(): # Node 1 Point Generator
    # Point was String
    pub = rospy.Publisher('obstacles_detected', Point, queue_size=10) # set to publish Point message to ob_det topic
    rospy.init_node('n1PtGen', anonymous=True) # name of the node, anonymous makes it unique
    rate = rospy.Rate(0.5) # 1hz, the rate object uses sleep method

    ptMsg = Point()
    
    while not rospy.is_shutdown(): # should the program exit?
        # hello_str = "Hello World @ %s " % rospy.get_time() # display message & time
	ptMsg.x = random.uniform(0, 1024)
        ptMsg.y = random.uniform(0, 1024)
        ptMsg.z = random.uniform(0, 1024)
        #rospy.loginfo(ptMsg) # message output to screen, node's log file, rosout
        rospy.loginfo('n1PtGen -> x: %g, y: %g, z: %g' % (ptMsg.x, ptMsg.y, ptMsg.z))
        pub.publish(ptMsg) # publishes string to ob_det topic
        rate.sleep() # similar to time.sleep but works with simulated time

if __name__ == '__main__':
    try: 
        n1PtGen()
    except rospy.ROSInterruptException: # exception handling, used bc sleep
        pass


# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

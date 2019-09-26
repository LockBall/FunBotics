#!/usr/bin/env python

# Revision $Id$

# https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
# includes a callback-based mechanism for publishing 

import rospy # required for ROS node
import random # for random numbers
from std_msgs.msg import String # allows string message container for publishing
from geometry_msgs.msg import Point # standard type, contains x, y, z floats
from funbotics_hw2.msg import posId # custom msg with string id and Point position

message = None # None = NULL, case sensitive

def callback(ptData):
    # global message
    rospy.loginfo(rospy.get_caller_id() + ' n1PtGen Sent x: %g, y: %g, z: %g' % (ptData.x, ptData.y, ptData.z)) 

    regOb = posId() # regOb is an instance of the posId message object that has a string id and a point in it
    regOb.id = str(random.uniform(0, 1024)) # unique id as required
    regOb.position.x = ptData.x
    regOb.position.y = ptData.y
    regOb.position.z = ptData.z

    # Point was String
    pub = rospy.Publisher('registered_obstacles', posId, queue_size=10) # set to publish a Point to reg_ob topic
    # rospy.loginfo(message) # for logging, arg was hello_str. redundant with above

    pub.publish(regOb)

def n2PtId():
    rospy.init_node('n2PtId', anonymous=True) # name of the node, anonymous makes it unique

    # Point was String
    rospy.Subscriber('obstacles_detected', Point, callback) # node subscribes to ob_det topic published by node 1
# When new message received, callback is invoked with the message as the first argument.


    rospy.spin()  # prevents python from exiting until this node is stopped

if __name__ == '__main__':
        n2PtId()


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

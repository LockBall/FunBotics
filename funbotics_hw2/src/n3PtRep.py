#!/usr/bin/env python
# Revision $Id$

import rospy # required for ROS node
from std_msgs.msg import String # allows string message container for publishing
from geometry_msgs.msg import Point # standard type, contains x, y, z
from funbotics_hw2.msg import posId # custom msg with string id and Point position

def callback(data): # %f could be used to display more precision
    rospy.loginfo(rospy.get_caller_id() + ' Detected obstacle %s at position x: %g, y: %g, z: %g \n' % (data.id, data.position.x, data.position.y, data.position.z))

def n3PtRep():
    rospy.init_node('n3PtRep', anonymous=True) # name of the node, anonymous makes it unique

    rospy.Subscriber('registered_obstacles', posId, callback) # node subscribes to reg_ob topic
# When new message is received, callback is invoked with message as first argument.
# message should go to callback function then screen

    rospy.spin()  # prevents python from exiting until this node is stopped

if __name__ == '__main__':
    n3PtRep()


# https://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

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

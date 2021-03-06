#!/usr/bin/env python
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
#
# Revision $Id$

import rospy
import serial
import struct
import binascii
from geometry_msgs.msg import Twist
from xbee import ZigBee

xbee = None
XBEE_ADDR_LONG = '\x00\x13\xA2\x00\x40\x31\x56\x46'
XBEE_ADDR_SHORT = '\x10\x52'
DEVICE = '/dev/ttyUSB0'


def print_data(data):
    rospy.loginfo("XBee Response: %s" % data)

def prepare_data(msg):
    linear = 0;
    angular = 0;

    if msg.linear.x > 0:
        linear = 1
    elif msg.linear.x < 0:
        linear = 2

    if msg.angular.z > 0:
        angular = 2
    elif msg.angular.z < 0:
        angular = 1

    data = struct.pack('BBB', 1, linear, angular)
    return data

def cmd_vel_command(msg):
    #a,b,c
    #a = 1
    #b = 1 - forward, b = -1 - backward
    #c > 0 - turn right, c < 0 - turn left

    data = prepare_data(msg)

    rospy.loginfo("Sending: %s" % binascii.hexlify(data))

    xbee.tx(
        dest_addr_long = XBEE_ADDR_LONG,
        dest_addr = XBEE_ADDR_SHORT,
        data=data,
    )

def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))
    
    cmd_vel_command(msg)
    
    
def listener():
    global xbee

    ser = serial.Serial(DEVICE, 9600)
    xbee = ZigBee(ser, callback=print_data)

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'talker' node so that multiple talkers can
    # run simultaenously.
    rospy.init_node('cmd_vel_listener', anonymous=True)

    rospy.Subscriber("/cmd_vel", Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    xbee.halt()
    ser.close()
        
if __name__ == '__main__':
    listener()

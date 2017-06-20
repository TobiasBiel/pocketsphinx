#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib; roslib.load_manifest('pocketsphinx')
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String


class voice_cmd_vel:

    def __init__(self):
        rospy.on_shutdown(self.cleanup)
        self.speed = 0.2
        self.msg = Twist()

        # publish to cmd_vel, subscribe to speech output
        self.pub_ = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('recognizer/output', String, self.speechCb)

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            self.pub_.publish(self.msg)
            r.sleep()
        rospy.spin()
        
    def speechCb(self, msg):
        rospy.loginfo("Got data: " + msg.data)

        if "full speed" in msg.data or "fast" in msg.data:
            if self.speed == 0.2:
                self.msg.linear.x = self.msg.linear.x*2
                self.msg.angular.z = self.msg.angular.z*2
                self.speed = 0.4
        if "half speed" in msg.data or "slow" in msg.data:
            if self.speed == 0.4:
                self.msg.linear.x = self.msg.linear.x/2
                self.msg.angular.z = self.msg.angular.z/2
                self.speed = 0.2

        if "forward"in msg.data or "ahead" in msg.data or "move" in msg.data:
            self.msg.linear.x = self.speed
            self.msg.angular.z = 0
        elif "left" in msg.data:
            if self.msg.linear.x != 0:
                if self.msg.angular.z < self.speed:
                    self.msg.angular.z += 0.05
            else:        
                self.msg.angular.z = self.speed*2
        elif "right" in msg.data:
            if self.msg.linear.x != 0:
                if self.msg.angular.z > -self.speed:
                    self.msg.angular.z -= 0.05
            else:        
                self.msg.angular.z = -self.speed*2
        elif "back" in msg.data:
            self.msg.linear.x = -self.speed
            self.msg.angular.z = 0
        elif "stop" in msg.data or "halt" in msg.data:
            self.msg = Twist()

        self.pub_.publish(self.msg)

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.pub_.publish(twist)

if __name__=="__main__":
    rospy.init_node('voice_cmd_vel')
    try:
        voice_cmd_vel()
    except:
        pass


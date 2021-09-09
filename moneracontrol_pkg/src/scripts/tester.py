#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

class Node:
    def __init__(self):
        rospy.init_node("tester", anonymous=True)
        rospy.Subscriber("/bacteria_2/vel", Twist, self.twistCallback)
    def update(self):
        rospy.spin()
    def twistCallback(self, twistMsg):
        l_x = twistMsg.linear.x
        l_y = twistMsg.linear.y
        l_z = twistMsg.linear.z
        a_x = twistMsg.angular.x
        a_y = twistMsg.angular.y
        a_z = twistMsg.angular.z
        print(twistMsg)

if __name__ == '__main__':
    node = Node()
    try:
        node.update()
    except rospy.ROSInterruptException:
        pass
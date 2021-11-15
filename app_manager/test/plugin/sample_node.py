#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def sample_node():
    rospy.init_node('sample_node', anonymous=True)
    pub = rospy.Publisher('/test_plugin', String, queue_size=10)
    param1 = rospy.get_param('~param1')
    param2 = rospy.get_param('~param2')
    rospy.loginfo('~A started.'.format(rospy.get_name()))
    rospy.loginfo('param1: {}'.format(param1))
    rospy.loginfo('param2: {}'.format(param2))
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish("{{'param1': '{}', 'param2': '{}'}}".format(param1, param2))
        rate.sleep()

if __name__ == '__main__':
    try:
        sample_node()
    except rospy.ROSInterruptException:
        pass

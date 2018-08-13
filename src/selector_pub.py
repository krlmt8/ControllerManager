#!/usr/bin/env python
'''selector ROS Node'''
# license removed for brevity
import rospy
from std_msgs.msg import String, UInt8
seq = ['6','4','1','2','3','1','2']

def talker():
    '''selector Publisher'''
    pub = rospy.Publisher('chatter', UInt8, queue_size=10)
    rospy.init_node('selector', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        for g in seq:
            gesture = int(g)
            rospy.loginfo(gesture)
            pub.publish(gesture)
            rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

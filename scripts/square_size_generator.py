#!/usr/bin/env python
# Code written by Mutiah Apampa

import rospy
import random
from ar_week10_test.msg import square_params

def talker():
    # Initialise Publisher
    pub = rospy.Publisher('square_params', square_params, queue_size=10)
    # Initialise node
    rospy.init_node('points_generator', anonymous=True)
    # Set rate
    rate = rospy.Rate(0.05) # 0.05hz

    value = square_params()

    while not rospy.is_shutdown():
        # Length
        value.length = round(random.uniform(0.05, 0.20), 6)

        # Useful during testing to display and log values
        rospy.loginfo(value)

        # Publish values
        pub.publish(value)
        
        rate.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
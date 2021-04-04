#! /usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
pub = None

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 5),
        'fright': min(min(msg.ranges[144:287]), 5),
        'front':  min(min(msg.ranges[288:431]), 5),
        'fleft':  min(min(msg.ranges[432:575]), 5),
        'left':   min(min(msg.ranges[576:719]), 5),
    }

    take_action(regions)

def take_action(regions):
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0

    state_description = ''

    if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 1 - nothing'
        msg.linear.x = 1
        msg.angular.z = 0
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
        state_description = 'case 2 - front'
        msg.linear.x = 0
        msg.angular.z = 1
    elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 3 - fright'
        msg.linear.x = 0
        msg.angular.z = 1
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 4 - fleft'
        msg.linear.x = 0
        msg.angular.z = -1
    elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
        state_description = 'case 5 - front and fright'
        msg.linear.x = 0
        msg.angular.z = 1
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
        state_description = 'case 6 - front and fleft'
        msg.linear.x = 0
        msg.angular.z = -1
    elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 7 - front and fleft and fright'
        msg.linear.x = 0
        msg.angular.z = 1
    elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
        state_description = 'case 8 - fleft and fright'
        msg.linear.x = 1
        msg.angular.z = 0
    else:
        state_description = 'unknown case'
        rospy.loginfo(regions)

    rospy.loginfo(state_description)

    pub.publish(msg)

def main():
    global pub

    rospy.init_node('reading_laser')

    pub = rospy.Publisher('/Diff_Drive/diff_drive_controller/cmd_vel', Twist, queue_size=1)

    sub = rospy.Subscriber('/Diff_Drive/laser/scan1', LaserScan, clbk_laser)

    rospy.spin()

if __name__ == '__main__':
    main()

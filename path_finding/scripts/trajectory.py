#!/usr/bin/env python

import rospy
import tf
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

path = Path()
clear = False
path.header.frame_id = 'map'


def clear_path(data):
    global clear
    clear = data.data
    if data.data:
        print("Path cleared")


if __name__ == "__main__":
    rospy.init_node('trajectory', anonymous=True)
    listener = tf.TransformListener()
    rospy.Subscriber('clear_trajectory', Bool, clear_path)
    pub = rospy.Publisher('trajectory', Path, queue_size=1)
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if clear:
            path = Path()
            path.header.frame_id = 'map'
            clear = False
        try:
            (trans, rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = trans[0]
        p.pose.position.y = trans[1]
        path.header.stamp = rospy.Time.now()
        path.poses.append(p)
        pub.publish(path)

        rate.sleep()

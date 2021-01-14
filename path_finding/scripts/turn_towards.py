#!/usr/bin/env python

import rospy
import tf
from math import pi, atan2
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from path_finding.srv import Turn_towards_srv, Turn_towards_srvResponse


def normalize_angle(_angle):
    if -pi < _angle < pi:
        return _angle
    if _angle > 2*pi:
        return _angle - 2*pi
    if _angle < -2*pi:
        return _angle + 2*pi
    if _angle > pi:
        return 2*pi - _angle
    if _angle < pi:
        return -2*pi - _angle


def turn_towards(robot_frame, point):
    vel_msg = Twist()
    tfl.waitForTransform(robot_frame, '/map', rospy.Time(), rospy.Duration(5))
    (trans, _) = tfl.lookupTransform(robot_frame, 'map', rospy.Time(0))
    x_prim = point[0] - trans[0]
    y_prim = point[1] - trans[1]
    angle = atan2(y_prim, x_prim)
    print('############################## angle: '+str(angle)+' #################')
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            tfl.waitForTransform(robot_frame, 'map', rospy.Time(), rospy.Duration(0.05))
            (_, rot) = tfl.lookupTransform(robot_frame, '/map', rospy.Time(0))
            rot = euler_from_quaternion(rot)
            turn_angle = normalize_angle(rot[2]-angle)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as err:
            print(err)
            continue
        print('##################### rot: ' +str(rot) +'## angle-rot: '+str(turn_angle) +'##########')
        if abs(turn_angle) < 0.1:
            vel_msg.angular.z = 0
            pub.publish(vel_msg)
            break
        vel_msg.angular.z = turn_angle
        pub.publish(vel_msg)
        rate.sleep()


def handle_request(req):
    turn_towards(req.robot_frame, [req.x, req.y])
    return Turn_towards_srvResponse(True)


if __name__ == "__main__":
    rospy.init_node('turn_towards_server')
    pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
    tfl = tf.TransformListener()
    srv = rospy.Service('turn_towards_srv', Turn_towards_srv, handle_request)
    rospy.spin()

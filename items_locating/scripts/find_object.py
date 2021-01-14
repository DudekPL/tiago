#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
from items_locating.srv import Find_object, Find_objectResponse

global_data = False


def image_callback(data):
    global global_data
    msg = Bool()
    msg.data = False

    try:
        frame = bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError, e:
        print(e)
        return
    np_frame = np.array(frame, dtype=np.uint8)

    hsv_frame = cv.cvtColor(np_frame, cv.COLOR_RGB2HSV)
    cv.imshow("hsv", hsv_frame)
    mask = cv.inRange(hsv_frame, np_boundaries[0], np_boundaries[1])
    cv.imshow("masks", mask)
    _, contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        c = max(contours, key=cv.contourArea)
        x, y, w, h = cv.boundingRect(c)
        cv.putText(np_frame, 'w: '+str(w)+' h: '+str(h), (30, 300), cv.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 1, cv.LINE_AA)
        if w >= 30 and h >= 50:
            msg.data = True
    cv.imshow(window_name, np_frame)
    global_data = msg.data
    pub.publish(msg)
    cv.waitKey(50)


def handle_find_object(request):
    rospy.loginfo("############################ searching, if found: " + str(global_data)+" ##########################")
    return Find_objectResponse(global_data)


def shutdown_hook():
    cv.destroyAllWindows()


if __name__ == '__main__':
    window_name = "CameraRAW"
    dtype = "uint8"
    boundaries = [[60, 0, 0], [70, 255, 255]]
    np_boundaries = [np.array(boundaries[0], dtype), np.array(boundaries[1], dtype)]
    rospy.init_node('cv_find_object')
    pub = rospy.Publisher('/find_object', Bool, queue_size=1)
    bridge = CvBridge()
    cv.namedWindow(window_name, cv.WINDOW_AUTOSIZE)
    cv.namedWindow("masks", cv.WINDOW_AUTOSIZE)
    rospy.on_shutdown(shutdown_hook)
    cv.waitKey(30)
    sub = rospy.Subscriber('/xtion/rgb/image_raw', Image, image_callback, queue_size=1)
    srv = rospy.Service('/find_object_srv', Find_object, handle_find_object)
    rospy.spin()

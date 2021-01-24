#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import DeleteModel
from items_locating.srv import Find_object
from tf.transformations import quaternion_from_euler
from path_finding.srv import Plan_path, Plan_pathRequest
from pickup.srv import pickup_objects, pickup_objectsResponse

start_point = 'spawn'


class Point:
    def __init__(self, _id, _object, _x, _y, _yaw):
        self.id = _id
        self.object = _object
        self.x = _x
        self.y = _y
        self.yaw = _yaw


def goal_msg(_point):
    """

    :type _point: Point
    """
    _goal = MoveBaseGoal()
    _goal.target_pose.header.frame_id = "map"
    _goal.target_pose.header.stamp = rospy.Time.now()
    _position = _goal.target_pose.pose.position
    _position.x = _point.x
    _position.y = _point.y
    _position.z = 0
    _quaternion = quaternion_from_euler(0, 0, _point.yaw)
    _orientation = _goal.target_pose.pose.orientation
    _orientation.x = _quaternion[0]
    _orientation.y = _quaternion[1]
    _orientation.z = _quaternion[2]
    _orientation.w = _quaternion[3]
    return _goal


def pickup_object_service(_req):
    to_pickup = _req.objects

    path_request = Plan_pathRequest()
    path_request.world_name = world_name
    path_request.objects = to_pickup

    path_response = plan_path(path_request)
    path = path_response.path

    move_base_client.wait_for_server()
    picked_up = []
    for p in path:
        if p.object in picked_up:  # skip points with objects that are already picked up
            continue
        goal = goal_msg(p)
        move_base_client.send_goal(goal)
        wait = move_base_client.wait_for_result()
        if not wait:
            rospy.logerr("Move base action server not available")
            rospy.signal_shutdown("Move base action server not available")
        state = move_base_client.get_state()
        if state not in [3]:  # SUCCEEDED
            rospy.logerr("Move base server failed: object: {}   id: {}  state is: {}".format(p.object, p.id, state))
            msg = "Move base server failed: object: {}   id: {}  state is: {}".format(p.object, p.id, state)
            rospy.signal_shutdown(msg)
        rospy.sleep(1)
        find_object_response = find_object()
        if find_object_response.object_found:
            remove_model_srv(p.object)
            picked_up.append(p.object)
    response = pickup_objectsResponse()
    not_picked_up = list(set(to_pickup)-set(picked_up))
    response.done = len(not_picked_up) == 0
    return response


if __name__ == '__main__':
    rospy.init_node('pickup_node', anonymous=True)
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # srv for removing picked up objects
    rospy.wait_for_service('gazebo/delete_model')
    remove_model_srv = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    # srv for checking if object is seen by robot
    rospy.wait_for_service('find_object_srv')
    find_object = rospy.ServiceProxy('find_object_srv', Find_object)
    # srv for planning path
    rospy.wait_for_service('plan_path_srv')
    plan_path = rospy.ServiceProxy('plan_path_srv', Plan_path)

    # loading params
    world_name = rospy.get_param('world_name')

    main_service = rospy.Service('pickup_objects', pickup_objects, pickup_object_service)
    rospy.spin()

#!/usr/bin/env python

import rospy
import rospkg
import itertools as itt
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.srv import DeleteModel
from items_locating.srv import Find_object
import sqlite3 as sql
from tf.transformations import quaternion_from_euler
import random

start_point = 'spawn'


class Point:

    def __init__(self, _id, _object, _x, _y, _yaw):
        self.id = _id
        self.object = _object
        self.x = _x
        self.y = _y
        self.yaw = _yaw


class Object:

    def __init__(self, _name, _expiring, _priority):
        self.name = _name
        self.expiring = _expiring
        self.priority = _priority


def get_priority(_point):
    return objects[_point.object].priority


def path_length(_path):
    prev_p = _path[0]
    _dist = spawn_dist[_path[0].id]
    for point in _path:
        _dist += dist_dict[prev_p.id][point.id]
        prev_p = point
    _dist += spawn_dist[prev_p.id]
    return _dist


def plan_path2():
    sorted_points = sorted(points, reverse=True, key=get_priority)
    groups = itt.groupby(sorted_points, key=get_priority)
    whole_path = []
    for _priority, _group in groups:
        perm = itt.permutations(_group)
        shortest_dist = float('inf')
        shortest_path = []
        for _path in perm:
            _dist = path_length(_path)
            if _dist < shortest_dist:
                shortest_dist = _dist
                shortest_path = _path
        for point in shortest_path:
            print(point.object + ' ' + str(point.id))
            whole_path.append(point)
        whole_path.append(spawn)
    return whole_path


def plan_path(d=2, k=4, Imax=3):
    shortest_path = []
    shortest_dist = float('inf')
    _sorted_points = sorted(points, reverse=True, key=get_priority)
    for i in range(0, Imax):
        _path = initialize_GRASP(_sorted_points, d, k)
        _dist = path_length(_path)
        if _dist < shortest_dist:
            shortest_dist = _dist
            shortest_path = list(_path)
    return shortest_path


def choose_random(_list):
    return random.choice(_list)


def initialize_GRASP(_sorted_points, d, k):
    _groups = itt.groupby(_sorted_points, key=get_priority)
    _points_to_visit = list(_sorted_points)
    _path = []
    last_point = spawn
    for priority, group in _groups:
        d_relaxed = [_p for _p in _points_to_visit if get_priority(_p) >= priority-d]
        current_group = [_p for _p in d_relaxed if get_priority(_p) == priority]
        while len(current_group) > 0:  # not empty
            d_relaxed = sorted(d_relaxed, key=lambda _p: dist_dict[last_point.id][_p.id])
            restricted_candidates = d_relaxed[:k]
            print('############################## {} {} {}######################1'.format(len(d_relaxed), len(restricted_candidates), len(current_group)))
            _random = choose_random(restricted_candidates)
            _path.append(_random)
            # remove point from lists
            if get_priority(_random) == priority:
                current_group.remove(_random)
            d_relaxed.remove(_random)
            _points_to_visit.remove(_random)
        _path.append(spawn)
    return _path


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


def get_objects_from_db(_cur, _objects_names):
    _query = "SELECT name, expiring, prority FROM objects WHERE name IN ("+",".join(["?"]*len(_objects_names))+")"
    _cur.execute(_query, _objects_names)
    _dict = {}
    for _row in _cur.fetchall():
        _dict[_row['name']] = Object(_row['name'], _row['expiring'], _row['prority'])
    return _dict


def get_look_up_points_from_db(_cur, _objects):
    _query = "SELECT * FROM look_up_points WHERE map=(?) AND object IN ("+",".join(["?"]*len(_objects))+")"
    _args = [world_name]
    _args.extend([_o.name for _o in _objects])
    _cur.execute(_query, _args)
    _list = []
    for _row in _cur.fetchall():
        _list.append(Point(_row['id'], _row['object'], _row['x'], _row['y'], _row['yaw']))
    return _list


def get_distances_dict_from_db(_cur, _points, _spawn):
    _all_points = list(_points)
    _all_points.append(_spawn)
    _query = 'SELECT * FROM distances WHERE start IN (' +",".join(["?"] * len(_all_points)) + ")"
    _query = _query + 'AND finish IN (' +",".join(["?"] * len(_all_points)) + ")"
    _args = [_p.id for _p in _all_points]
    _args.extend([_p.id for _p in _all_points])
    _cur.execute(_query, _args)
    _dict = {_p.id: {} for _p in _all_points}
    for _row in _cur.fetchall():
        _dict[_row['start']][_row['finish']] = _row['distance']
    return _dict


def get_spawn_from_db(_cur, _map):
    _query = 'SELECT * FROM look_up_points WHERE object=(?) AND map=(?)'
    _cur.execute(_query, [start_point, _map])
    _row = _cur.fetchone()
    return Point(_row['id'], _row['object'], _row['x'], _row['y'], _row['yaw'])


def get_spawn_distances_from_db(_cur, _spawn_id):
    _query = 'SELECT * FROM distances WHERE start=(?)'
    _cur.execute(_query, [_spawn_id])
    _dict = {}
    for _row in _cur.fetchall():
        _dict[_row['finish']] = _row['distance']
    return _dict


if __name__ == '__main__':
    rospy.init_node('pickup_node', anonymous=True)
    move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    # srv for removing picked up objects
    rospy.wait_for_service('gazebo/delete_model')
    remove_model_srv = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    # srv for checking if object is seen by robot
    rospy.wait_for_service('find_object_srv')
    find_object = rospy.ServiceProxy('find_object_srv', Find_object)
    # connection to database
    db_path = rospkg.RosPack().get_path('path_finding')
    conn = sql.connect(db_path + "/../test.db")
    conn.row_factory = sql.Row
    cursor = conn.cursor()
    # loading params
    to_pickup = rospy.get_param('/objects_to_pickup')
    world_name = rospy.get_param('world_name')
    # selects from db
    spawn = get_spawn_from_db(cursor, world_name)
    objects = get_objects_from_db(cursor, to_pickup)
    points = get_look_up_points_from_db(cursor, objects.values())
    dist_dict = get_distances_dict_from_db(cursor, points, spawn)
    spawn_dist = get_spawn_distances_from_db(cursor, spawn.id)

    path = plan_path()

    move_base_client.wait_for_server()
    picked_up = []
    print("############################ start to pick up ##########################")
    for p in path:
        if p.object in picked_up:  # skip points with objects that are already picked up
            continue
        goal = goal_msg(p)
        print("############################ going to: "+str(goal.target_pose.pose.position.x) +
              str(goal.target_pose.pose.position.y) + " ##########################")
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
        else:
            print("###################### object: {}    id: {}   arrived ###############".format(p.object, p.id))
        rospy.sleep(1)
        find_object_response = find_object()
        if find_object_response.object_found:
            remove_model_srv(p.object)
            picked_up.append(p.object)
            print("############################ picked up ##########################")
    rospy.spin()

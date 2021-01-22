#!/usr/bin/env python

import rospkg
import sqlite3 as sql
import rospy
from path_finding.srv import Plan_path_srv, Plan_path_srvResponse
from path_finding.msg import Point
import itertools as itt
import random

start_point = 'spawn'


class Object:
    def __init__(self, _name, _expiring, _priority):
        self.name = _name
        self.expiring = _expiring
        self.priority = _priority


def get_objects_from_db(_cur, _objects_names):
    _query = "SELECT name, expiring, prority FROM objects WHERE name IN ("+",".join(["?"]*len(_objects_names))+")"
    _cur.execute(_query, _objects_names)
    _dict = {}
    for _row in _cur.fetchall():
        _dict[_row['name']] = Object(_row['name'], _row['expiring'], _row['prority'])
    return _dict


def get_look_up_points_from_db(_cur, _objects, _world_name):
    _query = "SELECT * FROM look_up_points WHERE map=(?) AND object IN ("+",".join(["?"]*len(_objects))+")"
    _args = [_world_name]
    _args.extend([_o.name for _o in _objects])
    _cur.execute(_query, _args)
    _list = []
    for _row in _cur.fetchall():
        _list.append(Point(_row['id'], _row['object'], _row['x'], _row['y'], _row['yaw']))
    return _list


def get_distances_dict_from_db(_cur, _points, _spawn):
    _all_points = list(_points)
    _all_points.append(_spawn)
    _query = 'SELECT * FROM distances WHERE start IN (' + ",".join(["?"] * len(_all_points)) + ")"
    _query = _query + 'AND finish IN (' + ",".join(["?"] * len(_all_points)) + ")"
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


def get_priority(_point, _objects):
    return _objects[_point.object].priority


def path_length(_path, _spawn_distances, _distance_dictionary):
    prev_p = _path[0]
    _dist = _spawn_distances[_path[0].id]
    for point in _path:
        _dist += _distance_dictionary[prev_p.id][point.id]
        prev_p = point
    _dist += _spawn_distances[prev_p.id]
    return _dist


def choose_random(_list):
    return random.choice(_list)


def initialize_GRASP(_sorted_points, _d, _k, _spawn_point, _distance_dictionary, _objects):
    _groups = itt.groupby(_sorted_points, key=lambda _p: get_priority(_p, _objects))
    _points_to_visit = list(_sorted_points)
    _path = []
    last_point = _spawn_point
    for priority, group in _groups:
        d_relaxed = [_p for _p in _points_to_visit if get_priority(_p, _objects) >= priority - _d]
        current_group = [_p for _p in d_relaxed if get_priority(_p, _objects) == priority]
        while len(current_group) > 0:  # not empty
            d_relaxed = sorted(d_relaxed, key=lambda _p: _distance_dictionary[last_point.id][_p.id])
            restricted_candidates = d_relaxed[:_k]
            _random = choose_random(restricted_candidates)
            _path.append(_random)
            # remove point from lists
            if get_priority(_random, _objects) == priority:
                current_group.remove(_random)
            d_relaxed.remove(_random)
            _points_to_visit.remove(_random)
        _path.append(_spawn_point)
    return _path


def plan_path(_req, _Imax, _d, _k):
    to_pickup = _req.objects
    world_name = _req.world_name
    # connection to database
    db_path = rospkg.RosPack().get_path('path_finding')
    conn = sql.connect(db_path + "/../test.db")
    conn.row_factory = sql.Row
    cursor = conn.cursor()
    # selects from db
    spawn = get_spawn_from_db(cursor, world_name)
    objects = get_objects_from_db(cursor, to_pickup)
    points = get_look_up_points_from_db(cursor, objects.values(), world_name)
    dist_dict = get_distances_dict_from_db(cursor, points, spawn)
    spawn_dist = get_spawn_distances_from_db(cursor, spawn.id)

    shortest_path = []
    shortest_dist = float('inf')
    _sorted_points = sorted(points, reverse=True, key=lambda _p: get_priority(_p, objects))
    for i in range(0, _Imax):
        _path = initialize_GRASP(_sorted_points, _d, _k, spawn, dist_dict, objects)
        _dist = path_length(_path, spawn_dist, dist_dict)
        if _dist < shortest_dist:
            shortest_dist = _dist
            shortest_path = list(_path)
    conn.close()
    response = Plan_path_srvResponse()
    response.path = shortest_path
    return response


if __name__ == '__main__':
    rospy.init_node('plan_path')
    Imax = 5
    d = 2
    k = 4
    rospy.Service('plan_path_srv', Plan_path_srv, lambda _req: plan_path(_req, Imax, d, k))
    rospy.spin()

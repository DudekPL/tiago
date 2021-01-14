#!/usr/bin/env python
import rospkg

import rospy
from nav_msgs import msg as nav
from geometry_msgs import msg as geom
import tf2_ros
from math import sqrt
import sqlite3 as sql

got_msg = False
planned_path = nav.Path


class Point:
    def __init__(self, _object, _x, _y, _id):
        self.object = _object
        self.id = _id
        self.x = _x
        self.y = _y

    @staticmethod
    def points_list(_points_cursor):
        _list = []
        for _row in _points_cursor:
            _p = Point(_row['object'], _row['x'], _row['y'], _row['id'])
            _list.append(_p)
        return _list


class Distance:
    def __init__(self, _start_id, _finish_id, _distance):
        self.start_id = _start_id
        self.finish_id = _finish_id
        self.distance = _distance

    @staticmethod
    def distances_list(_points_list):
        _list = []
        for _start in _points_list:
            for _finish in _points_list:
                _dist = 0 if _start.id == _finish.id else Distance.distance(_start, _finish)
                _list.append(Distance(_start.id, _finish.id, _dist))
        return _list

    @staticmethod
    def distance_from_path(_poses):
        _prev = _poses[0]
        _dist = 0
        for _pose in _poses:
            _length2 = (_pose.pose.position.x - _prev.pose.position.x) ** 2 + (
                        _pose.pose.position.y - _prev.pose.position.y) ** 2
            _dist = _dist + sqrt(_length2)
            _prev = _pose
        return _dist

    @staticmethod
    def distance(_start, _finish):
        global got_msg
        got_msg = False
        _msg = goal_msg(_finish)
        _start_msg = start_tf_msg(_start)
        for _i in range(0, 4):
            tfbr.sendTransform(_start_msg)
            rospy.sleep(0.05)
        pub.publish(_msg)
        while not got_msg:
            tfbr.sendTransform(_start_msg)
            rospy.sleep(0.05)
        return Distance.distance_from_path(planned_path.poses)


def goal_msg(_goal):
    _msg = geom.PoseStamped()
    _msg.header.stamp = rospy.Time.now()
    _msg.header.frame_id = "map"
    _msg.pose.position.x = _goal.x
    _msg.pose.position.y = _goal.y
    _msg.pose.position.z = 0
    return _msg


def start_tf_msg(_start):
    _msg = geom.TransformStamped()
    _msg.header.stamp = rospy.Time.now()
    _msg.header.frame_id = "map"
    _msg.child_frame_id = "base_link"
    _msg.transform.translation.x = _start.x
    _msg.transform.translation.y = _start.y
    _msg.transform.translation.z = 0
    _msg.transform.rotation = geom.Quaternion(0, 0, 0, 1)
    return _msg


def sub_call(data):
    global got_msg
    global planned_path
    planned_path = data
    got_msg = True


def create_distances_table(_cursor):
    _cursor.execute('CREATE TABLE IF NOT EXISTS "distances" ('
                    '"start"	INTEGER NOT NULL,'
                    '"finish"	INTEGER NOT NULL,'
                    '"distance"	REAL NOT NULL,'
                    '"id"	INTEGER NOT NULL UNIQUE,'
                    'PRIMARY KEY("id" AUTOINCREMENT),'
                    'FOREIGN KEY("start") REFERENCES "look_up_points"("id"),'
                    'FOREIGN KEY("finish") REFERENCES "look_up_points"("id"),'
                    'UNIQUE("start","finish"));')


def save_distances(_distances, _cursor):
    create_distances_table(_cursor)
    _sql_distances = [(_dist.start_id, _dist.finish_id, _dist.distance) for _dist in _distances]
    _cursor.executemany('INSERT OR REPLACE INTO distances (start, finish, distance) VALUES (?, ?, ?)', _sql_distances)


if __name__ == '__main__':
    rospy.init_node("create_dist_matrix", anonymous=True)
    pub = rospy.Publisher("planner/goal", geom.PoseStamped, queue_size=10)
    sub = rospy.Subscriber("planner/planner/plan", nav.Path, sub_call)
    tfbr = tf2_ros.TransformBroadcaster()

    world_name = rospy.get_param('world_name')

    db_path = rospkg.RosPack().get_path('path_finding')
    conn = sql.connect(db_path + "/../test.db")
    conn.row_factory = sql.Row
    cursor = conn.cursor()
    cursor.execute("SELECT id, object, x, y FROM look_up_points WHERE map=(?)", [world_name])
    points_list = Point.points_list(cursor)

    cursor.execute("SELECT spawn_x, spawn_y FROM maps WHERE name=(?)", [world_name])
    row = cursor.fetchone()
    for i in range(0, 100, 5):
        rospy.sleep(0.05)
        tfbr.sendTransform(start_tf_msg(Point("", row['spawn_x'], row['spawn_y'], '')))

    dist_list = Distance.distances_list(points_list)
    save_distances(dist_list, cursor)
    conn.commit()

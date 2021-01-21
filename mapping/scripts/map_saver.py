#!/usr/bin/env python
from doctest import master

import rospy
import sqlite3 as sql
import Tkinter as Tk
from threading import Thread
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import roslaunch
import rospkg
import os

class App:

    def __init__(self, _world_name):
        self.point = PoseStamped()
        self.list_to_sql_id = {}
        self.angle = 0
        self.world_name = _world_name
        self.master = None
        self.conn = None
        self.c = None
        self.x_output = None
        self.y_output = None
        self.w_output = None

    def save_button_callback(self):
        print ('####################################################')
        self.c.execute('SELECT * FROM maps WHERE name=(?)', [self.world_name])
        if len(self.c.fetchall()) > 0:
            print("UPDATE")
            _query = "UPDATE maps SET name=(?), spawn_x=(?), spawn_y=(?), spawn_yaw=(?) WHERE name=(?)"
            self.c.execute(_query, [self.world_name, self.point.pose.position.x,
                                    self.point.pose.position.y, self.angle, self.world_name])
            _query = "UPDATE look_up_points SET x=(?), y=(?), yaw=(?) WHERE object=(?) AND map=(?)"
            self.c.execute(_query, [self.point.pose.position.x, self.point.pose.position.y, self.angle,
                                    'spawn', self.world_name])
        else:
            print("INSERT")
            _query = "INSERT INTO maps (name, spawn_x, spawn_y, spawn_yaw) VALUES ( (?), (?), (?), (?))"
            self.c.execute(_query, [self.world_name, self.point.pose.position.x,
                                    self.point.pose.position.y, self.angle])
            _query = "INSERT INTO look_up_points (object, x, y, map, yaw) VALUES ( (?), (?), (?), (?), (?)) "
            self.c.execute(_query, ['spawn', self.point.pose.position.x, self.point.pose.position.y,
                                    self.world_name, self.angle])
        self.master.quit()
        self.master.destroy()
        print("COMMIT")
        self.conn.commit()

    def window_func(self):
        self.conn = sql.connect(pkg_path+'/../test.db')
        self.conn.row_factory = sql.Row
        self.c = self.conn.cursor()
        self.master = Tk.Tk()
        self.master.title("map_saver")

        Tk.Label(master, text=self.world_name).grid(row=0, column=0, columnspan=2)

        Tk.Label(self.master, text="Spawn point:").grid(row=1, columnspan=2)
        Tk.Label(self.master, text="x").grid(row=2)
        Tk.Label(self.master, text="y").grid(row=3)
        Tk.Label(self.master, text='yaw').grid(row=4)

        self.x_output = Tk.Label(self.master, text=str(self.point.pose.position.x))
        self.x_output.grid(row=2, column=1)

        self.y_output = Tk.Label(self.master, text=str(self.point.pose.position.y))
        self.y_output.grid(row=3, column=1)

        self.w_output = Tk.Label(self.master, text=str(0))
        self.w_output.grid(row=4, column=1)

        add_but = Tk.Button(self.master, text='Commit', command=self.save_button_callback)
        add_but.grid(row=5, columnspan=2)

        self.master.mainloop()
        rospy.signal_shutdown("Window closed by user")


def sub_callback(data):
    """

    :type data: PoseStamped
    """
    global app
    print(data)
    app.x_output.config(text=data.pose.position.x)
    app.y_output.config(text=data.pose.position.y)
    _o = data.pose.orientation
    _angle = euler_from_quaternion([_o.x, _o.y, _o.z, _o.w])
    app.w_output.config(text=_angle[2])
    app.point = data
    app.angle = _angle[2]


if __name__ == "__main__":
    rospy.init_node("base_access")
    world_name = rospy.get_param('world_name')
    pkg_path = rospkg.RosPack().get_path('mapping')
    save_path = pkg_path + '/maps/' + world_name
    if not os.path.exists(save_path):
        os.makedirs(save_path)
    _node = roslaunch.core.Node(package='map_server', required=True,  node_type='map_saver',
                                args='-f '+save_path+'/map', output='screen')
    _launch = roslaunch.scriptapi.ROSLaunch()
    _launch.start()
    _launch.launch(_node)
    app = App(world_name)
    t = Thread(target=app.window_func)
    t.start()
    rospy.Subscriber('move_base_simple/goal', PoseStamped, sub_callback)
    rospy.spin()
    t.join()

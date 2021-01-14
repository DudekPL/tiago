#!/usr/bin/env python
import rospkg
from doctest import master

import rospy
import sqlite3 as sql
import Tkinter as Tk
from threading import Thread
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


class App:

    def __init__(self):
        self.point = PoseStamped()
        self.list_to_sql_id = {}
        self.angle = 0

    def add_button_callback(self):
        if self.name_input.get() == "":
            return
        self.c.execute('SELECT * FROM objects WHERE name=(?)', [self.name_input.get()])
        if len(self.c.fetchall()) == 0:
            App.ObjectWindow.create_object_window(self, self.name_input.get())
        else:
            self.c.execute('INSERT INTO look_up_points (object, x, y, yaw, map) VALUES ((?), (?), (?), (?), (?))',
                           [self.name_input.get(), self.point.pose.position.x, self.point.pose.position.y,
                            self.angle, self.maps_spin.get()])
            self.maps_spin_callback()

    def maps_spin_callback(self):
        self.points_list.delete(0, Tk.END)
        self.list_to_sql_id.clear()
        self.c.execute('SELECT * FROM look_up_points WHERE map=(?)', [self.maps_spin.get()])
        _i = 0
        for _r in self.c.fetchall():
            self.points_list.insert(_i, "{}: x={:.3f} y={:.3f} yaw={:.3f}".format(_r['object'], _r['x'], _r['y'], _r['yaw']))
            self.list_to_sql_id[_i] = _r['id']
            _i += 1

    def del_button_callback(self):
        if len(self.points_list.curselection()) > 0:
            self.c.execute('DELETE FROM look_up_points WHERE id=(?)',
                           [self.list_to_sql_id[self.points_list.curselection()[0]]])
            self.maps_spin_callback()

    class ObjectWindow:

        app = None  # type: App

        def close(self):
            self.obj_win.destroy()

        def __init__(self, _app, _name):
            self.app = _app
            self.name = _name
            self.obj_win = Tk.Toplevel(self.app.master)
            Tk.Label(self.obj_win, text="Name ").grid(row=0, column=0)
            Tk.Label(self.obj_win, text="Priority").grid(row=1, column=0)
            Tk.Label(self.obj_win, text="Expiring").grid(row=2, column=0)
            Tk.Label(self.obj_win, text=_name).grid(row=0, column=1)
            self.priority = Tk.Spinbox(self.obj_win, from_=0, to=10)
            self.priority.grid(row=1, column=1)
            self.expiring = Tk.IntVar()
            Tk.Checkbutton(self.obj_win, variable=self.expiring).grid(row=2, column=1)
            Tk.Button(self.obj_win, text="Add", command=self.add_object_button_callback).grid(row=3, column=0)
            Tk.Button(self.obj_win, text="Cancel", command=self.close).grid(row=3, column=1)

        def add_object_button_callback(self):
            self.app.c.execute('INSERT INTO objects (name, expiring, prority) VALUES ((?), (?), (?))',
                               [self.name, self.expiring.get(), self.priority.get()])
            self.close()

        @staticmethod
        def create_object_window(_app, _name):
            _obj_win = App.ObjectWindow(_app, _name)
            _obj_win.obj_win.mainloop()

    def window_func(self):
        self.conn = sql.connect(db_path + '/../test.db')
        self.conn.row_factory = sql.Row
        self.c = self.conn.cursor()
        self.master = Tk.Tk()
        self.master.title("base_access")

        Tk.Label(master, text="Map").grid(row=0, column=0)
        self.c.execute('SELECT name FROM maps')
        self.maps_spin = Tk.Spinbox(master, values=[(r['name']) for r in self.c.fetchall()],
                                    command=self.maps_spin_callback, state="readonly", wrap=True)
        self.maps_spin.grid(row=0, column=1)

        Tk.Label(self.master, text="Object").grid(row=2)
        Tk.Label(self.master, text="Point:").grid(row=1, columnspan=2)
        Tk.Label(self.master, text="x").grid(row=3)
        Tk.Label(self.master, text="y").grid(row=4)
        Tk.Label(self.master, text="yaw").grid(row=5)

        self.name_input = Tk.Entry(self.master)
        self.name_input.grid(row=2, column=1)

        self.x_output = Tk.Label(self.master, text=str(self.point.pose.position.x))
        self.x_output.grid(row=3, column=1)

        self.y_output = Tk.Label(self.master, text=str(self.point.pose.position.y))
        self.y_output.grid(row=4, column=1)

        self.w_output = Tk.Label(self.master, text=str(0))
        self.w_output.grid(row=5, column=1)

        add_but = Tk.Button(self.master, text='Add Point', command=self.add_button_callback)
        add_but.grid(row=6, columnspan=2)

        Tk.Label(self.master, text="List of Points").grid(row=0, column=2)
        self.points_list = Tk.Listbox(self.master, selectmode=Tk.SINGLE, width=50)
        self.points_list.grid(row=1, column=2, rowspan=6)
        self.maps_spin_callback()

        Tk.Button(self.master, text="Delete", command=self.del_button_callback).grid(row=8, column=2)

        Tk.Button(self.master, text="Commit", command=self.conn.commit).grid(row=9, columnspan=3)

        self.master.mainloop()
        rospy.signal_shutdown("Window closed by user")  # TODO: sprawdzic czy dziala


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
    db_path = rospkg.RosPack().get_path('pointer')
    app = App()
    t = Thread(target=app.window_func)
    t.start()
    rospy.Subscriber('move_base_simple/goal', PoseStamped, sub_callback)
    rospy.spin()
    t.join()

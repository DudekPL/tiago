#!/usr/bin/env python

import sys
import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

myargv = rospy.myargv(argv=sys.argv)
sim_variant = int(myargv[1])

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
rospack = rospkg.RosPack()

pose_dic = rospy.get_param('/objects_spawns')
objects_list = pose_dic.keys()

for obj in objects_list:
	no_positions = len(pose_dic[obj])
	pose_list = pose_dic[obj][sim_variant % no_positions]
	sim_variant = (sim_variant - (sim_variant % no_positions)) / no_positions
	initial_pose = Pose()
	pos = initial_pose.position
	pos.x, pos.y, pos.z = pose_list

	try:
		spawn_model_client(
			model_name=obj,
			model_xml=open(rospack.get_path('my_simulation')+'/models/object/object.sdf', 'r').read(),
			robot_namespace='/'+obj,
			initial_pose=initial_pose,
			reference_frame='world'
		)
	except Exception as e:
		print(e)	

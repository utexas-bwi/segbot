"""Function calls for various arm actions"""
import rospy

import actionlib
import actionlib_msgs.msg

from geometry_msgs.msg import *
import std_msgs.msg
from kinova_msgs.msg import *
from sensor_msgs.msg import *
from segbot_arm_manipulation.msg import *
from segbot_arm_perception.srv import *

heard_state = False
current_state = None 

def wait_for_state():
	global heard_state
	heard_state = False
	while not heard_state :
		if heard_state:
			break

def joint_cb (data):
	global heard_state
	global current_state
	heard_state = True
	current_state = data



"""Function to call the perception service that gets objects off the table"""
def get_table_scene():
	rospy.wait_for_service('tabletop_object_detection_service')
	try:
		perception_client = rospy.ServiceProxy('tabletop_object_detection_service', TabletopPerception)
	
		table_scene_resp = perception_client(False, 0.0)
		return table_scene_resp
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

"""function to get the largest object on the table"""
def get_largest_object(cloud_clusters):
	largest_pc_index = -1;
	largest_num_points = -1;
	index = 0
	for pc in cloud_clusters:
		
		num_points_i = pc.height* pc.width
		
		if (num_points_i > largest_num_points):
			largest_num_points = num_points_i
			largest_pc_index = index
		index+=1
	return largest_pc_index;
		
	
		
		

"""Function to grasp an object on the table, returns table scene"""
def arm_grasp():
	"""Send goal to grasp the largest object on the table"""
	action_address = 'segbot_tabletop_grasp_as'
	client = actionlib.SimpleActionClient(action_address, segbot_arm_manipulation.msg.TabletopGraspAction)
	client.wait_for_server()
	
	goal = TabletopGraspGoal()
	goal.action_name = segbot_arm_manipulation.msg.TabletopGraspGoal.GRASP
	goal.grasp_selection_method = segbot_arm_manipulation.msg.TabletopGraspGoal.CLOSEST_JOINTSPACE_SELECTION
	
	table_scene = get_table_scene()
	
	goal.cloud_plane = table_scene.cloud_plane
	goal.cloud_plane_coef = table_scene.cloud_plane_coef
	
	goal.cloud_clusters = table_scene.cloud_clusters
	goal.target_object_cluster_index = get_largest_object(table_scene.cloud_clusters)

	client.send_goal(goal)
	print 'sent goal'
	client.wait_for_result()
	print 'get result'
	return table_scene
	
"""function for lifting an object"""
def arm_lift(table_scene):
	action_address = 'arm_lift_verify_as'
	lift_client = actionlib.SimpleActionClient(action_address, segbot_arm_manipulation.msg.LiftVerifyAction)
	lift_client.wait_for_server()
	
	goal = LiftVerifyGoal()
	largest_index = get_largest_object(table_scene.cloud_clusters)
	
	goal.tgt_cloud = table_scene.cloud_clusters[largest_index]
	goal.bins = 8
	
	lift_client.send_goal(goal)
	lift_client.wait_for_result()
	
	return lift_client.get_result()
	

"""Function for object replacement"""
"""Requires user to save the joint state after the grasp action has completing to be passed as 'grasped_state' """
def arm_replacement(grasped_state):
	action_address = 'segbot_tabletop_grasp_as'
	replace_client = actionlib.SimpleActionClient(action_address, segbot_arm_manipulation.msg.TabletopGraspAction)
	replace_client.wait_for_server()
	
	goal = TabletopGraspGoal()
	goal.action_name = segbot_arm_manipulation.msg.TabletopGraspGoal.REPLACEMENT
	goal.grasped_joint_state = grasped_state
	
	replace_client.send_goal(goal)
	replace_client.wait_for_result()
	
	return replace_client.get_result()
	
"""Function for handing an object over to a person"""
def arm_handover ():
	action_address = 'segbot_tabletop_grasp_as'
	handover_client = actionlib.SimpleActionClient(action_address, segbot_arm_manipulation.msg.TabletopGraspAction)
	handover_client.wait_for_server()
	
	goal = TabletopGraspGoal()
	goal.action_name = segbot_arm_manipulation.msg.TabletopGraspGoal.HANDOVER
	
	handover_client.send_goal(goal)
	handover_client.wait_for_result()
	
	return handover_client.get_result()
	
"""Function to open the fingers of the robot"""
def open_finger():
	action_address = '/m1n6s200_driver/fingers/finger_positions'
	finger_client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.SetFingersPositionAction)
	finger_client.wait_for_server()
	
	goal = kinova_msgs.msg.SetFingersPositionGoal()
	goal.fingers.finger1 = 100
	goal.fingers.finger2 = 100
	goal.fingers.finger3 = 0

	finger_client.send_goal(goal)
	finger_client.wait_for_result()
	
	return finger_client.get_result()
	
def arm_actions_init():
    rospy.Subscriber("/joint_states", JointState , joint_cb)

def arm_node_init():
    rospy.init_node('arm_action_tester', anonymous=True)
    arm_actions_init()

#table_scene = arm_grasp()
#wait_for_state()
#new_state = current_state
#arm_lift(table_scene)
#arm_handover()

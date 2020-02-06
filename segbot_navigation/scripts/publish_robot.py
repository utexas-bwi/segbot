#!/usr/bin/env python
import rospy
from people_msgs.msg import People, Person
from geometry_msgs.msg import Point, PointStamped, PoseWithCovarianceStamped, PoseStamped, Twist


"""
This is a Robot to Person spoofing node that utilizes the estimated pose and commanded velocity of 
a robot, as exposed through a ROSBridge connection.

"""

class MultiRobotPositionInterface:

	def __init__(self):
	    self.robot_pos = PoseWithCovarianceStamped()
	    self.robot_vel = Twist()

	def cmd_vel_callback(self, data):
	    self.robot_vel = data

	def pose_callback(self, data):
	    self.robot_pos = data

	def run(self):
	    rospy.init_node('person_publisher', anonymous=True)
	    person_pub = rospy.Publisher('/people', People, queue_size=1)
	    point_pub = rospy.Publisher('/person_point', PointStamped, queue_size=1)
	    rospy.Subscriber("/connected_robots/pickles/cmd_vel", Twist, self.cmd_vel_callback)
	    rospy.Subscriber("/connected_robots/pickles/amcl_pose", PoseWithCovarianceStamped, self.pose_callback)
	    rate = rospy.Rate(5)
	    
	    while not rospy.is_shutdown():
		people_msg = People()
		people_msg.header.frame_id = "/level_mux_map"
		people_msg.header.stamp = rospy.Time.now()
		person_position = Point()
		person_velocity = Point()


		# Stuff person data
		person_position = self.robot_pos.pose.pose.position
		person_velocity.x = -1 * self.robot_vel.linear.x 
		person_velocity.y = self.robot_vel.angular.z
		person_velocity.z = 0

		person = Person()
		point_viz_msg = PointStamped()

		point_viz_msg.header = people_msg.header
		point_viz_msg.point = person_position

		person.name = "Robot"
		person.position = person_position
		person.velocity = person_velocity
		person.reliability = 0.99

		people_msg.people.append(person)       
		
		person_pub.publish(people_msg)
		point_pub.publish(point_viz_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		mrpi = MultiRobotPositionInterface()
		mrpi.run()
	except rospy.ROSInterruptException:
		pass

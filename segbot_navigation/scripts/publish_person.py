#!/usr/bin/env python
import rospy
from people_msgs.msg import People
from people_msgs.msg import Person
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped

"""
This is a static-person publisher for use in testing the social navigation layers in costmap_2d.
This file publishes a People_msgs People message on the /people topic, which is the expected 
topic for People messages for the social navigation plugins.

This file is for TESTING purposes only.

"""

def pub_fn():
    person_pub = rospy.Publisher('/people', People, queue_size=1)
    point_pub = rospy.Publisher('/person_point', PointStamped, queue_size=1)
    rospy.init_node('person_publisher', anonymous=True)
    rate = rospy.Rate(3) # 10hz
    people_msg = People()
    people_msg.header.frame_id = "base_link"
    people_msg.header.stamp = rospy.Time.now()
    robot_position = Point()
    robot_velocity = Point()

    #set initial position relative to robot base_link
    robot_position.x = 2.0
    robot_position.z = 0.5
    robot_velocity.x = 0.3

    rospy.sleep(2.0)
    while not rospy.is_shutdown():
	person = Person()
	point_viz_msg = PointStamped()

	point_viz_msg.header = people_msg.header
	point_viz_msg.point = robot_position

	person.name = "Max"
	person.position = robot_position
	person.velocity = robot_velocity
	person.reliability = 0.9
	
	people_msg.people.append(person)       
	
        person_pub.publish(people_msg)
        point_pub.publish(point_viz_msg)
	rate.sleep()

if __name__ == '__main__':
    try:
	pub_fn()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

#The next code will move the mini car until the
#laser detects an obstacle.

import rospy
from std_msgs.msg import Int16
from sensor_msgs.msg import LaserScan
import time

def callback_laser_scan(msg):
	
	global obstacle_detected_near
	global obstacle_detected_far

	#[0] laser scan center, with respect to de front if the car
	if (msg.ranges[0] < 0.3):
		print("Nearby bstacle detected ")
		obstacle_detected_near = True
	else:
		obstacle_detected_near = False

	if (msg.ranges[0] < 1.0):
		print("Far away bstacle detected")
		obstacle_detected_far = True
	else:
		obstacle_detected_far = False



def main():

	print("move_mini_car_node")
	rospy.init_node("move_mini_car_node")
	rospy.Subscriber("/scan", LaserScan, callback_laser_scan)
	pub_vel = rospy.Publisher("/AutoNOMOS_mini/manual_control/speed", Int16, queue_size = 10)
	pub_direction = rospy.Publisher("/AutoNOMOS_mini/manual_control/steering", Int16, queue_size = 10)
	loop = rospy.Rate(10)

	#car velocity has a range between -2000 to 2000 
	vel = Int16() 
	direction = Int16()

	global obstacle_detected_near
	obstacle_detected_near = False
	global obstacle_detected_far
	obstacle_detected_far = False

	while not rospy.is_shutdown():

		if obstacle_detected_far:
			vel.data = -150
			direction.data = 0
			pub_vel.publish(vel)
			pub_direction.publish(direction)
			time.sleep(4)
		elif obstacle_detected_near:
			vel.data = 0
		else:
			vel.data = -500
			direction.data = 88

		pub_vel.publish(vel)
		pub_direction.publish(direction)

		loop.sleep()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python
import rospy
import tf
import math
from std_msgs.msg import Int16

SAMPLING_FREQUENCY = 20
MAX_SPEED    = 1.0
MAX_STEERING = 0.7
CAR_LENGTH   = 0.26

def callback_speed(msg):
    global speed
    speed = -msg.data/850.0

def callback_steering(msg):
    global steering
    steering = -(90 - msg.data)/5.0 * math.pi / 180

def odometry(robot_x, robot_y, robot_t, speed, steering):
    simulation_step = 0.001
    n = int(1.0/simulation_step/SAMPLING_FREQUENCY)
    speed    = max(-MAX_SPEED,    min(speed,    MAX_SPEED))
    steering = max(-MAX_STEERING, min(steering, MAX_STEERING))
    for i in range(n):
        robot_x += simulation_step*speed*math.cos(robot_t)
        robot_y += simulation_step*speed*math.sin(robot_t)
        robot_t += simulation_step*speed/CAR_LENGTH*math.tan(steering)
        
    return robot_x, robot_y, robot_t

def main():
    print("Initializing odometry node ...")
    rospy.init_node("odometry")
    rospy.Subscriber("/AutoNOMOS_mini/manual_control/speed",    Int16, callback_speed)
    rospy.Subscriber("/AutoNOMOS_mini/manual_control/steering", Int16, callback_steering)
    loop = rospy.Rate(SAMPLING_FREQUENCY);
    br = tf.TransformBroadcaster()
    global speed, steering
    speed = 0
    steering = 0
    robot_x = 0
    robot_y = 0
    robot_t = 0
    while not rospy.is_shutdown():
        robot_x, robot_y, robot_t = odometry(robot_x, robot_y, robot_t, speed, steering)
        br.sendTransform((robot_x, robot_y, 0), tf.transformations.quaternion_from_euler(0, 0, robot_t),
                         rospy.Time.now(), "base_link", "odom")
        loop.sleep()

if __name__ == '__main__':
    main()
        

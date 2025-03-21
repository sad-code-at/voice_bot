#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Initial energy level
meulas = 100  
pub = None  # Publisher for movement commands

def update_energy(amount):
    global meulas
    meulas += amount
    meulas = max(0, min(meulas, 100))  # Ensure energy stays between 0 and 100
    rospy.loginfo(f"Energy Level: {meulas}")

def move_robot(linear=0.0, angular=0.0, duration=2, energy_cost=5):
    # Moves the robot and decreases energy accordingly
    global meulas

    if meulas <= 0:
        rospy.logwarn("Not enough energy to move!")
        return
    t0 = rospy.Time.now().to_sec()
    t1 = rospy.Time.now().to_sec()
    while(t1-t0 <= duration):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        pub.publish(twist)
        t1 = rospy.Time.now().to_sec()
    #idea: (t1-t0)<duration keep publishing

    # Stop movement after duration
    twist.linear.x = 0
    twist.angular.z = 0
    pub.publish(twist)

    # Reduce energy
    update_energy(-energy_cost)

def voice_command_callback(msg):
    """ Processes voice commands and moves the robot accordingly """
    command = msg.data.lower().strip()

    if command == "forward":
        move_robot(linear=0.2, duration=2, energy_cost=5)
    elif command == "backward":
        move_robot(linear=-0.2, duration=2, energy_cost=5)
    elif command == "forward five seconds":
        move_robot(linear=0.2, duration=5, energy_cost=15)
    elif command == "backward five seconds":
        move_robot(linear=-0.2, duration=5, energy_cost=15)
    elif command == "left":
        move_robot(angular=1.0, duration=3.1416/2, energy_cost=10)
    elif command == "right":
        move_robot(angular=-1.0, duration=3.1416/2, energy_cost=10)
    elif command == "kaboom":
        update_energy(100)
    else:
        rospy.logwarn(f"Invalid Command: {command}")

def main():
    """ Initializes the ROS node and subscribes to the voice command topic """
    global pub
    rospy.init_node("robot_control", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    rospy.Subscriber("/voice_commands", String, voice_command_callback)
    rospy.loginfo("Robot Control Node Started. Listening to voice_commands...")
    
    rospy.spin()  

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")

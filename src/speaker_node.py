#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class VoiceCommandParser:
    def __init__(self):
        rospy.init_node("voice_command_parser", anonymous=True)

        # Subscribe to speech recognition result
        rospy.Subscriber("/speech_recognition/final_result", String, self.voice_callback)

        # Publisher for valid voice commands
        self.command_pub = rospy.Publisher("/voice_commands", String, queue_size=10)

        # List of valid commands
        self.valid_commands = [
            "right", "left", "forward", "backward",
            "forward five seconds", "backward five seconds", "kaboom"
        ]

        rospy.loginfo("Voice Command Parser Node Initialized. Listening for commands...")

    def voice_callback(self, msg):
        command = msg.data.lower().strip()
        rospy.loginfo(f"Received Command: {command}")

        # Validate command
        if command in self.valid_commands:
            rospy.loginfo(f"Valid Command: {command}")
            self.command_pub.publish(command)
        else:
            rospy.logwarn(f"Invalid Command: {command}")

if __name__ == "__main__":
    try:
        VoiceCommandParser()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

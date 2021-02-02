#!/usr/bin/env python
import rospy
import std_msgs

MESSAGE_SEPARATOR = "@@"

ROS_STATE = dict()


def main():
    rospy.init_node("auto_reply")
    ROS_STATE["pub"] = rospy.Publisher("/alexa/reply", std_msgs.msg.String, queue_size=1)
    ROS_STATE["sub"] = rospy.Subscriber("/alexa/command", std_msgs.msg.String, receive_command)
    print("set up publisher and subscriber, spinning...")
    rospy.spin()


def receive_command(msg):
    uuid, command = msg.data.split(MESSAGE_SEPARATOR)
    print("received command {}".format(command))
    if "yes" in command:
        response_msg = MESSAGE_SEPARATOR.join([uuid, "understood, executing {}".format(command), str(0)])
    else:
        response_msg = MESSAGE_SEPARATOR.join(
            [uuid, "I'm not sure about {}, did you mean beer?".format(command), str(1)]
        )
    ROS_STATE["pub"].publish(response_msg)
    print("replied {}".format(response_msg))


if __name__ == "__main__":
    main()

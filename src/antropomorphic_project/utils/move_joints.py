#!/usr/bin/env python3
import time
import rospy
from std_msgs.msg import Float64


class AntropomorphicArmMoverNode(object):
    """antropomorphic arm topics to write on:
    type: std_msgs/Float64
        /antropomorphic_arm/joint1_position_controller/command
        /antropomorphic_arm/joint2_position_controller/command
        /antropomorphic_arm/joint3_position_controller/command
    """

    def __init__(self):
        self.loop_rate = rospy.Rate(1)  # 1hz

        self.publisher_joint1 = rospy.Publisher(
            "/antropomorphic_arm/joint1_position_controller/command",
            Float64,
            queue_size=1,
        )
        self.publisher_joint2 = rospy.Publisher(
            "/antropomorphic_arm/joint2_position_controller/command",
            Float64,
            queue_size=1,
        )
        self.publisher_joint3 = rospy.Publisher(
            "/antropomorphic_arm/joint3_position_controller/command",
            Float64,
            queue_size=1,
        )
        
        # test connection with antropomorphic arm
        self.check_connection()

    def check_connection(self):
        # connect all topic before movement
        self.check_topic_connection(self.publisher_joint1, "joint1")
        self.check_topic_connection(self.publisher_joint2, "joint2")
        self.check_topic_connection(self.publisher_joint3, "joint3")

        # node acknowledgement
        rospy.loginfo("Antropomorphic arm mover node ready for movement!")

    def check_topic_connection(self, publisher, topic):
        connection = publisher.get_num_connections()
        counter = 0

        # loop until publisher connected
        while connection == 0 and not rospy.is_shutdown():
            # update parameters
            connection = publisher.get_num_connections()
            counter += 1

            # log info
            rospy.loginfo(f" Waiting for {topic}: {counter}")

            # sleep for few seconds
            self.loop_rate.sleep()

        rospy.loginfo(f"Publisher connected : {topic}")

    def move_all_joints(self, theta1, theta2, theta3):
        # define joint message
        message_joint1 = Float64()
        message_joint1.data = theta1

        message_joint2 = Float64()
        message_joint2.data = theta2

        message_joint3 = Float64()
        message_joint3.data = theta3

        # publish messages
        self.publisher_joint1.publish(message_joint1)
        self.publisher_joint2.publish(message_joint2)
        self.publisher_joint3.publish(message_joint3)

    def spin(self, positions):
        # loop until shutdown requested
        while not rospy.is_shutdown():
            for target in positions:
                # move joint to new position
                self.move_all_joints(target[0], target[1], target[2])

                # sleep for few seconds
                time.sleep(5)


def main(args=None):
    # initialize ros and node
    rospy.init_node("antropomorphic_arm_mover_node")
    node = AntropomorphicArmMoverNode()

    # test position
    position_array = [
        [0.7853981633974483, 0.09188093307208842, 1.0471975511965976],
        [-2.356194490192345, 2.002514169321107, 1.0471975511965976],
        [-2.356194490192345, 3.049711720517705, -1.0471975511965976],
    ]

    # execute node
    try:
        node.spin(position_array)
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

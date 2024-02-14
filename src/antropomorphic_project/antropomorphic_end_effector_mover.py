#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Vector3
from planar_3dof_control.msg import EndEffector

# local import
from antropomorphic_project.ik_antropomorphic_arm import AntropomorphicInverseKinematic
from antropomorphic_project.utils.move_joints import AntropomorphicArmMoverNode
from antropomorphic_project.utils.rviz_marker import RvizMarkerBasicsNode


class AntropomorphicEndEffectorMoverNode(object):
    def __init__(self):
        self.loop_rate = rospy.Rate(25)  # 25hz

        # member variables
        self.mover = AntropomorphicArmMoverNode()
        self.marker = RvizMarkerBasicsNode()
        self.model = AntropomorphicInverseKinematic(0.0, 1.0, 1.0)

        # ros object
        self.subscriber_move = rospy.Subscriber(
            "/ee_pose_commands", EndEffector, self.subscriber_move_callback
        )
        self.subscriber_real = rospy.Subscriber(
            "/end_effector_real_pose", Vector3, self.subscriber_real_callback
        )

        # test connection with antropomorphic arm
        self.check_connection()

    def check_connection(self):
        # connect all topic before spinning
        self.check_topic_connection(self.subscriber_move, "ee_pose_commands")
        self.check_topic_connection(
            self.subscriber_real, "joiend_effector_real_posent2"
        )

        # node acknowledgement
        rospy.loginfo("Antropomorphic end effector mover node ready for movement!")

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

        rospy.loginfo(f"Subscriber connected : {topic}")

    def subscriber_real_callback(self, msg):
        # reading current position from topic
        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def subscriber_move_callback(self, msg):
        # reading target position from topic
        self.px = msg.ee_xy_theta.x
        self.py = msg.ee_xy_theta.y
        self.pz = msg.ee_xy_theta.z

        # read current policy
        self.policy = msg.elbow_policy.data

    def spin(self, marker_index=0):
        # loop until shutdown requested
        while not rospy.is_shutdown():
            # read current policy
            sign1, sign2 = self.policy.split("-")

            # customize into bool condition
            sign1 = True if sign1 == "plus" else False
            sign2 = True if sign2 == "plus" else False

            # find solutions
            is_possible, theta1, theta2, theta3 = self.model.calculate_solution(
                self.px, self.py, self.pz, sign1, sign2
            )

            # is solution found move to target position
            if is_possible:
                self.mover.move_all_joints(theta1, theta2, theta3)
                self.marker.publish_point(self.px, self.py, self.pz, marker_index)
            else:
                rospy.logerr("Unable to find solution, robot cant reach target pose!")

            # update marker index
            marker_index += 1

            # sleep for few seconds
            self.loop_rate.sleep()


def main(args=None):
    # initialize ros and node
    rospy.init_node("elipsoidal_end_effector_motion_node")
    node = AntropomorphicEndEffectorMoverNode()

    # execute node
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

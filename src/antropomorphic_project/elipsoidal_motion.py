#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Vector3
from planar_3dof_control.msg import EndEffector

# constant value
PI = math.pi


class ElipsoidalMotionrNode(object):
    def __init__(self):
        self.publisher = rospy.Publisher("/ee_pose_commands", EndEffector, queue_size=1)
        self.loop_rate = rospy.Rate(20.0)

        # elipse a, b parameters
        self.a = 1.5
        self.b = 1.0
        self.min = 0.8
        self.max = 1.7
        self.delta = 0.1

        # elipsoidal motion height
        self.z = 0.0
        self.z_max = 0.5
        self.z_min = -0.5

        # initial parameters
        self.theta = 0.0
        self.increment = True

    def generate_elipse_points(self, delta=0.02):
        # generate new points
        x = self.a * math.cos(self.theta)
        y = self.b * math.sin(self.theta)

        # update theta
        self.theta += delta

        # check if full circle completed
        if self.theta >= 2 * PI:
            # reset theta value and update z value
            self.theta = 0.0
            if self.increment:
                self.z += self.delta
                self.a -= self.delta
                self.b -= self.delta

                # update elipse a, b value
                if self.a < self.min:
                    self.a = self.min
                if self.b < self.min:
                    self.b = self.min
            else:
                self.z -= self.delta
                self.a += self.delta
                self.b += self.delta

                # update elipse a, b value
                if self.a > self.max:
                    self.a = self.max
                if self.b > self.max:
                    self.b = self.max

            # change direction if min/max value reached
            if self.z >= self.z_max:
                self.increment = False

            if self.z <= self.z_min:
                self.increment = True
        
        # return x, y, z elipsoidal motion
        return x, y, self.z

    def spin(self):
        # node acknowledgement
        rospy.loginfo("publishing to topic /ee_pose_commands")

        # policy by default, if solution not possible
        # the arm movier will try the other options
        elbow_policy = "plus-minus"

        message = EndEffector()
        end_effector_pose = Vector3()

        # loop until shutdown requested
        while not rospy.is_shutdown():
            # calculate new postion
            x, y, z = self.generate_elipse_points()

            end_effector_pose.x = x
            end_effector_pose.y = y
            end_effector_pose.z = z

            message.ee_xy_theta = end_effector_pose
            message.elbow_policy.data = elbow_policy

            # log info
            rospy.logdebug_throttle(1, f"{elbow_policy} [{x} {y} {z}]")

            # publish and sleep
            self.publisher.publish(message)
            self.loop_rate.sleep()


def main(args=None):
    # initialize ros and node
    rospy.init_node("elipsoidal_motion_node")
    node = ElipsoidalMotionrNode()

    # execute node
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

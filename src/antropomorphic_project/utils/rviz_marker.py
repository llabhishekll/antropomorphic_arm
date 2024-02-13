#!/usr/bin/env python3
import math
import random
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

# constant value
PI = math.pi


class RvizMarkerBasicsNode(object):
    def __init__(self):
        self.publisher = rospy.Publisher("/ee_position", Marker, queue_size=1)
        self.loop_rate = rospy.Rate(1)  # 1hz

        # initialize marker
        self.init_marker()

    def init_marker(self):
        # define visual marker
        self.marker = Marker()
        self.marker.header.frame_id = "frame_0"
        self.marker.header.stamp = rospy.get_rostime()
        self.marker.ns = ""
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.action = Marker.ADD

        self.marker.pose.position = Point()
        self.marker.pose.orientation.x = 0
        self.marker.pose.orientation.y = 0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.scale.z = 0.1

        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 0.8  # no transparent

        # keep marker forever, then 0
        # otherwise seconds before desapearing
        self.marker.lifetime = rospy.Duration(0)

    def update_pose(self, x, y, z, index):
        # define new marker position
        position = Point()
        position.x = x
        position.y = y
        position.z = z

        self.marker.id = index
        self.marker.pose.position = position

        # random colour each time
        self.marker.color.r = random.random()
        self.marker.color.g = random.random()
        self.marker.color.b = random.random()

    def publish_point(self, x, y, z, index):
        # update mark value
        self.update_pose(x, y, z, index)

        # publish new mark point
        self.publisher.publish(self.marker)

    def spin(self, theta=0, z=0.0, index=0, increment=True):
        # loop until shutdown requested
        while not rospy.is_shutdown():
            # publish new marker
            x = math.cos(theta)
            y = math.sin(theta)
            self.publish_point(x, y, z, index)

            # update theta and index
            theta += 0.2
            index += 1

            # check if full circle completed
            if theta >= 2 * PI:
                # reset theta value and update z value
                theta = 0.0
                z = z + 0.1 if increment else z - 0.1

                # change direction if min/max value reached
                increment = not increment if (z <= -0.5) or (z >= 0.5) else increment

            # sleep for few seconds
            self.loop_rate.sleep()


def main(args=None):
    # initialize ros and node
    rospy.init_node("rviz_marker_basic_node")
    node = RvizMarkerBasicsNode()

    # execute node
    try:
        node.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()

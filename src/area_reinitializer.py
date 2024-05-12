#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from std_msgs.msg import Bool

# Define the areas as polygons
areas = [
    # Area 1
    [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)],

    # Area 2
    [(2.0, 2.0), (2.0, 4.0), (4.0, 4.0), (4.0, 2.0)],

    # Area 3
    [(0.0, -3.0), (0.0, -5.0), (2.0, -5.0), (2.0, -3.0)]
]

class AreaReinitializerNode:
    def __init__(self):
        rospy.init_node('area_reinitializer_node')

        self.current_pose = None
        self.inside_areas = [False] * len(areas)  # Track if inside any of the areas
        self.reinitialization_done = False  # Track if reconfiguration has been done
        self.enable_reconfiguration = True  # Track enable/disable status

        rospy.loginfo("Area reinitializer node started")

        # Subscribe to robot's pose
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)

        # Subscribe to reconfiguration enable/disable topic
        rospy.Subscriber('/area_reconf_enable', Bool, self.enable_callback)

        # Publish initial pose
        self.initialpose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        if not self.enable_reconfiguration:
            return
            # if self.reinitialization_done:
            #     self.publish_initial_pose()
            #     self.reinitialization_done = False
        else:
            for i, area in enumerate(areas):
                if self.check_is_inside_area(self.current_pose.pose.pose.position, area):
                    self.inside_areas[i] = True
                else:
                    self.inside_areas[i] = False

            if any(self.inside_areas):
                if not self.reinitialization_done:
                    rospy.loginfo("Robot is in the reinitialization area.")
                    self.publish_initial_pose()
                    self.reinitialization_done = True
            else:
                if self.reinitialization_done:
                    rospy.loginfo("Robot is out of the reinitialization area.")
                    self.reinitialization_done = False

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data  # Update enable/disable status

    def publish_initial_pose(self):
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = rospy.Time.now()
        initial_pose_msg.header.frame_id = "map"  # Adjust frame_id if necessary
        initial_pose_msg.pose.pose.position = self.current_pose.pose.pose.position
        initial_pose_msg.pose.pose.orientation = self.current_pose.pose.pose.orientation
        initial_pose_msg.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787]
        self.initialpose_pub.publish(initial_pose_msg)
        rospy.loginfo("Pose Reinitialized")

    def check_is_inside_area(self, position, vertices):
        # Check if the given position is inside the area defined by vertices
        x, y = position.x, position.y
        inside = self.point_inside_polygon(x, y, vertices)
        return inside

    def point_inside_polygon(self, x, y, vertices):
        # Check if the given point is inside the polygon defined by vertices
        n = len(vertices)
        inside = False
        p1x, p1y = vertices[0]
        for i in range(n + 1):
            p2x, p2y = vertices[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

if __name__ == '__main__':
    try:
        node = AreaReinitializerNode()
        rospy.spin()  # Process incoming messages

    except rospy.ROSInterruptException:
        pass

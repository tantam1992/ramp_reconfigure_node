#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool

# List of ramp areas
ramp_areas = [
    # lift ramp
    [(-8.5, 10.7), (-4.1, 10.7), (-4.1, 1.4), (-8.5, 1.4)],
    # outside ramp
    [(-15.38, 56.38), (-16.33, 56.27), (-13.59, 59.29), (-14.84, 59.20)]
]

class RampReconfigureNode:
    def __init__(self):
        rospy.init_node('ramp_reconfigure_node')

        self.current_pose = None
        self.inside = False
        self.reconfiguration_done = False  # Track if reconfiguration has been done
        self.enable_reconfiguration = True  # Track enable/disable status

        rospy.loginfo("ramp reconfigure node started")

        # Subscribe to robot's pose
        rospy.Subscriber('robot_pose', Pose, self.pose_callback)

        # Subscribe to reconfiguration enable/disable topic
        rospy.Subscriber('/rampreconf_enable', Bool, self.enable_callback)

        # Dynamic Reconfigure client
        self.reconfigure_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        # rospy.loginfo("The current pose is : \n {}".format(pose_msg))
        # Implement logic to determine if the robot is inside the ramp area
        if not self.enable_reconfiguration:  # Check if reconfiguration is enabled
            if self.reconfiguration_done:
                self.reconfigure_max_vel(0.4)
                self.reconfigure_min_vel(-0.3)
                self.reconfiguration_done = False  # Reset reconfiguration status
        else:
            if self.check_is_inside_any_ramp_area(pose_msg.position):
                if not self.reconfiguration_done:  # Perform reconfiguration only once
                    rospy.loginfo("Robot is inside a ramp area.")
                    self.reconfigure_max_vel(0.3)  # Adjust the max_vel_x parameter
                    self.reconfigure_min_vel(-0.15)
                    self.reconfiguration_done = True  # Set reconfiguration status
            else:
                if self.reconfiguration_done:
                    rospy.loginfo("Robot is outside ramp areas.")
                    self.reconfigure_max_vel(0.4)
                    self.reconfigure_min_vel(-0.3)
                    self.reconfiguration_done = False  # Reset reconfiguration status

    def enable_callback(self, enable_msg):
        self.enable_reconfiguration = enable_msg.data  # Update enable/disable status

    def reconfigure_max_vel(self, new_max_vel):
        rospy.loginfo("Reconfiguring max_vel_x to: {}".format(new_max_vel))
        params = {'max_vel_x': new_max_vel}
        self.reconfigure_client.update_configuration(params)

    def reconfigure_min_vel(self, new_min_vel):
        rospy.loginfo("Reconfiguring min_vel_x to: {}".format(new_min_vel))
        params = {'min_vel_x': new_min_vel}
        self.reconfigure_client.update_configuration(params)

    def check_is_inside_any_ramp_area(self, position):
        # Check if the given position is inside any of the ramp areas
        for ramp_area_polygon in ramp_areas:
            if self.point_inside_polygon(position.x, position.y, ramp_area_polygon):
                return True
        return False

    def point_inside_polygon(self, x, y, vertices):
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
        node = RampReconfigureNode()
        rospy.spin()  # Process incoming messages
    except rospy.ROSInterruptException:
        pass

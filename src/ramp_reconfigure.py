#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client
from geometry_msgs.msg import Pose

ramp_area_polygon = [(0.0, 0.0), (0.0, 3.0), (3.0, 3.0), (3.0, 0.0)]

class RampReconfigureNode:
    def __init__(self):
        rospy.init_node('ramp_reconfigure_node')

        self.current_pose = None
        self.inside = False
        rospy.loginfo("ramp reconfigure node started")

        # Subscribe to robot's pose
        rospy.Subscriber('robot_pose', Pose, self.pose_callback)

        # Dynamic Reconfigure client
        self.reconfigure_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')

    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg
        rospy.loginfo("The current pose is : \n {}".format(pose_msg))
        #Implement logic to determine if the robot is inside the ramp area
        if self.check_is_inside_ramp_area (pose_msg.position):
            self.reconfigure_max_vel(0.15)  # Adjust the max_vel_x parameter
        else:
            self.reconfigure_max_vel(0.5)

    def reconfigure_max_vel(self, new_max_vel):
        rospy.loginfo("Reconfiguring max_vel_x to: {}".format(new_max_vel))
        params = {'max_vel_x': new_max_vel}
        self.reconfigure_client.update_configuration(params)

    def check_is_inside_ramp_area(self, position):
        # Check if the given position is inside the ramp area polygon
        x, y = position.x, position.y
        inside = self.point_inside_polygon(x, y, ramp_area_polygon)
        rospy.loginfo("Inside ramp area: {}".format(inside))
        return inside

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
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

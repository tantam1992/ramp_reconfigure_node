#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client

class SimTimeReconfigureNode:
    def __init__(self):
        rospy.init_node('sim_time_reconfigure_node')
        rospy.loginfo("sim time reconfigure node started")

        # Dynamic Reconfigure client
        self.reconfigure_client = dynamic_reconfigure.client.Client('move_base/DWAPlannerROS')
        # reconfigure sim time to 1.0
        self.reconfigure_sim_time(1.0)
        # reconfigure sim time to 3.0
        # self.reconfigure_sim_time(3.0)

    # function to reconfigure sim time
    def reconfigure_sim_time(self, new_sim_time):
        rospy.loginfo("Reconfiguring sim_time to: {}".format(new_sim_time))
        params = {'sim_time': new_sim_time}
        self.reconfigure_client.update_configuration(params)

if __name__ == '__main__':
    try:
        node = SimTimeReconfigureNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

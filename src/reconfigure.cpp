#include <ros/ros.h>
#include <ros/master.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Config.h>
 
int main(int argc, char **argv) {
    ros::init(argc, argv, "param_client_all_nodes");  // 初始化ROS节点
    ros::NodeHandle nh;
 
    // Create a dynamic reconfigure client
    ros::ServiceClient client = nh.serviceClient<dynamic_reconfigure::Reconfigure>("/move_base/DWAPlannerROS/set_parameters");
    dynamic_reconfigure::Reconfigure srv;
    dynamic_reconfigure::Config config;

    // Request the current parameter values
    double previous_sim_time;
    ros::param::get("/move_base/DWAPlannerROS/sim_time", previous_sim_time);
    
    // Set the new sim_time
    dynamic_reconfigure::DoubleParameter double_param;
    double_param.name = "sim_time";
    double_param.value = 1.0;
    config.doubles.push_back(double_param);

    srv.request.config = config;

    if (client.call(srv)) {
        ROS_INFO_STREAM("Set sim_time to " << double_param.value << " succeeded.");
    } else {
        ROS_WARN_STREAM("Failed to set sim_time ");
    }
    return 0;
}
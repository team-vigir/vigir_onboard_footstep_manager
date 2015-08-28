#include "ros/ros.h"
#include <nodelet/loader.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vigir_onboard_footstep_manager");

    nodelet::Loader manager(true);
    nodelet::M_string remappings;
    nodelet::V_string my_argv;

    manager.load(ros::this_node::getName(), "vigir_onboard_footstep_manager/FootstepManager", remappings, my_argv);

    ros::spin();
    return 0;
}

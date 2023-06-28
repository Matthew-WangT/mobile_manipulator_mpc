#include "interactive_marker/interactive_marker.h"


int main(int argc, char *argv[])
{
    const std::string robotName = "mobile_manipulator";
    ros::init(argc, argv, robotName + "_target");
    ros::NodeHandle nodeHandle;

    InteractiveMarker targetPoseCommand(nodeHandle, robotName);
    targetPoseCommand.publishInteractiveMarker();

    // Successful exit
    return 0;
}
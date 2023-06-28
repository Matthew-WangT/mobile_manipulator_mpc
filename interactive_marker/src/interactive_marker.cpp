#include <geometry_msgs/Pose.h>
#include "interactive_marker/interactive_marker.h"


InteractiveMarker::InteractiveMarker(ros::NodeHandle &nodeHandle, const std::string &topicPrefix)
    : server_("simple_marker")
{
    targetPublisher_ = nodeHandle.advertise<geometry_msgs::Pose>(topicPrefix + "_mpc_target", 1, false);
    // create an interactive marker for our server
    menuHandler_.insert("Send target pose", boost::bind(&InteractiveMarker::processFeedback, this, _1));

    // create an interactive marker for our server
    auto interactiveMarker = createInteractiveMarker();

    // add the interactive marker to our collection &
    // tell the server to call processFeedback() when feedback arrives for it
    server_.insert(interactiveMarker); //, boost::bind(&TargetTrajectoriesInteractiveMarker::processFeedback, this, _1));
    menuHandler_.apply(server_, interactiveMarker.name);

    // 'commit' changes and send to all clients
    server_.applyChanges();
}

/******************************************************************************************************/
void InteractiveMarker::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    // Desired state trajectory
    // const Eigen::Vector3d position(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
    // const Eigen::Quaterniond orientation(feedback->pose.orientation.w, feedback->pose.orientation.x, feedback->pose.orientation.y,
                                        //  feedback->pose.orientation.z);
    targetPublisher_.publish(feedback->pose);
}

/******************************************************************************************************/
visualization_msgs::InteractiveMarker InteractiveMarker::createInteractiveMarker() const
{
    visualization_msgs::InteractiveMarker interactiveMarker;
    interactiveMarker.header.frame_id = "odom";
    interactiveMarker.header.stamp = ros::Time::now();
    interactiveMarker.name = "Goal";
    interactiveMarker.scale = 0.2;
    interactiveMarker.description = "Right click to send command";
    interactiveMarker.pose.position.z = 1.0;

    // create a grey box marker
    const auto boxMarker = []()
    {
        visualization_msgs::Marker marker;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.5;
        return marker;
    }();

    // create a non-interactive control which contains the box
    visualization_msgs::InteractiveMarkerControl boxControl;
    boxControl.always_visible = 1;
    boxControl.markers.push_back(boxMarker);
    boxControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;

    // add the control to the interactive marker
    interactiveMarker.controls.push_back(boxControl);

    // create a control which will move the box
    // this control does not contain any markers,
    // which will cause RViz to insert two arrows
    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    interactiveMarker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    interactiveMarker.controls.push_back(control);

    return interactiveMarker;
}


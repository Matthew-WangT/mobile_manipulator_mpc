#pragma once
#include <memory>
#include <mutex>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <ros/ros.h>

class InteractiveMarker final{
    public:
    InteractiveMarker(ros::NodeHandle &nodeHandle, const std::string &topicPrefix);
    /**
     * Spins ROS to update the interactive markers.
     */
    void publishInteractiveMarker() { ros::spin(); }

private:
    visualization_msgs::InteractiveMarker createInteractiveMarker() const;
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    interactive_markers::MenuHandler menuHandler_;
    interactive_markers::InteractiveMarkerServer server_;

    // std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;
    ros::Publisher targetPublisher_;
    // nodeHandle.advertise<ocs2_msgs::mpc_target_trajectories>(topicPrefix + "_mpc_target", 1, false);
};
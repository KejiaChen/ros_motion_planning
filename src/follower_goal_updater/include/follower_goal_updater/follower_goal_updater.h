#ifndef FOLLOWER_GOAL_UPDATER_H
#define FOLLOWER_GOAL_UPDATER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class FollowerGoalUpdater {
public:
    FollowerGoalUpdater();

    /**
     * @brief Callback function to handle the leader's pose message and publish the follower's goal.
     * @param msg The leader's pose message.
     */
    void leaderPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    /**
     * @brief Converts a tf2::Transform to geometry_msgs::Pose.
     * @param transform The tf2::Transform to convert.
     * @return The corresponding geometry_msgs::Pose.
     */
    geometry_msgs::Pose transformToPose(const tf2::Transform& transform);

private:
    ros::NodeHandle nh_;                   ///< General ROS node handle
    ros::Subscriber leader_pose_sub_;      ///< Subscriber for the leader's pose
    ros::Publisher goal_pub_;              ///< Publisher for the follower's goal

    std::string leader_pose_topic_ = "/robot1/amcl_pose"; ///< Topic for the leader's pose
    std::string goal_pub_topic_ = "/robot2/move_base_simple/goal"; ///< Topic for the follower's goal
    double offset_ = -1.0;                 ///< Offset for the follower's position
};

#endif // FOLLOWER_GOAL_UPDATER_H

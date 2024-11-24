#include <follower_goal_updater/follower_goal_updater.h>

// Constructor
FollowerGoalUpdater::FollowerGoalUpdater() {
    ros::NodeHandle private_nh("~"); // Private node handle for retrieving parameters
    private_nh.param("leader_pose_topic", leader_pose_topic_, leader_pose_topic_);
    private_nh.param("goal_pub_topic", goal_pub_topic_, goal_pub_topic_);
    private_nh.param("offset", offset_, offset_);

    // Subscribe and advertise using configured topics
    leader_pose_sub_ = nh_.subscribe(leader_pose_topic_, 1, &FollowerGoalUpdater::leaderPoseCallback, this);
    goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(goal_pub_topic_, 1);
}

// Leader pose callback
void FollowerGoalUpdater::leaderPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    ROS_INFO("Received leader pose: x = %.2f, y = %.2f", msg->pose.pose.position.x, msg->pose.pose.position.y);

    geometry_msgs::PoseStamped follower_goal;
    follower_goal.header = msg->header;

    // Compute follower goal based on leader pose
    tf2::Transform leader_transform;
    tf2::fromMsg(msg->pose.pose, leader_transform);

    tf2::Transform offset_transform;
    offset_transform.setOrigin(tf2::Vector3(offset_, 0.0, 0.0)); // Desired offset in leader's frame
    offset_transform.setRotation(tf2::Quaternion::getIdentity());

    tf2::Transform follower_transform = leader_transform * offset_transform;
    follower_goal.pose = transformToPose(follower_transform);

    // Publish the updated goal for the follower
    goal_pub_.publish(follower_goal);
    ROS_INFO("Published follower goal position (x, y): (%.2f, %.2f)", follower_goal.pose.position.x, follower_goal.pose.position.y);
}

// Transform to Pose conversion
geometry_msgs::Pose FollowerGoalUpdater::transformToPose(const tf2::Transform& transform) {
    geometry_msgs::Pose pose;

    // Set the position
    pose.position.x = transform.getOrigin().x();
    pose.position.y = transform.getOrigin().y();
    pose.position.z = transform.getOrigin().z();

    // Set the orientation
    tf2::Quaternion quat = transform.getRotation();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();

    return pose;
}

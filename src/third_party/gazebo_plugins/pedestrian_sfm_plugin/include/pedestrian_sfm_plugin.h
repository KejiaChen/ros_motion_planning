/***********************************************************
 *
 * @file: pedestrian_sfm_plugin.cpp
 * @breif: Gazebo plugin for pedestrians using social force model
 * @author: Yang Haodong
 * @update: 2023-03-15
 * @version: 1.1
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PEDESTRIANSFM_GAZEBO_PLUGIN_H
#define PEDESTRIANSFM_GAZEBO_PLUGIN_H

// C++
#include <string>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>

// Social Force Model
#include <lightsfm/sfm.hpp>

namespace gazebo
{
class GZ_PLUGIN_VISIBLE PedestrianSFMPlugin : public ModelPlugin
{

public:
  /**
   * @brief Construct a gazebo plugin
   */
  PedestrianSFMPlugin();

  /**
   * @brief Load the actor plugin.
   * @param _model  Pointer to the parent model.
   * @param _sdf    Pointer to the plugin's SDF elements.
   */
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /**
   * @brief Initialize the social force model.
   */
  virtual void Reset();


private:
  /**
   * @brief Function that is called every update cycle.
   * @param _info Timing information.
   */
  void OnUpdate(const common::UpdateInfo &_info);

  /**
   * @brief Helper function to detect the closest obstacles.
   */
  void handleObstacles();

  /**
   * @brief Helper function to detect the nearby pedestrians (other actors).
   */
  void handlePedestrians();


private:
  // Gazebo ROS node
  std::unique_ptr<ros::NodeHandle> node_;
  // topic publisher
  ros::Publisher pose_pub_;
  // this actor as a SFM agent
  sfm::Agent sfm_actor_;
  // names of the other models in my walking group.
  std::vector<std::string> group_names_;
  // vector of pedestrians detected.
  std::vector<sfm::Agent> other_actors_;
  // Maximum distance to detect nearby pedestrians.
  double people_dist_;
  // Pointer to the parent actor.
  physics::ActorPtr actor_;
  // Pointer to the world, for convenience.
  physics::WorldPtr world_;
  // Pointer to the sdf element.
  sdf::ElementPtr sdf_;
  // Velocity of the actor
  ignition::math::Vector3d velocity_;
  // List of connections
  std::vector<event::ConnectionPtr> connections_;
  // Time scaling factor. Used to coordinate translational motion with the actor's walking animation.
  double animation_factor_ = 1.0;
  // Time of the last update.
  common::Time last_update_;
  // List of models to ignore. Used for vector field
  std::vector<std::string> ignore_models_;
  // Custom trajectory info.
  physics::TrajectoryInfoPtr trajectory_info_;
};
}
#endif

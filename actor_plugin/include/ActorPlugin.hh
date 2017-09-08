/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GAZEBO_PLUGINS_ACTORPLUGIN_HH_
#define GAZEBO_PLUGINS_ACTORPLUGIN_HH_

#include <string>
#include <vector>
#include <memory>
#include <thread>
#include <actor_services/SetPose.h>
#include <actor_services/GetVel.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_broadcaster.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE ActorPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: 
      ActorPlugin();

      /// \brief Load the actor plugin.
      /// \param[in] _model Pointer to the parent model.
      /// \param[in] _sdf Pointer to the plugin's SDF elements.
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

      /// \brief Function that is called every update cycle.
      /// \param[in] _info Timing information
    private: 
      void OnUpdate(const common::UpdateInfo &_info);

      /// \brief A node use for ROS transport
      // std::shared_ptr<ros::NodeHandle> rosNode;
      ros::NodeHandlePtr rosNode;

      ros::ServiceServer SetPoseService;

      ros::ServiceServer SetTargetService;

      ros::ServiceServer GetVelService;

      // ros::ServiceClient GetVelClient;

      ignition::math::Vector3d CallActorVelClient(std::string) const;

      /// \brief Helper function to choose a new target location
      void ChooseNewTarget();

      /// \brief Helper function to avoid obstacles. This implements a very
      /// simple vector-field algorithm.
      /// \param[in] _pos Direction vector that should be adjusted according
      /// to nearby obstacles.
      void HandleObstacles(ignition::math::Vector3d &_pos);

      //double socialForceFactor;
      //double desiredForceFactor;
      //double obstacleForceFactor;

      /// Compute the social force.
      ignition::math::Vector3d SocialForce(ignition::math::Pose3d &_pose, ignition::math::Vector3d _velocity) const;

      /// Compute the obstacle force.
      ignition::math::Vector3d ObstacleForce(ignition::math::Pose3d &_pose) const;

      /// \brief Pointer to the parent actor.
      physics::ActorPtr actor;

      /// \brief Pointer to the world, for convenience.
      physics::WorldPtr world;

      /// \brief Velocity of the actor
      ignition::math::Vector3d velocity;

      /// \brief Angular Velocity of the yaw axis
      double yaw_vel;

      /// \brief Max velocity of the actor
      double vMax;

      /// \brief The direction the actor will dodge in, will dodge right if true or by default
      //bool dodgingRight;

      /// \brief List of connections
      std::vector<event::ConnectionPtr> connections;

      /// \brief Current target location
      ignition::math::Vector3d target;

      /// \brief Start location
      ignition::math::Vector3d start_location;

      /// \brief Target location weight (used for vector field)
      //double targetWeight = 1.0;

      /// \brief Obstacle weight (used for vector field)
      //double obstacleWeight = 1.0;

      /// \brief Time scaling factor. Used to coordinate translational motion
      /// with the actor's walking animation.
      //double animationFactor = 1.0;

      /// \brief Time of the last update.
      common::Time lastUpdate;

      /// \brief List of models to ignore. Used for vector field
      std::vector<std::string> ignoreModels;

      /// \brief Custom trajectory info.
      physics::TrajectoryInfoPtr trajectoryInfo;


      // ros::Publisher PosePublisher;
      ros::Publisher VelPublisher;
      bool SetPoseCallback(actor_services::SetPose::Request&,
          actor_services::SetPose::Response&);

      bool SetTargetCallback(actor_services::SetPose::Request&,
          actor_services::SetPose::Response&);

      bool GetVelCallback(actor_services::GetVel::Request&,
          actor_services::GetVel::Response&);

      void CallPublisher(
          ignition::math::Vector3d, 
          ignition::math::Vector3d, 
          ignition::math::Vector3d, 
          double);

      /// \brief A ROS callbackqueue that helps process messages
      ros::CallbackQueue rosQueue;

      /// \brief A thread the keeps running the rosQueue
      std::thread rosQueueThread;

      void QueueThread();

      // param list
      void get_ros_parameters(const ros::NodeHandlePtr);

      double socialForceFactor;
      double desiredForceFactor;
      double obstacleForceFactor;
      double maxSpeed;
      double maxAngleUpdate;
      bool dodgingRight;
      bool tb3_as_actor;
      std::string tb3_name;
      double animationFactor;
      double sf_lambdaImportance;
      double sf_gamma;
      double sf_n;
      double sf_n_prime;
      double neighborRange;
      double depth_fov;
      double fixed_actor_height; 
      double sf_distance_th;
      double vel_param;
  };
}
#endif

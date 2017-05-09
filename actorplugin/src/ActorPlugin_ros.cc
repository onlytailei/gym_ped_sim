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

#include <functional>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/math/gzmath.hh>
#include "ActorPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(ActorPlugin)

#define WALKING_ANIMATION "walking"

  /////////////////////////////////////////////////
ActorPlugin::ActorPlugin()
{
}

/////////////////////////////////////////////////
void ActorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->actor = boost::dynamic_pointer_cast<physics::Actor>(_model);
  this->world = this->actor->GetWorld();
  this->start_location = this->actor->WorldPose().Pos();

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
        std::bind(&ActorPlugin::OnUpdate, this, std::placeholders::_1)));

  this->velocity = 0.8;

  // Read in the first target location
  if (_sdf->HasElement("target"))
    this->target = _sdf->Get<ignition::math::Vector3d>("target");
  else
    this->target = ignition::math::Vector3d(0, -5, 1.2138);

  // Read in the target weight
  if (_sdf->HasElement("target_weight"))
    this->targetWeight = _sdf->Get<double>("target_weight");
  else
    this->targetWeight = 1.15;

  // Read in the obstacle weight
  if (_sdf->HasElement("obstacle_weight"))
    this->obstacleWeight = _sdf->Get<double>("obstacle_weight");
  else
    this->obstacleWeight = 1.5;

  // Read in the animation factor (applied in the OnUpdate function).
  if (_sdf->HasElement("animation_factor"))
    this->animationFactor = _sdf->Get<double>("animation_factor");
  else
    this->animationFactor = 4.5;

  // Add our own name to models we should ignore when avoiding obstacles.
  this->ignoreModels.push_back(this->actor->GetName());

  // Read in the other obstacles to ignore
  if (_sdf->HasElement("ignore_obstacles"))
  {
    sdf::ElementPtr modelElem =
      _sdf->GetElement("ignore_obstacles")->GetElement("model");
    while (modelElem)
    {
      this->ignoreModels.push_back(modelElem->Get< std::string >());
      modelElem = modelElem->GetNextElement("model");
    }
  }

  auto skelAnims = this->actor->SkeletonAnimations();
  if (skelAnims.find(WALKING_ANIMATION) == skelAnims.end())
  {
    gzerr << "Skeleton animation " << WALKING_ANIMATION << " not found.\n";
  }
  else
  {
    // Create custom trajectory
    this->trajectoryInfo.reset(new physics::TrajectoryInfo());
    this->trajectoryInfo->type = WALKING_ANIMATION;
    this->trajectoryInfo->duration = 1.0;

    this->actor->SetCustomTrajectory(this->trajectoryInfo);
  }

  // Initialize ros, if it has not already bee initialized.
  if (!ros::isInitialized())
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "gazebo_client",
        ros::init_options::NoSigintHandler);
  }

  // Create our ROS node. This acts in a similar manner to
  // the Gazebo node
  this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
  
  // Register the callbackqueue to the node
  this->rosNode->setCallbackQueue(&this->rosQueue);
  
  // Create the set position service  
  this->SetPoseService = this->rosNode->advertiseService("/"+this->actor->GetName()+"/SetActorPosition",
      &ActorPlugin::SetPoseCallback, this);
  
  // Create the set target position service  
  this->SetTargetService = this->rosNode->advertiseService("/"+this->actor->GetName()+"/SetActorTarget",
      &ActorPlugin::SetTargetCallback, this);
  
  // Spin up the queue helper thread.
  this->rosQueueThread =
    std::thread(std::bind(&ActorPlugin::QueueThread, this));
}

/////////////////////////////////////////////////
// TODO deprecated
void ActorPlugin::ChooseNewTarget()
{
  ignition::math::Vector3d newTarget(this->target);
  while ((newTarget - this->target).Length() < 2.0)
  {
    newTarget.X(ignition::math::Rand::DblUniform(-3, 3.5));
    newTarget.Y(ignition::math::Rand::DblUniform(-10, 2));

    for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
    {
      double dist = (this->world->ModelByIndex(i)->WorldPose().Pos()
          - newTarget).Length();
      if (dist < 2.0)
      {
        newTarget = this->target;
        break;
      }
    }
  }
  this->target = newTarget;
}

/////////////////////////////////////////////////
// TODO  A vary naive strategy need to be update
void ActorPlugin::HandleObstacles(ignition::math::Vector3d &_pos)
{
  for (unsigned int i = 0; i < this->world->ModelCount(); ++i)
  {
    physics::ModelPtr model = this->world->ModelByIndex(i);
    if (std::find(this->ignoreModels.begin(), this->ignoreModels.end(),
          model->GetName()) == this->ignoreModels.end())
    {
      ignition::math::Vector3d offset = model->WorldPose().Pos() -
        this->actor->WorldPose().Pos();
      double modelDist = offset.Length();
      if (modelDist < 2.0)
      {
        double invModelDist = this->obstacleWeight / modelDist;
        offset.Normalize();
        offset *= invModelDist;
        _pos -= offset;
      }
    }
  }
}

/////////////////////////////////////////////////
void ActorPlugin::OnUpdate(const common::UpdateInfo &_info)
{
  // Time delta
  double dt = (_info.simTime - this->lastUpdate).Double();

  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();
  
  static tf::TransformBroadcaster br;
  tf::Transform tf_transform;
  tf_transform.setOrigin(tf::Vector3(pose.Pos().X(),pose.Pos().Y(),0));
  tf::Quaternion tf_q;
  tf_q.setRPY(rpy.X(), rpy.Y(),rpy.Z());
  tf_transform.setRotation(tf_q);
  br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "default_world", this->actor->GetName()));
  ros::spinOnce();
  
  double distance = pos.Length();

  // Choose a new target position if the actor has reached its current
  // target. It will go back to the start position defaultly.
  if (distance < 0.3)
  {
    ignition::math::Vector3d temp = this->start_location; 
    this-> start_location = this->target;
    this->target = temp;
    pos = this->target - pose.Pos();
  }

  // Normalize the direction vector, and apply the target weight
  pos = pos.Normalize() * this->targetWeight;

  // Adjust the direction vector by avoiding obstacles
  this->HandleObstacles(pos);

  // Compute the yaw orientation
  ignition::math::Angle yaw = atan2(pos.Y(), pos.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();

  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    pose.Pos() += pos * this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }

  // Make sure the actor stays within bounds
  pose.Pos().X(std::max(-10.0, std::min(10.0, pose.Pos().X())));
  pose.Pos().Y(std::max(-10.0, std::min(10.0, pose.Pos().Y())));
  pose.Pos().Z(1.02);

  // Distance traveled is used to coordinate motion with the walking
  // animation
  double distanceTraveled = (pose.Pos() -
      this->actor->WorldPose().Pos()).Length();

  this->actor->SetWorldPose(pose, false, false);
  this->actor->SetScriptTime(this->actor->ScriptTime() +
      (distanceTraveled * this->animationFactor));
  this->lastUpdate = _info.simTime;
}

// Set target position service callback. Response is the target position right now
bool ActorPlugin::SetTargetCallback(actor_services::SetPose::Request& req, actor_services::SetPose::Response& res){
  res.x = this->target.X();
  res.y = this->target.Y();
  if (req.set_flag == true)
  {
    ignition::math::Vector3d newTarget(this->target);
    newTarget.X(req.new_x);
    newTarget.Y(req.new_y);
    this->target = newTarget;
  }
  return true;
}

// Set actor position service callback. Response is the position right now
bool ActorPlugin::SetPoseCallback(actor_services::SetPose::Request& req,
             actor_services::SetPose::Response& res){
  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = pose.Pos();
  res.x = pos.X();
  res.y = pos.Y();
  if (req.set_flag == true)
  {
    const math::Vector3 new_pose(req.new_x, req.new_y, pos.Z());
    const math::Quaternion new_quaterion(pose.Rot());
    const math::Pose newWorldPose(new_pose, new_quaterion); 
    this->actor->SetWorldPose(newWorldPose);
  }
  return true;
}

/// \brief ROS helper function that processes messages
void ActorPlugin::QueueThread()
{
  // It gonna be really slow if you change it to 0
  static const double timeout = 0.01;
  while (this->rosNode->ok())
  {
    this->rosQueue.callAvailable(ros::WallDuration(timeout));
  }
}


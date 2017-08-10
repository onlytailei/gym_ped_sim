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
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include "ActorPlugin.hh"
#include <ros/console.h>
#include <iostream>
#include <string>
#include <ros/console.h> //roslogging


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

  this->velocity = 0.5;

  // Read in the social force factor
  if (_sdf->HasElement("socialForce"))
    this->socialForceFactor = _sdf->Get<double>("socialForce");
  else
    this->socialForceFactor = 0.0;

  // Read in the desired force factor
  if (_sdf->HasElement("desiredForce"))
    this->desiredForceFactor = _sdf->Get<double>("desiredForce");
  else
    this->desiredForceFactor = 0.0;

  // Read in the obstacle force factor
  if (_sdf->HasElement("obstacleForce"))
    this->obstacleForceFactor = _sdf->Get<double>("obstacleForce");
  else
    this->obstacleForceFactor = 0.0;

  // Read in the speed
  if (_sdf->HasElement("speed"))
    this->vMax = _sdf->Get<double>("speed");
  else
    // just take the average speed if not given
    this->vMax = 1.34;

  // Read in the dodge direction, right by default
  if (_sdf->HasElement("dodgingRight"))
    this->dodgingRight = _sdf->Get<bool>("dodgingRight");
  else
    this->dodgingRight = true;

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

  // Initialize ros, if it has not already been initialized.
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

  // Broadcast the model velocity and position topic
  this->VelPublisher = this->rosNode->advertise<geometry_msgs::Pose>("/"+this->actor->GetName()+"/actor_vel",1);
  this->PosePublisher = this->rosNode->advertise<geometry_msgs::Twist>("/"+this->actor->GetName()+"/actor_pose",1);

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


ignition::math::Vector3d ActorPlugin::ObstacleForce(ignition::math::Pose3d &_pose) const
{
  ignition::math::Vector3d minDiff;
  double minDistanceSquared = INFINITY;

  for(unsigned int i = 0; i < this->world->ModelCount(); i++) {
    physics::ModelPtr currentObstacle = this->world->ModelByIndex(i);

    // Calculate for everything that is not an actor
    if (currentObstacle->HasType(physics::Base::EntityType::ACTOR)) {
      continue;
    }

    double distance = currentObstacle->WorldPose().Pos().Distance(_pose.Pos());
    double distanceSquared = distance * distance;
    if (distanceSquared < minDistanceSquared) {
      minDistanceSquared = distanceSquared;
      minDiff = distance;
    }
  }
  double minDistance = sqrt(minDistanceSquared);
  double forceAmount = exp(-minDistance / 0.8);
  return forceAmount * minDiff.Normalize();
}

/////////////////////////////////////////////////
// Compute social force on the actor.
ignition::math::Vector3d ActorPlugin::SocialForce(ignition::math::Pose3d &_pose, ignition::math::Vector3d _velocity) const
{
    // define relative importance of position vs velocity vector
    // (set according to Moussaid-Helbing 2009)
    const double lambdaImportance = 2.0;

    // define speed interaction
    // (set according to Moussaid-Helbing 2009)
    const double gamma = 0.35;

    // define speed interaction
    // (set according to Moussaid-Helbing 2009)
    const double n = 2;

    // define angular interaction
    // (set according to Moussaid-Helbing 2009)
    const double n_prime = 3;

    ignition::math::Vector3d force;

    // TODO: set to a good range.
    double neighborRange = 20.0;

    // Iterate over all neighbors in range of influence.
    for(unsigned int i = 0; i < this->world->ModelCount(); i++) {
      physics::ModelPtr currentAgent = this->world->ModelByIndex(i);
      
      // Check if other actor, don't calculate social force to objects
      if (!currentAgent->HasType(physics::Base::EntityType::ACTOR)) {
        continue;
      }

      // Do not calculate social force to self.
      if (currentAgent == this->actor) {
        continue;
      }

      // Only compute for other agents in neighborhood range.
      double distance = currentAgent->WorldPose().Pos().Distance(_pose.Pos());
      if (distance > neighborRange)
      {
        continue;
      }

      // Calculate difference between both agents' positions
      ignition::math::Vector3d diff = currentAgent->WorldPose().Pos() - _pose.Pos();
      ignition::math::Vector3d diffDirection = diff.Normalize();

      // compute difference between both agents' velocity vectors
      ignition::math::Vector3d velDeff = _velocity - currentAgent->RelativeLinearVel();

      // compute interaction direction t_ij
      ignition::math::Vector3d interactionVector = lambdaImportance * velDeff + diffDirection;
      double interactionLength = interactionVector.Length();
      ignition::math::Vector3d interactionDirection = interactionVector / interactionLength;

      // compute angle theta (between interaction and position difference vector)
      double thisAngle = atan2(interactionDirection.Y(), interactionDirection.X());
      // thisAngle.Normalize();
      double otherAngle = atan2(diffDirection.Y(), diffDirection.X());
      // otherAngle.Normalize();

      double theta = otherAngle - thisAngle;

      // Get sign of theta.
      int thetaSign = (theta == 0) ? (0) : (theta / abs(theta));
      //ROS_ERROR("abs theta: %d, thetaSign: %d", abs(theta),thetaSign);
      //ROS_ERROR("std abs theta: %lf, thetaSign: %d", std::abs(theta),thetaSign);
      // compute model parameter B = gamma * ||D||
      double B = gamma * interactionLength;

      double forceVelocityAmount = -exp(-diff.Length()/B - (n_prime * B * theta) * (n_prime * B * theta));
      double forceAngleAmount = -thetaSign * exp(-diff.Length() / B - (n * B * theta) * (n * B * theta));

      ignition::math::Vector3d forceVelocity = forceVelocityAmount * interactionDirection;

      // Dodge to the direction preset
      ignition::math::Vector3d interactionDirectionNormal;
      if (this->dodgingRight) {
        interactionDirectionNormal = ignition::math::Vector3d(-interactionDirection.Y(), interactionDirection.X(), interactionDirection.Z());
      }
      else {
        interactionDirectionNormal = ignition::math::Vector3d(interactionDirection.Y(), -interactionDirection.X(), interactionDirection.Z());
      }

      ignition::math::Vector3d forceAngle = forceAngleAmount * interactionDirectionNormal;

      force += forceVelocity + forceAngle;
    }
    //ROS_ERROR("======force: %lf======", force.X());
    return force;
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

  // Position of this actor
  ignition::math::Pose3d pose = this->actor->WorldPose();

  // Direct vector to the current target
  ignition::math::Vector3d pos = this->target - pose.Pos();
  ignition::math::Vector3d rpy = pose.Rot().Euler();

  // Actually move
  pose.Pos() = pose.Pos() + this->velocity * dt;

  // Get the desired force to waypoint: "I want to go there at full speed!"
  ignition::math::Vector3d desiredForce = pos.Normalize() * this->vMax;

  ignition::math::Vector3d obstacleForce = ObstacleForce(pose);

  // Sum of all forces
  ignition::math::Vector3d a = ((this->socialForceFactor * SocialForce(pose, this->velocity)) + (this->desiredForceFactor * desiredForce)) + (this->obstacleForceFactor*obstacleForce);

  // Calculate new velocity
  this->velocity = 0.5 * this->velocity + a * dt;

  // Don't exceed max speed
  double speed = this->velocity.Length();
  ROS_ERROR("speed: %lf, max speed: %lf", speed, this->vMax);
  if (speed > this->vMax) {
    this->velocity = this->velocity.Normalize() * this->vMax;
  }

  // ros stuff
  static tf::TransformBroadcaster br;
  tf::Transform tf_transform;
  tf_transform.setOrigin(tf::Vector3(pose.Pos().X(),pose.Pos().Y(),0));
  tf::Quaternion tf_q;
  tf_q.setRPY(rpy.X(), rpy.Y(),rpy.Z());
  tf_transform.setRotation(tf_q);
  br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "default_world", this->actor->GetName()));
  ros::spinOnce();


  // Choose a new target position if the actor has reached its current
  // target. It will go back to the start position defaultly.

  double distance = pose.Pos().Distance(this->target);

  if (distance < 0.3)
  {
    ignition::math::Vector3d temp = this->start_location;
    this->start_location = this->target;
    this->target = temp;
    pos = this->target - pose.Pos();
  }


  // TODO: Handle obstacles

  // Compute the yaw orientation
  // Needs to be reworked!
  ignition::math::Angle yaw = atan2(this->velocity.Y(), this->velocity.X()) + 1.5707 - rpy.Z();
  yaw.Normalize();


  /*
  // Rotate in place, instead of jumping.
  if (std::abs(yaw.Radian()) > IGN_DTOR(10))
  {
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+
        yaw.Radian()*0.001);
  }
  else
  {
    // pose.Pos() += this->velocity * dt;
    pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());
  }
  */


  pose.Rot() = ignition::math::Quaterniond(1.5707, 0, rpy.Z()+yaw.Radian());

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

// \Set actor position service callback. Response is the position right now
bool ActorPlugin::SetPoseCallback(actor_services::SetPose::Request& req,
             actor_services::SetPose::Response& res){
  ignition::math::Pose3d pose = this->actor->WorldPose();
  ignition::math::Vector3d pos = pose.Pos();
  res.x = pos.X();
  res.y = pos.Y();
  if (req.set_flag == true)
  {
    pose.Pos().X(req.new_x);
    pose.Pos().Y(req.new_y);
    this->actor->SetWorldPose(pose);
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

void ActorPlugin::CallPublisher()
{
  ignition::math::Vector3d vel_ = this->actor->RelativeLinearVel();
  ignition::math::Vector3d pose_  = this->actor->WorldPose().Pos();
}

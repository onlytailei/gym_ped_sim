/*************************************************************************
	> File Name: ActorParam.h
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Mo 14 Aug 2017 13:44:51 CEST
 ************************************************************************/

#ifndef _ACTORPARAM_H
#define _ACTORPARAM_H

#include <ros/ros.h>
#include <string>

class ActorParam{
  private:
    const ros::NodeHandlePtr rosNodeConstPtr;
  public:
    ActorParam(ros::NodeHandlePtr);
    double socialForceFactor;
    double desiredForceFactor;
    double obstacleForceFactor;
    double maxSpeed;
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

};

#endif

/*************************************************************************
	> File Name: ActorParam.cpp
	> Author: TAI Lei
	> Mail: lei.tai@my.cityu.edu.hk
	> Created Time: Mo 14 Aug 2017 13:46:57 CEST
 ************************************************************************/

#include <iostream>
#include "ActorParam.h"
using namespace std;

ActorParam::ActorParam(ros::NodeHandlePtr rosNode_pr_):
  rosNodeConstPtr(rosNode_pr_){
    assert(rosNodeConstPtr->getParam("/SOCIAL_FORCE_FACTOR", socialForceFactor));
    assert(rosNodeConstPtr->getParam("/DESIRED_FORCE_FACTOR", desiredForceFactor));
    assert(rosNodeConstPtr->getParam("/OBSTACLE_FORCE_FACTOR", obstacleForceFactor));
    assert(rosNodeConstPtr->getParam("/MAX_SPEED", maxSpeed));
    assert(rosNodeConstPtr->getParam("/DODGING_RIGHT", dodgingRight));
    assert(rosNodeConstPtr->getParam("/TB3_AS_ACTOR", tb3_as_actor));
    assert(rosNodeConstPtr->getParam("/TB3_NAME", tb3_name));
    assert(rosNodeConstPtr->getParam("/ANIMATION_FACTOR", animationFactor));
    assert(rosNodeConstPtr->getParam("/SF_LAMBDA_IMPORTANCE", sf_lambdaImportance));
    assert(rosNodeConstPtr->getParam("/SF_GAMMA", sf_gamma));
    assert(rosNodeConstPtr->getParam("/SF_N", sf_n));
    assert(rosNodeConstPtr->getParam("/SF_N_PRIME", sf_n_prime));
    assert(rosNodeConstPtr->getParam("/NEIGHBOR_RANGE", neighborRange));
    assert(rosNodeConstPtr->getParam("/DEPTH_FOV", depth_fov));
    assert(rosNodeConstPtr->getParam("/FIXED_ACTOR_HEIGHT", fixed_actor_height));
    assert(rosNodeConstPtr->getParam("/SF_DISTANCE_TH", sf_distance_th));
}

#ifndef _COMMON_H
#define _COMMON_H
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <stdio.h>
#include <iostream>
#include "ConfigJoints.h"
#include "PositionAttitude.h"
#include "StartBaseTree.h"
#include "StartBaseTreeNode.h"
#include "BaseJoints.h"
#include "BackTreeNode.h"
struct BiJoints
{
    double x,y,base_joint;
    double joint1,joint2,joint3,joint4,joint5,joint6,joint7;
    double finger1,finger2,finger3;
};
class ArmJoints
{
public:
    ArmJoints()
    {
        joint1=0;joint2=M_PI;joint3=M_PI;joint4=0;joint5=0;joint6=0;
    }
    ArmJoints(double jo1,double jo2,double jo3,double jo4,double jo5,double jo6):joint1(jo1),joint2(jo2),joint3(jo3),joint4(jo4),joint5(jo5),joint6(jo6){}
    ArmJoints(const ArmJoints& aj)
    {
        joint1=aj.joint1;joint2=aj.joint2;joint3=aj.joint3;joint4=aj.joint4;joint5=aj.joint5;joint6=aj.joint6;
    }
    void setJoint(double jo1,double jo2,double jo3,double jo4,double jo5,double jo6)
    {
        joint1=jo1;joint2=jo2;joint3=jo3;joint4=jo4;joint5=jo5;joint6=jo6;
    }
    double joint1,joint2,joint3,joint4,joint5,joint6;

};
double rand01();
double min(double x,double y);
double max(double x,double y);

#endif

#ifndef _EXTENDJOINTS_H
#define _EXTENDJOINTS_H
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
#include "BaseJoints.h"
#define M_pi 3.141592653
class ExtendJoints:public BaseJoints
{
public:
    ExtendJoints(double ax,double ay,double atheta,vector<double> & ajoints):BaseJoints(ax,ay,atheta){
        joints_of_arm.assign(ajoints.begin(),ajoints.end());
    }
    ExtendJoints(double ax,double ay,double atheta,double aj1,double aj2,double aj3,double aj4,double aj5,double aj6,double aj7):BaseJoints(ax,ay,atheta),joint_1(aj1),joint_2(aj2),joint_3(aj3),joint_4(aj4),joint_5(aj5),joint_6(aj6),joint_7(aj7){}
    ExtendJoints():BaseJoints(){
        joint_1=0;joint_2=0;joint_3=0;joint_4=0;joint_5=0;joint_6=0;joint_7=0;joints_of_arm.assign(7,0);
    }
    double joint_1;
    double joint_2;
    double joint_3;
    double joint_4;
    double joint_5;
    double joint_6;
    double joint_7;
    vector<double> joints_of_arm;
};

#endif

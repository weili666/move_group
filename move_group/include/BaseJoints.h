#ifndef _BASEJOINTS_H
#define _BASEJOINTS_H
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
#define M_pi 3.141592653
using namespace std;
class BaseJoints
{
public:
    BaseJoints(){
        x=0;y=0;theta=0;
    }
    BaseJoints(double ax,double ay,double atheta):x(ax),y(ay),theta(atheta){}
    BaseJoints(const BaseJoints& bj)
    {
        x=bj.x;y=bj.y;theta=bj.theta;
    }
    void setBase(double ax,double ay,double atheta)
    {
        x=ax;y=ay;theta=atheta;
    }

    double x;
    double y;
    double theta;
};
#endif

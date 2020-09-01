#ifndef _CONFIGJOINTS_H
#define _CONFIGJOINTS_H
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
using namespace std;
class ConfigJoints
{
public:
    ConfigJoints(){
        joints.assign(10,0);
    }
    ConfigJoints(double j1,double j2,double j3,double j4,double j5,double j6,double j7,double j8,double j9,double j10){
        joints.push_back(j1);joints.push_back(j2);joints.push_back(j3);joints.push_back(j4);joints.push_back(j5);joints.push_back(j6);joints.push_back(j7);joints.push_back(j8);joints.push_back(j9);joints.push_back(j10);
    }
    ConfigJoints(vector<double> &jo){
        joints=jo;
    }
    ConfigJoints(const ConfigJoints &cj)
    {
        joints=cj.joints;
    }
    ConfigJoints(double jo[10]){
        int size_count=sizeof(jo)/sizeof(double);
        joints.assign(jo,jo+size_count);
    }
    ConfigJoints(BaseJoints bj,vector<double> &jo2){
        joints.push_back(bj.x);joints.push_back(bj.y);joints.push_back(bj.theta);
        joints.insert(joints.begin()+3,jo2.begin(),jo2.end());
    }
    void setJoints(double jo[10]){
        int size_count=sizeof(jo)/sizeof(double);
        joints.assign(jo,jo+size_count);
    }
    vector<double> getJoints(){
        return joints;
    }
private:
    vector<double> joints;
};
#endif

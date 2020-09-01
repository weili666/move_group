#ifndef _BASETRAJECTORY_H
#define _BASETRAJECTORY_H
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
#include "BaseJoints.h"
#include "ConfigJoints.h"
#include "StartBaseTreeNode.h"
#include "PositionAttitude.h"
#include "StartBaseTree.h"
#define M_pi 3.141592653
using namespace std;
class BaseTrajectory
{
public:
    BaseTrajectory(){}
    BaseTrajectory(const BaseJoints &in,const BaseJoints &tar,const double& ina,planning_scene_monitor::PlanningSceneMonitorPtr sc,moveit::planning_interface::MoveGroup *ro):initial(in),target(tar),init_angle_of_robot_base(ina),scene_(sc),robot_(ro){
        StartBaseTreeNode *rootnode = new StartBaseTreeNode;
        rootnode->setData(in);
        rootnode->setLevel(0);
        ctree = new StartBaseTree;
        ctree->setRoot(rootnode);
    }
    ~BaseTrajectory(){}

    vector<BaseJoints> CurveAStar();
    StartBaseTree* getTree();
    vector<StartBaseTreeNode *> getChain();
    StartBaseTree *ctree;
private:
    double init_angle_of_robot_base;
    BaseJoints initial;
    BaseJoints target;
    planning_scene_monitor::PlanningSceneMonitorPtr scene_;
    moveit::planning_interface::MoveGroup *robot_;
    vector<StartBaseTreeNode *> motion_chain;
};
#endif

#ifndef _STARTBASETREE_H
#define _STARTBASETREE_H
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
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
#include "ConfigJoints.h"
#include "StartBaseTreeNode.h"
#include "PositionAttitude.h"
#define M_pi 3.141592653
using namespace std;
class StartBaseTree
{
public:
    StartBaseTree(){
        root=new StartBaseTreeNode;
        // root->deleteFather();
        root->ptrs_of_startbasetree.push_back(root);
    }

    ~StartBaseTree(){
        delete root;
    }
    bool IsEmpty() const
    {
        return((root)?false:true);
    }
    bool Root(BaseJoints &start) const;
    StartBaseTreeNode* AddNode(const PositionAttitude& element,StartBaseTreeNode *f);
    StartBaseTreeNode * Extend( planning_scene_monitor::PlanningSceneMonitorPtr scene_, moveit::planning_interface::MoveGroup *robot_,const double &init_angle_of_robot,StartBaseTreeNode* current_base,const BaseJoints& target_base);
    StartBaseTreeNode* getRoot();
    void setRoot(StartBaseTreeNode *);
    list<StartBaseTreeNode *> open_list;
    list<StartBaseTreeNode *> close_list;

private:
    StartBaseTreeNode* root;

};
#endif

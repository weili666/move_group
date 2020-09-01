#ifndef _STARTTREE_H
#define _STARTTREE_H
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
#include"ExtendJoints.h"
#include"StartTreeNode.h"
#include"StartBaseTreeNode.h"
#include"StartBaseTree.h"
#define M_pi 3.141592653
using namespace std;
class StartTree:public StartBaseTree
{
public:
    StartTree(){
        root=new StartTreeNode;
        root->ptrs_of_starttree.push_back(root);
    }
     ~StartTree(){
        delete root;
    }
    bool IsEmpty() const
    {
        return((root)?false:true);
    }
    bool Root(ExtendJoints &start)const;
    void setRoot(StartTreeNode *);
    StartTreeNode* getRoot()const;
    StartTreeNode* Extend(planning_scene_monitor::PlanningSceneMonitorPtr scene_,moveit::planning_interface::MoveGroup * robot_,const double &init_angle_of_robot);
private:
    StartTreeNode* root;
};
#endif

#ifndef _BACKTREENODE_H
#define _BACKTREENODE_H
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
#include"PositionAttitude.h"
using namespace std;
class BackTreeNode
{
public:
    void Visit(BackTreeNode *,BackTreeNode *);
    void Order(BackTreeNode *,BackTreeNode *);

    BackTreeNode()
    { }
    ~BackTreeNode(){}
    BackTreeNode(const PositionAttitude& e)
    {
        Level=e.Level;
        data=e;
        Father=new BackTreeNode;
    }
    BackTreeNode(const PositionAttitude& e,BackTreeNode *f)
    {
        data=e;
        Father=new BackTreeNode;
        Father=f;
        Father->setChild(this);
    }

    vector<BackTreeNode *> ptrs_of_backtree;
    void setData(PositionAttitude data);
    void setLevel(int lev);
    int getLevel();
    PositionAttitude getData();
    BackTreeNode* getFather();
    BackTreeNode* getChild(int index);
    void setFather(BackTreeNode *f);
    void setChild(BackTreeNode *c);
    int getNumOfBackTreeNode();
private:
    int number_of_children;
    int Level;
    PositionAttitude data;
    vector<BackTreeNode *> Children;
    BackTreeNode *Father;
};
#endif

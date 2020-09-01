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
#include "ConfigJoints.h"
#include "PositionAttitude.h"
#include "BackTreeNode.h"
#define M_pi 3.141592653
using namespace std;
void BackTreeNode::Visit(BackTreeNode *t,BackTreeNode *root)
{
    root->ptrs_of_backtree.push_back(t);
}
void BackTreeNode::Order(BackTreeNode *t, BackTreeNode *root)
{
    if(t!=NULL)
    {
        root->number_of_children++;
        root->ptrs_of_backtree.push_back(t);
        for(vector<BackTreeNode *>::iterator iter=t->Children.begin();iter!=t->Children.end();iter++)
        {
            Order(t,root);
        }
    }
}
void BackTreeNode::setData(PositionAttitude d)
{
    data=d;
}
PositionAttitude BackTreeNode::getData()
{
    return data;
}
int BackTreeNode::getNumOfBackTreeNode()
{
    return number_of_children;
}
BackTreeNode* BackTreeNode::getFather()
{
    return Father;
}
BackTreeNode* BackTreeNode::getChild(int index)
{
    return Children[index];
}
void BackTreeNode::setFather(BackTreeNode *f)
{
    Father=f;
}
void BackTreeNode::setChild(BackTreeNode *c)
{
    Children.push_back(c);
}
void BackTreeNode::setLevel(int lev)
{
    Level=lev;
}
int BackTreeNode::getLevel()
{
    return Level;
}

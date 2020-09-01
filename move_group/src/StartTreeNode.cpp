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
#define M_pi 3.141592653
using namespace std;
void StartTreeNode::Visit(StartTreeNode *t,StartTreeNode *root)
{
    root->ptrs_of_starttree.push_back(t);
}

void StartTreeNode::Order(StartTreeNode *t, StartTreeNode *root)
{
    if(t!=NULL)
    {
        root->number_of_children++;
        root->ptrs_of_starttree.push_back(t);
        for(vector<StartTreeNode *>::iterator iter=t->Children.begin();iter!=t->Children.end();iter++)
        {
            Order(*iter,root);
        }
    }
}

void StartTreeNode::setData(ExtendJoints d)
{
    data.x=d.x;data.y=d.y;data.theta=d.theta;data.joint_1=d.joint_1;data.joint_2=d.joint_2;data.joint_3=d.joint_3;data.joint_4=d.joint_4;data.joint_5=d.joint_5;data.joint_6=d.joint_6;data.joint_7=d.joint_7;
}

void StartTreeNode::setData(double ax, double ay, double atheta, double j1, double j2, double j3, double j4, double j5, double j6, double j7)
{
    data.x=ax;data.y=ay;data.theta=atheta;data.joint_1=j1;data.joint_2=j2;data.joint_3=j3;data.joint_4=j4;data.joint_5=j5;data.joint_6=j6;data.joint_7=j7;
}

 ExtendJoints StartTreeNode::getData()
 {
     return data;
 }

 int StartTreeNode::getNumOfChildren()
 {
      number_of_children=Children.size();
      return number_of_children;
 }

 StartTreeNode* StartTreeNode::getFather()
 {
     return Father;
 }

 StartTreeNode*StartTreeNode:: getChild(int index)
 {
     return Children[index];
 }

 void StartTreeNode::setFather(StartTreeNode *f)
 {
     Father=f;
 }

 void StartTreeNode::setChild(StartTreeNode *c)
 {
     Children.push_back(c);
 }

 void StartTreeNode::setLevel(int l)
 {
     level=l;
 }

 int StartTreeNode::getLevel()
 {
     return level;
 }

 double StartTreeNode::_distance_(double x,double y,double joint_1,double joint_2,double joint_3,double joint_4,double joint_5,double joint_6,double joint_7)
 {
     double dist=sqrt((data.x-x)*(data.x-x)+(data.y-y)*(data.y-y)+(data.joint_1-joint_1)*(data.joint_1-joint_1)+(data.joint_2-joint_2)*(data.joint_2-joint_2)+(data.joint_3-joint_3)*(data.joint_3-joint_3)+(data.joint_4-joint_4)*(data.joint_4-joint_4)+(data.joint_5-joint_5)*(data.joint_5-joint_5)+(data.joint_6-joint_6)*(data.joint_6-joint_6)+(data.joint_7-joint_7)*(data.joint_7-joint_7));
     return dist;
 }


#ifndef _STARTTREENODE_H
#define _STARTTREENODE_H
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
#include "ConfigJoints.h"
#include"BaseJoints.h"
#include"ExtendJoints.h"
#include"StartBaseTreeNode.h"
#define M_pi 3.141592653
using namespace std;
class StartTreeNode:public StartBaseTreeNode
{
public:
    StartTreeNode(){}
    ~StartTreeNode(){}
    StartTreeNode(ExtendJoints etj):StartBaseTreeNode(BaseJoints(etj.x,etj.y,etj.theta)),data(etj){
        Father=new StartTreeNode;
    }
    StartTreeNode(ExtendJoints etj,StartTreeNode *f):StartBaseTreeNode(BaseJoints(etj.x,etj.y,etj.theta))
    {
        Father=new StartTreeNode;
        data=etj;
        Father=f;
    }
    void Visit(StartTreeNode *,StartTreeNode *);
    void Order(StartTreeNode *,StartTreeNode *);
    void setData(ExtendJoints data);
    void setData(double ax,double ay,double atheta,double j1,double j2,double j3,double j4,double j5,double j6,double j7);
    ExtendJoints getData();
    int getNumOfChildren();
    StartTreeNode* getFather();
    StartTreeNode* getChild(int index);
    void setFather(StartTreeNode *f);
    void setChild(StartTreeNode *c);
    vector<StartTreeNode *> ptrs_of_starttree;
    int getLevel();
    void setLevel(int l);
    double _distance_(double,double,double ,double,double,double,double,double,double);

private:
    int level;
    int number_of_children;
    ExtendJoints data;
    vector<StartTreeNode *> Children;
    StartTreeNode *Father;
};
#endif

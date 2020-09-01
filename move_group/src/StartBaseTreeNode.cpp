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
#include "PositionAttitude.h"
#include "StartBaseTreeNode.h"
#define M_pi 3.141592653
using namespace std;
void StartBaseTreeNode::Visit(StartBaseTreeNode *t,StartBaseTreeNode *root)
{
    root->ptrs_of_startbasetree.push_back(t);
}

void StartBaseTreeNode::Order(StartBaseTreeNode *t,StartBaseTreeNode *root)
{
    if(t!=NULL)
    {
        root->number_of_children++;
        root->ptrs_of_startbasetree.push_back(t);
        for(vector<StartBaseTreeNode *>::iterator iter=t->Children.begin();iter!=t->Children.end();iter++)
        {
            Order(*iter,root);
        }
    }
}

void StartBaseTreeNode::setData(BaseJoints d)
{
   data.x=d.x;data.y=d.y;data.theta=d.theta;
}
void StartBaseTreeNode::setX(double xx)
{
    data.x=xx;
}
void StartBaseTreeNode::setY(double yy)
{
    data.y=yy;
}
void StartBaseTreeNode::setTHETA(double tt)
{
    data.theta=tt;
}
void StartBaseTreeNode::setABCDEF(double aa,double bb,double cc,double dd,double ee,double ff)
{
    a=aa;
    b=bb;
    c=cc;
    d=dd;
    e=ee;
    f=ff;
}
double StartBaseTreeNode::getA()
{
    return a;
}
double StartBaseTreeNode::getB()
{
    return b;
}
double StartBaseTreeNode::getC()
{
    return c;
}
double StartBaseTreeNode::getD()
{
    return d;
}
double StartBaseTreeNode::getE()
{
    return e;
}
double StartBaseTreeNode::getF()
{
    return f;
}
BaseJoints StartBaseTreeNode::getData()
{
    return data;
}

int StartBaseTreeNode::getLevel()
{
    return level;
}

double StartBaseTreeNode::getValG()
{
    return val_g;
}

double StartBaseTreeNode::getValH()
{
    return val_h;
}

double StartBaseTreeNode::getValF()
{
    return val_f;
}

void StartBaseTreeNode::setLevel(int l)
{
    level=l;
}

void StartBaseTreeNode::setVal(double g,double h,double f)
{
    val_g=g;
    val_h=h;
    val_f=f;
}

void StartBaseTreeNode::setValG(double g)
{
    val_g=g;
}

void StartBaseTreeNode::setValH(double h)
{
    val_h=h;
}

void StartBaseTreeNode::setValF(double f)
{
    val_f=f;
}

double StartBaseTreeNode::getX()
{
    return data.x;
}

double StartBaseTreeNode::getY()
{
    return data.y;
}

double StartBaseTreeNode::getTheta()
{
    return data.theta;
}

double StartBaseTreeNode::distance_(double x,double y)
{
    double dist;
    dist=sqrt((x-data.x)*(x-data.x)+(y-data.y)*(y-data.y));
    return dist;
}




int StartBaseTreeNode::getNumOfChildren()
{
    number_of_children=Children.size();
    return number_of_children;
}

StartBaseTreeNode*  StartBaseTreeNode::getFather()
{
    return Father;
}

void StartBaseTreeNode::setFather(StartBaseTreeNode *f)
{
    Father=f;
}

StartBaseTreeNode* StartBaseTreeNode::getChild(int index)
{
    return Children[index];
}

void StartBaseTreeNode::setChild(StartBaseTreeNode *c)
{
    Children.push_back(c);
}

#ifndef _STARTBASETREENODE_H
#define _STARTBASETREENODE_H
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

#define M_pi 3.141592653
using namespace std;
class StartBaseTreeNode
{
public:
    StartBaseTreeNode(){}
    ~StartBaseTreeNode(){}
    StartBaseTreeNode(BaseJoints dt):data(dt){
        Father=new StartBaseTreeNode;
    }
    StartBaseTreeNode(BaseJoints dt,StartBaseTreeNode *f)
    {
        Father=new StartBaseTreeNode;
        data=dt;
        Father=f;
    }
    virtual void Visit(StartBaseTreeNode *,StartBaseTreeNode *);
    virtual void Order(StartBaseTreeNode *,StartBaseTreeNode *);
    virtual void setData(BaseJoints data);
    void setX(double xx);
    void setY(double yy);
    void setTHETA(double tt);
    void setABCDEF(double aa,double bb,double cc,double dd,double ee,double ff);
    double getX();
    double getY();
    double getTheta();
    BaseJoints getData();
    double getA();
    double getB();
    double getC();
    double getD();
    double getE();
    double getF();
    double getValG();
    double getValH();
    double getValF();
    int getNumOfChildren();
    StartBaseTreeNode* getFather();
    StartBaseTreeNode* getChild(int index);
    void setFather(StartBaseTreeNode *f);
    void setChild(StartBaseTreeNode *c);
    vector<StartBaseTreeNode *> ptrs_of_startbasetree;
    double distance_(double x,double y);
    int getLevel();
    void setLevel(int l);
    void setVal(double g,double h,double f);
    void setValG(double g);
    void setValH(double h);
    void setValF(double f);
private:
    int level;
    BaseJoints data;
    int number_of_children;
    double a,b,c,d,e,f;
    double val_g,val_h,val_f;
    vector<StartBaseTreeNode *> Children;
    StartBaseTreeNode *Father;
};
#endif

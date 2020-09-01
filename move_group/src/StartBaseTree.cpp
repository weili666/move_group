#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <math.h>
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
#include "Commen.h"
#include "PositionAttitude.h"
#include "ConfigJoints.h"
#include "BaseJoints.h"
#include "StartBaseTreeNode.h"
#include "StartBaseTree.h"
#define M_pi 3.141592653
using namespace std;

bool StartBaseTree::Root(BaseJoints &start) const
{
    if(root)
    {
        start=root->getData();
    }
}
 StartBaseTreeNode* StartBaseTree::getRoot()
 {
     return root;
 }

 void StartBaseTree::setRoot(StartBaseTreeNode *root_)
 {
     root->setData(root_->getData());
 }

StartBaseTreeNode * StartBaseTree::Extend( planning_scene_monitor::PlanningSceneMonitorPtr scene_, moveit::planning_interface::MoveGroup *robot_,const double &init_angle_of_robot,StartBaseTreeNode* current_base,const BaseJoints& target_base)
{

    double W=0.3;
    double ds=0.10;
    double new_alpha;

    double nearest_x;
    double nearest_y;
    double nearest_theta;
    int num_of_compare = 0;

    vector<StartBaseTreeNode *>::iterator q;
    StartBaseTreeNode *NewNodeReturn = new StartBaseTreeNode;

    vector<double> base_joints;

    nearest_x = current_base->getX();
    nearest_y = current_base->getY();
    nearest_theta = current_base->getTheta();

    planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    robot_->setStartState(current_robot_state);
    bool is_colliding;
    double min_f = 1000000;

    for(int i=0;i<8;i++)
    {
        StartBaseTreeNode *NewNode = new StartBaseTreeNode;
        new_alpha = i*M_PI/4.0;
        BaseJoints rand;
        vector<double> current_joints;

        double delta_dist;
        double dist_from_targ;
        double v1,v2,dx,dy;

        rand.x=nearest_x+ds*round(cos(new_alpha));
        current_joints.push_back(nearest_x);
        base_joints.push_back(rand.x);

        rand.y=nearest_y+ds*round(sin(new_alpha));
        current_joints.push_back(nearest_y);
        base_joints.push_back(rand.y);

        delta_dist = ds*sqrt(round(cos(new_alpha))*round(cos(new_alpha))+round(sin(new_alpha))*round(sin(new_alpha)));

        dx = abs(rand.x-target_base.x);
        dy = abs(rand.y-target_base.y);
        v1 = sqrt(2)*min(dx,dy);
        v2 = max(dx,dy)-min(dx,dy);
        dist_from_targ = v1+v2;

        rand.theta=new_alpha-init_angle_of_robot;
        current_joints.push_back(nearest_theta);
        base_joints.push_back(rand.theta);
        current_robot_state.copyJointGroupPositions( robot_->getName(), current_joints);
        string word[3]={"world_to_robot_x","world_to_robot_y","world_to_robot_roll"};
        std::vector<std::string> joint_names(word,word+3);
        current_robot_state.setVariablePositions( joint_names, base_joints);
        is_colliding = current_scene->isStateColliding( current_robot_state );
        cout<<"is colliding:"<<is_colliding<<endl;
        if(!is_colliding)
        {
            bool belong = false;
            for(list<StartBaseTreeNode *>::iterator iter = close_list.begin(); iter != close_list.end(); iter++)
            {
                if((rand.x == (*iter)->getX())&&(rand.y == (*iter)->getY()))
                {
                    belong = true;
                    break;
                }
                else
                    continue;
            }
            if(belong == true)
            {
                cout<<"belong is true!"<<endl;
                continue;
            }
            else
            {
                bool belong2 = false;

                for(list<StartBaseTreeNode *>::iterator iter = open_list.begin(); iter != open_list.end(); iter++)
                {
                    if((rand.x == (*iter)->getX())&&(rand.y == (*iter)->getY()))
                    {
                        NewNode = *iter;
                        belong2 = true;
                        break;
                    }
                    else
                        continue;
                }
                if(belong2 == true)
                {
                    double dist_from_current = current_base->getValG()+delta_dist;
                    if(dist_from_current < NewNode->getValG())
                    {
                        NewNode->setData(rand);
                        NewNode->setValG(dist_from_current);
                        NewNode->setValH(dist_from_targ);
                        NewNode->setValF(dist_from_current+dist_from_targ);
                        NewNode->setFather(current_base);
                        current_base->setChild(NewNode);
                        NewNode->setLevel(current_base->getLevel()+1);
                        cout<<"belong is false! belong2 is true! And new x1:"<<rand.x<<","<<NewNode->getX()<<" new y1:"<<rand.y<<","<<NewNode->getY()<<" new theta1:"<<rand.theta<<" dist_from_current:"<<dist_from_current<<" dist_from_targ:"<<dist_from_targ<<" value of function:"<<NewNode->getValF()<<"="<<NewNode->getValG()<<"+"<<NewNode->getValH()<<",target:"<<target_base.x<<","<<target_base.y<<endl;
                    }
                }
                else
                {
                    double dist_from_current = current_base->getValG()+delta_dist;
                    NewNode->setData(rand);
                    NewNode->setValG(dist_from_current);
                    NewNode->setValH(dist_from_targ);
                    NewNode->setValF(dist_from_current+dist_from_targ);
                    NewNode->setFather(current_base);
                    current_base->setChild(NewNode);
                    root->ptrs_of_startbasetree.push_back(NewNode);
                    NewNode->setLevel(current_base->getLevel()+1);
                    open_list.push_back(NewNode);
                    cout<<"belong is false! belong2 is false! And new x1:"<<rand.x<<","<<NewNode->getX()<<" new y1:"<<rand.y<<","<<NewNode->getY()<<" new theta1:"<<rand.theta<<" dist_from_current:"<<dist_from_current<<" dist_from_targ:"<<dist_from_targ<<" value of function:"<<NewNode->getValF()<<"="<<NewNode->getValG()<<"+"<<NewNode->getValH()<<",the size of open_list:"<<open_list.size()<<",target:"<<target_base.x<<","<<target_base.y<<endl;
                }


            }


        }
        else
        {
            cout<<"extend block!"<<endl;
//            getchar();
            continue;
        }
    }

    for(list<StartBaseTreeNode *>::iterator iter = open_list.begin(); iter != open_list.end(); iter++)
    {
        if((*iter)->getValF()<=min_f)
        {
            min_f = (*iter)->getValF();
            NewNodeReturn = (*iter);
            num_of_compare ++;
        }
    }

    open_list.remove(NewNodeReturn);
    if(num_of_compare > 0)
    {
        cout<<"number of compare:"<<num_of_compare<<endl;
        return NewNodeReturn;
    }
    else
    {
        return 0;
    }


}

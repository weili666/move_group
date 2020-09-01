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
#include"Commen.h"
#include"ExtendJoints.h"
#include"StartTreeNode.h"
#include"StartBaseTreeNode.h"
#include"StartBaseTree.h"
#include"StartTree.h"
#define M_pi 3.141592653
using namespace std;
bool StartTree::Root(ExtendJoints &start) const
{
    if(root)
    {
        start=root->getData();
    }
}

StartTreeNode* StartTree::getRoot()const
{
    return root;
}

void StartTree::setRoot(StartTreeNode *root_)
{
    root->setData(root_->getData());
}

StartTreeNode* StartTree::Extend(planning_scene_monitor::PlanningSceneMonitorPtr scene_, moveit::planning_interface::MoveGroup *robot_, const double &init_angle_of_robot)
{
    double resol_x=20;
    double resol_y=20;
    double new_alpha;
    double x_rand=resol_x*rand01()-resol_x/2;
    double y_rand=resol_y*rand01()-1;
    double nearest_x;
    double nearest_y;
    double nearest_theta;
    double TAN_R;
    StartTreeNode* NewNode=new StartTreeNode;
    StartTreeNode* Nearest=new StartTreeNode;
    double resol_1=2*M_pi;
    double resol_2=4*M_pi;
    double resol_3=4*M_pi;
    double resol_4=4*M_pi;
    double resol_5=4*M_pi;
    double resol_6=4*M_pi;
    double resol_7=4*M_pi;
    double ds=0.1;
    double min_dist=10000,dist;
    double joint_1_rand=resol_1*rand01()-resol_1/2;
    double joint_2_rand=resol_2*rand01()-resol_2/2;
    double joint_3_rand=resol_3*rand01()-resol_3/2;
    double joint_4_rand=resol_4*rand01()-resol_4/2;
    double joint_5_rand=resol_5*rand01()-resol_5/2;
    double joint_6_rand=resol_6*rand01()-resol_6/2;
    double joint_7_rand=resol_7*rand01()-resol_7/2;
    ExtendJoints ext_joints;
    vector<double> back_joints;
    vector<double>forward_joints;
    vector<StartTreeNode *>::iterator iter;
    for(iter=root->ptrs_of_starttree.begin();iter!=root->ptrs_of_starttree.end();iter++)
    {
        dist=(*iter)->_distance_(x_rand,y_rand,joint_1_rand,joint_2_rand,joint_3_rand,joint_4_rand,joint_5_rand,joint_6_rand,joint_7_rand);
        if(dist<min_dist)
        {
            Nearest=(*iter);
            min_dist=dist;
        }
    }
    nearest_x=Nearest->getData().x;
    nearest_y=Nearest->getData().y;
    nearest_theta=Nearest->getData().theta;
    if(x_rand!=Nearest->getData().x)
    {
        TAN_R=(y_rand-nearest_y)/(x_rand-nearest_x);

        if((x_rand-nearest_x>0)&&(TAN_R>0))
        {
            new_alpha=atan(TAN_R);
        }
        else if((x_rand-nearest_x<0)&&(TAN_R>0))
        {
            new_alpha=atan(TAN_R)-M_pi;
        }
        else if((x_rand-nearest_x>0)&&(TAN_R<0))
        {
            new_alpha=atan(TAN_R);
        }
        else if((x_rand-nearest_x<0)&&(TAN_R<0))
        {
            new_alpha=atan(TAN_R)+M_pi;
        }
    }
    else if(x_rand==nearest_x)
    {
        if(y_rand>nearest_y)
        {
            new_alpha=M_pi/2;
        }
        else if(y_rand<nearest_y)
        {
            new_alpha=-M_pi/2;
        }
    }

    planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    robot_->setStartState(current_robot_state);
    bool is_colliding;
    ext_joints.x=nearest_x+ds*(x_rand-nearest_x)/min_dist;
    back_joints.push_back(nearest_x);forward_joints.push_back(ext_joints.x);
    ext_joints.y=nearest_y+ds*(y_rand-nearest_y)/min_dist;
    back_joints.push_back(nearest_y);forward_joints.push_back(ext_joints.y);
    ext_joints.theta=new_alpha-init_angle_of_robot;
    back_joints.push_back(nearest_theta);forward_joints.push_back(ext_joints.theta);
    ext_joints.joint_1=Nearest->getData().joint_1+ds*(joint_1_rand-Nearest->getData().joint_1)/min_dist;
    back_joints.push_back(Nearest->getData().joint_1);forward_joints.push_back(ext_joints.joint_1);
    ext_joints.joint_2=Nearest->getData().joint_2+ds*(joint_2_rand-Nearest->getData().joint_2)/min_dist;
    back_joints.push_back(Nearest->getData().joint_2);forward_joints.push_back(ext_joints.joint_2);
    ext_joints.joint_3=Nearest->getData().joint_3+ds*(joint_3_rand-Nearest->getData().joint_3)/min_dist;
    back_joints.push_back(Nearest->getData().joint_3);forward_joints.push_back(ext_joints.joint_3);
    ext_joints.joint_4=Nearest->getData().joint_4+ds*(joint_4_rand-Nearest->getData().joint_4)/min_dist;
    back_joints.push_back(Nearest->getData().joint_4);forward_joints.push_back(ext_joints.joint_4);
    ext_joints.joint_5=Nearest->getData().joint_5+ds*(joint_5_rand-Nearest->getData().joint_5)/min_dist;
    back_joints.push_back(Nearest->getData().joint_5);forward_joints.push_back(ext_joints.joint_5);
    ext_joints.joint_6=Nearest->getData().joint_6+ds*(joint_6_rand-Nearest->getData().joint_6)/min_dist;
    back_joints.push_back(Nearest->getData().joint_6);forward_joints.push_back(ext_joints.joint_6);
    ext_joints.joint_7=Nearest->getData().joint_7+ds*(joint_7_rand-Nearest->getData().joint_7)/min_dist;
    back_joints.push_back(Nearest->getData().joint_7);forward_joints.push_back(ext_joints.joint_7);
    current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
    string word[10]={"world_to_robot_x","world_to_robot_y","world_to_robot_roll","support_to_base","left_jaco_joint_1","left_jaco_joint_2","left_jaco_joint_3","left_jaco_joint_4","left_jaco_joint_5","left_jaco_joint_6"};
    std::vector<std::string> joint_names(word,word+10);
    current_robot_state.setVariablePositions( joint_names, forward_joints);
    is_colliding = current_scene->isStateColliding( current_robot_state );
    cout<<"is colliding:"<<is_colliding<<endl;
    if(!is_colliding)
    {
        NewNode->setData(ext_joints);
        NewNode->setFather(Nearest);
        Nearest->setChild(NewNode);
        root->ptrs_of_startbasetree.push_back(NewNode);
        NewNode->setLevel(Nearest->getLevel()+1);
        return NewNode;
    }
    else
    {
        cout<<"extend block!"<<endl;
        return 0;
    }

}

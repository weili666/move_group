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
#include "BackHandTree.h"
#include "BackTreeNode.h"
#define M_pi 3.141592653
using namespace std;
BackTreeNode* BackHandTree::getRoot()
{
    return root;
}

void BackHandTree::setRoot(const PositionAttitude &t)
{
    root->setData(t);
}

void BackHandTree::CreateHandTree()
{
    cout<<"start create a new tree!"<<endl;
    double resol0=20.0,resol1=20.0,resol2=4.0,resol3=2*M_pi,resol4=2*M_pi,resol5=2*M_pi;
    double min_dist=10000.0,dist_end;
    double x_rand,y_rand,z_rand,alpha_rand,beta_rand,gamma_rand;
    double x,y,z,alpha,beta,gamma;
    double x_new,y_new,z_new,alpha_new,beta_new,gamma_new;
    double step=0.1;
    int num_tree_nodes=200000;
    for(int i=0;i<num_tree_nodes;i++)
    {
    min_dist=10000.0;
    x_rand=resol0*rand01()-resol0/2+root->getData().x;
    y_rand=resol1*rand01()-resol1/2+root->getData().y;
    z_rand=resol2*rand01()-resol2/2+root->getData().z;
    alpha_rand=resol3*rand01()-resol3/2+root->getData().alpha;
    beta_rand=resol4*rand01()-resol4/2+root->getData().beta;
    gamma_rand=resol5*rand01()-resol5/2+root->getData().gamma;
    vector<BackTreeNode *>::iterator b;
    BackTreeNode *Nearest=new BackTreeNode;
    BackTreeNode *NewNode=new BackTreeNode;
    cout<<"the number of tree nodes:"<<root->ptrs_of_backtree.size()<<endl;
    for(b=root->ptrs_of_backtree.begin();b<root->ptrs_of_backtree.end();b++)
    {   x=(*b)->getData().x;y=(*b)->getData().y;z=(*b)->getData().z;alpha=(*b)->getData().alpha;beta=(*b)->getData().beta;gamma=(*b)->getData().gamma;
        dist_end=sqrt(10*(x-x_rand)*(x-x_rand)+10*(y-y_rand)*(y-y_rand)+10*(z-z_rand)*(z-z_rand)+(alpha-alpha_rand)*(alpha-alpha_rand)+
                      (beta-beta_rand)*(beta-beta_rand)+(gamma-gamma_rand)*(gamma-gamma_rand));
        if(dist_end<min_dist)
        {
            Nearest=(*b);
            min_dist=dist_end;
        }
    }
    x=Nearest->getData().x;y=Nearest->getData().y;z=Nearest->getData().z;
    alpha=Nearest->getData().alpha;beta=Nearest->getData().beta;gamma=Nearest->getData().gamma;

   x_new=x+sqrt(10)*step*(x_rand-x)/min_dist;
   y_new=y+sqrt(10)*step*(y_rand-y)/min_dist;
   z_new=z+sqrt(10)*step*(z_rand-z)/min_dist;
   if(((y_new<-0.5)||(y_new>0.5)||(x_new<1.5)||(x_new>2.5)||(z_new>1.4))&&((x_new<-4.9)||(x_new>4.9)||(y_new<-4.2)||(y_new>-3.8))&&((x_new<-5.2)||(x_new>-4.8)||(y_new<-4.2)||(y_new>4.2))&&((x_new<-4.9)||(x_new>4.9)||(y_new<3.8)||(y_new>4.2))&&((x_new<4.8)||(x_new>5.2)||(y_new<-4.1)||(y_new>4.1))&&((x_new<-3.2)||(x_new>-2.8)||(y_new<-3.9)||(y_new>2.0))&&((x_new<-1.2)||(x_new>-0.8)||(y_new<-2.0)||(y_new>3.9))&&((x_new<2.8)||(x_new>3.2)||(y_new<-2.5)||(y_new>2.5))&&((x_new<0.5)||(x_new>3.0)||(y_new<2.3)||(y_new>2.7))&&((x_new<0.5)||(x_new>3.0)||(y_new<-2.7)||(y_new>-2.3)))
   {
       if((z_new>1.5)&&(z_new<2.5))
      {
       alpha_new=alpha+step*(alpha_rand-alpha)/min_dist;
       if(alpha_new>M_pi)
       {
           alpha_new=alpha_new-2*M_pi;
       }
       else if(alpha_new<-M_pi)
       {
           alpha_new=alpha_new+2*M_pi;
       }
       beta_new=beta+step*(beta_rand-beta)/min_dist;
       if(beta_new>M_pi)
       {
           beta_new=beta_new-2*M_pi;
       }
       else if(beta_new<-M_pi)
       {
           beta_new=beta_new+2*M_pi;
       }
       gamma_new=gamma+step*(gamma_rand-gamma)/min_dist;
       if(gamma_new>M_pi)
       {
           gamma_new=gamma_new-2*M_pi;
       }
       else if(gamma_new<-M_pi)
       {
           gamma_new=gamma_new+2*M_pi;
       }
       cout<<"add a new node, x:"<<x_new<<" y:"<<y_new<<" z:"<<z_new<<" alpha:"<<alpha_new<<" beta:"<<beta_new<<" gamma:"<<gamma_new<<endl;
       PositionAttitude New(x_new,y_new,z_new,alpha_new,beta_new,gamma_new);

       NewNode->setData(New);
       cout<<"level of root:"<<root->getLevel()<<endl;
       cout<<"level of nearest:"<<Nearest->getLevel()<<endl;
       NewNode->setLevel(Nearest->getLevel()+1);
       cout<<"level of new node:"<<NewNode->getLevel()<<endl;
       Nearest->setChild(NewNode);
       NewNode->setFather(Nearest);
       root->ptrs_of_backtree.push_back(NewNode);
        }
   }
   else
       continue;
   }
}

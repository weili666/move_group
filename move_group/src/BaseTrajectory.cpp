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
#include "StartBaseTreeNode.h"
#include "PositionAttitude.h"
#include "StartBaseTree.h"
#include "BaseTrajectory.h"
#include "Commen.h"
#define M_pi 3.141592653
using namespace std;
vector<BaseJoints> BaseTrajectory::CurveAStar()
{

    //============================Extend a chaos curve through A*=============================//
    double x_torr=0.1;
    double y_torr=0.1;
    double theta_torr= M_pi/3;
    double W=0.3;
    StartBaseTreeNode* find_node = new StartBaseTreeNode;
    StartBaseTreeNode* current_base = new StartBaseTreeNode;
    StartBaseTreeNode* new_node = new StartBaseTreeNode;
    vector<BaseJoints> backwards;
    vector<BaseJoints> forward;
    vector<StartBaseTreeNode *> back_node;

    double dist_between_start_targ = sqrt((target.x-ctree->getRoot()->getX())*(target.x-ctree->getRoot()->getX())+(target.y-ctree->getRoot()->getY())*(target.y-ctree->getRoot()->getY()));
    ctree->getRoot()->setValG(0.0);
    ctree->getRoot()->setValH(dist_between_start_targ);
    ctree->getRoot()->setValF(ctree->getRoot()->getValG()+ctree->getRoot()->getValH());
    ctree->open_list.push_back(ctree->getRoot());
    current_base = ctree->getRoot();
    cout<<"root:x:"<<ctree->getRoot()->getX()<<",y:"<<ctree->getRoot()->getY()<<",theta:"<<ctree->getRoot()->getTheta()<<",value of function:"<<ctree->getRoot()->getValF()<<"="<<ctree->getRoot()->getValG()<<"+"<<ctree->getRoot()->getValH()<<endl;
    cout<<"target:x:"<<target.x<<",y:"<<target.y<<",theta:"<<target.theta<<endl;
    cout<<"The first node position is : x = "<<current_base->getX()<<",y = "<<current_base->getY()<<endl;

    int i=0;
    int Max_Torr_Num=20000;
    int num_of_trajectory_node=1;
    bool judgement=true;
    int step;
    step=0;
    while(judgement)
    {
        step++;
        new_node = ctree->Extend(scene_,robot_,init_angle_of_robot_base,current_base,target);
        //cout<<"new node !"<<endl;
        if(new_node!=NULL)
        {
            //cout<<"new_node:x:"<<new_node->getX()<<",y:"<<new_node->getY()<<",theta:"<<new_node->getTheta()<<",value of function:"<<new_node->getValF()<<",target:x:"<<target.x<<",y:"<<target.y<<endl;
            judgement=((abs(new_node->getData().x-target.x)>x_torr)||(abs(new_node->getData().y-target.y)>y_torr))&&(i<Max_Torr_Num);
            current_base = new_node;
            ctree->close_list.push_back(new_node);
            i++;
            //cout<<"the size of close list:"<<ctree->close_list.size()<<",the size of open list:"<<ctree->open_list.size()<<endl;
        }
        else if(new_node==NULL)
        {
            //cout<<"haha3 !"<<endl;
            judgement=true;
            current_base = current_base->getFather();
            continue;
        }
        //cout<<"the number of new added node:"<<i<<endl;
        //cout<<endl;
    }
    current_base->setX(target.x);
    current_base->setY(target.y);
    current_base->setTHETA(target.theta);

    back_node.push_back(current_base);
    if(i<Max_Torr_Num)
    {
        find_node=current_base;
        while(find_node!=ctree->getRoot())
        {
            find_node=find_node->getFather();
            back_node.push_back(find_node);
            backwards.push_back(find_node->getData());
        }
        vector<StartBaseTreeNode *>::reverse_iterator r_iter;
        for(r_iter= back_node.rbegin();r_iter!= back_node.rend();r_iter++)
        {
            motion_chain.push_back(*r_iter);
            forward.push_back((*r_iter)->getData());
        }
        num_of_trajectory_node=motion_chain.size();
         cout<<"the number of trajectory nodes:"<<num_of_trajectory_node<<endl;
    }
    else
    {
        int index_of_node=0;
        int index_of_min_dist=0;
        double Level;
        double max_Level=0;
        vector<StartBaseTreeNode *>::iterator iter;
        for(iter=ctree->getRoot()->ptrs_of_startbasetree.begin();iter!=ctree->getRoot()->ptrs_of_startbasetree.end();iter++)
        {
            Level=(*iter)->getLevel();
            if(Level>max_Level)
            {
                index_of_min_dist=index_of_node;
                max_Level=Level;
            }
            index_of_node++;
        }
        find_node=ctree->getRoot()->ptrs_of_startbasetree[index_of_min_dist];

        while(find_node!=ctree->getRoot())
        {
            find_node=find_node->getFather();
            back_node.push_back(find_node);
            backwards.push_back(find_node->getData());
            num_of_trajectory_node++;
        }
        cout<<"the number of trajectory nodes:"<<num_of_trajectory_node<<endl;
        vector<StartBaseTreeNode *>::reverse_iterator r_iter;
        for(r_iter= back_node.rbegin();r_iter!= back_node.rend();r_iter++)
        {
            motion_chain.push_back(*r_iter);
            forward.push_back((*r_iter)->getData());
        }
    }


    //===========================smooth the trajectory==========================//
    planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    robot_->setStartState(current_robot_state);
    int start_smooth_index;
    int end_smooth_index;
    int exchange;
    double dist;
    string word[3]={"world_to_robot_x","world_to_robot_y","world_to_robot_roll"};
    std::vector<std::string> joint_names(word,word+3);
    int Num_of_Smooth=200;
    srand(time(NULL));
    for(int i=Num_of_Smooth;i>0;i--)
    {
 //        cout<<"smooth times:"<<Num_of_Smooth-i+1<<endl;
        end_smooth_index=5;
        start_smooth_index=3;

        while(end_smooth_index-start_smooth_index<10)
        {

            start_smooth_index=rand()%num_of_trajectory_node;
            end_smooth_index=rand()%num_of_trajectory_node;
            if(start_smooth_index>end_smooth_index)
            {
                exchange=start_smooth_index;
                start_smooth_index=end_smooth_index;
                end_smooth_index=exchange;
            }

        }

        bool Is_Colliding;//[end_smooth_index-start_smooth_index-1];

        bool jump=0;
        dist=motion_chain[start_smooth_index]->distance_(forward[end_smooth_index].x,forward[end_smooth_index].y);
        double x_0,y_0,theta_0,x_1,y_1,theta_1;
        double theta_m;

        x_0=motion_chain[start_smooth_index]->getData().x;
        y_0=motion_chain[start_smooth_index]->getData().y;
        theta_0=motion_chain[start_smooth_index]->getData().theta+init_angle_of_robot_base;
        x_1=motion_chain[end_smooth_index]->getData().x;
        y_1=motion_chain[end_smooth_index]->getData().y;
        theta_1=motion_chain[end_smooth_index]->getData().theta+init_angle_of_robot_base;
        double xx[end_smooth_index-start_smooth_index+1];
        double yy[end_smooth_index-start_smooth_index+1];
        double ttheta[end_smooth_index-start_smooth_index+1];
        int num_nodes=end_smooth_index-start_smooth_index+1;
        if(dist>2*W)
        {
            xx[0]=x_0;yy[0]=y_0;ttheta[0]=theta_0;xx[num_nodes-1]=x_1;yy[num_nodes-1]=y_1;ttheta[num_nodes-1]=theta_1;
            if(x_1>x_0)
            {
                theta_m=atan((y_1-y_0)/(x_1-x_0));
            }
            else if((x_1<x_0)&&(y_1>y_0))
            {
                theta_m=M_pi+atan((y_1-y_0)/(x_1-x_0));
            }
            else if((x_1<x_0)&&(y_1<y_0))
            {
                theta_m=atan((y_1-y_0)/(x_1-x_0))-M_pi;
            }
            else if((x_0==x_1)&&(y_1>y_0))
            {
                theta_m=M_pi/2;
            }
            else if((x_0==x_1)&&(y_1<y_0))
            {
                theta_m=-M_pi/2;
            }
            for(int i=1;i<num_nodes-1;i++)
            {
                xx[i]=xx[0]+i*(xx[num_nodes-1]-xx[0])/(num_nodes-1);
                yy[i]=yy[0]+i*(yy[num_nodes-1]-yy[0])/(num_nodes-1);
                ttheta[i]=theta_m;
            }
            ttheta[num_nodes-1]=theta_m;
            for(int i=0;i<num_nodes-1;i++)
            {
                vector<double> forward_joints;
                vector<double> back_joints;
                forward_joints.push_back(xx[i+1]);
                forward_joints.push_back(yy[i+1]);
                forward_joints.push_back(ttheta[i+1]-init_angle_of_robot_base);
                back_joints.push_back(xx[i]);
                back_joints.push_back(yy[i]);
                back_joints.push_back(ttheta[i]-init_angle_of_robot_base);
                current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
                current_robot_state.setVariablePositions( joint_names, forward_joints);
                Is_Colliding = current_scene->isStateColliding( current_robot_state );
                if(Is_Colliding)
                {
                    jump=1;
                    break;
                }
            }
           if(jump==0)
           {
            for(int i=0;i<num_nodes;i++)
            {
                motion_chain[start_smooth_index+i]->setX(xx[i]);
                motion_chain[start_smooth_index+i]->setY(yy[i]);
                motion_chain[start_smooth_index+i]->setTHETA(ttheta[i]-init_angle_of_robot_base);
                forward[start_smooth_index+i].x=xx[i];
                forward[start_smooth_index+i].y=yy[i];
                forward[start_smooth_index+i].theta=ttheta[i]-init_angle_of_robot_base;
            }}
        }

    }

    cout<<"The size of trajectory:"<<motion_chain.size()<<","<<forward.size()<<endl;

    //==========================Spline Curves the Trajectory=============================//
   vector<StartBaseTreeNode*> turning_points;
   vector<BaseJoints> turning_points_xyt;
   vector<int> turning_index;
   double limit=0.001;
   int num_of_spline_curve;
   for(int i=2;i<motion_chain.size()-2;i++)
   {
       double dtan;
       dtan=(((motion_chain[i+1]->getData().y-motion_chain[i]->getData().y)/(motion_chain[i+1]->getData().x-motion_chain[i]->getData().x))-((motion_chain[i]->getData().y-motion_chain[i-1]->getData().y)/(motion_chain[i]->getData().x-motion_chain[i-1]->getData().x)))/(1+((motion_chain[i+1]->getData().y-motion_chain[i]->getData().y)/(motion_chain[i+1]->getData().x-motion_chain[i]->getData().x))*((motion_chain[i]->getData().y-motion_chain[i-1]->getData().y)/(motion_chain[i]->getData().x-motion_chain[i-1]->getData().x)));

       if(abs(dtan)>limit)
       {
           turning_points.push_back(motion_chain[i]);
           turning_points_xyt.push_back(forward[i]);
           turning_index.push_back(i);
           //cout<<"TURNNING POINT,index:"<<i<<",x:"<<motion_chain[i]->getData().x<<",y:"<<motion_chain[i]->getData().y<<",theta:"<<motion_chain[i]->getData().theta<<endl;
       }
   }
num_of_spline_curve=turning_points.size()+1;
cout<<"number of spline curve:"<<num_of_spline_curve<<endl;

if(turning_index.size()!=0)
{
    for( int i=0; i<turning_index.size();i++ )
    {
        StartBaseTreeNode* add_node = new StartBaseTreeNode;
        BaseJoints turn( turning_points_xyt[i].x, turning_points_xyt[i].y, motion_chain[turning_index[i]+i+1]->getData().theta );
        //cout<<"turning point x:"<<turning_points_xyt[i].x<<",y:"<<turning_points_xyt[i].y<<",theta:"<<motion_chain[turning_index[i]+i+1]->getData().theta <<endl;
        add_node->setData( turn );
        add_node->setFather(motion_chain[turning_index[i]+i]);
        motion_chain.insert(motion_chain.begin()+turning_index[i]+i+1,add_node);
        motion_chain[turning_index[i]+i+2]->setFather(add_node);
        forward.insert(forward.begin()+turning_index[i]+i+1,turn);
    }
}

BaseJoints turn_begin( motion_chain[0]->getData().x,motion_chain[0]->getData().y,motion_chain[1]->getData().theta );
forward.insert(forward.begin()+1,turn_begin);
StartBaseTreeNode* add_start = new StartBaseTreeNode;
add_start->setData(turn_begin);
add_start->setFather(motion_chain[0]);
motion_chain.insert(motion_chain.begin()+1,add_start);
motion_chain[2]->setFather(add_start);

int Len = motion_chain.size();
BaseJoints turn_end( motion_chain[Len-1]->getData().x,motion_chain[Len-1]->getData().y,motion_chain[Len-2]->getData().theta );
forward.insert(forward.begin()+Len-1,turn_end);
StartBaseTreeNode* add_end = new StartBaseTreeNode;
add_end->setData(turn_end);
add_end->setFather(motion_chain[Len-2]);
motion_chain.insert(motion_chain.begin()+Len-1,add_end);
motion_chain[Len]->setFather(add_end);

forward[forward.size()-1].x = target.x;
forward[forward.size()-1].y = target.y;
forward[forward.size()-1].theta = target.theta;

//cout<<"The size of trajectory after insert:"<<motion_chain.size()<<","<<forward.size()<<endl;
//for(int i = 0; i < forward.size(); i ++)
//{
//    cout<<"forward["<<i<<"],x:"<<forward[i].x<<",y:"<<forward[i].y<<",theta:"<<forward[i].theta<<endl;
//}

//double a[num_of_spline_curve];
//double b[num_of_spline_curve];
//double c[num_of_spline_curve];
//double d[num_of_spline_curve];
//double e[num_of_spline_curve];
//double f[num_of_spline_curve];
//double max_C=20;
//double x[num_of_spline_curve+1];
//double y[num_of_spline_curve+1];
//double theta[num_of_spline_curve+1];
//double k[num_of_spline_curve+1];
//double p[num_of_spline_curve+1];
//double x_r[num_of_spline_curve+1];
//double y_r[num_of_spline_curve+1];

//double m1;
//double m2;
//double h;
//cout<<"the number of nodes of the entile trajectory is:"<<motion_chain.size()<<endl;
//x[0]=motion_chain[0]->getData().x;
//y[0]=motion_chain[0]->getData().y;
//theta[0]=motion_chain[0]->getData().theta+init_angle_of_robot_base;
//cout<<"motion_chain[0]->getData().theta:"<<motion_chain[0]->getData().theta<<endl;
//cout<<"The first point,x:"<<x[0]<<",y:"<<y[0]<<",theta:"<<theta[0]<<endl;
//cout<<"init_angle_of_robot_base:"<<init_angle_of_robot_base<<endl;

//if(turning_points.size()!=0)
//{
//    cout<<"There are more than one turning points!"<<endl;
//    for(int i=0;i<turning_points.size();i++)
//    {
//    x[i+1]=turning_points_xyt[i].x;
//    y[i+1]=turning_points_xyt[i].y;
//    double theta_turning= (motion_chain[turning_index[i]-1]->getData().theta+motion_chain[turning_index[i]+1]->getData().theta)/2;
//    theta[i+1]=theta_turning+init_angle_of_robot_base;
//    }
//    x[num_of_spline_curve]=motion_chain[motion_chain.size()-1]->getData().x;
//    y[num_of_spline_curve]=motion_chain[motion_chain.size()-1]->getData().y;
//    theta[num_of_spline_curve]=motion_chain[motion_chain.size()-1]->getData().theta+init_angle_of_robot_base;

//    cout<<"The last point,x:"<<x[num_of_spline_curve]<<",y:"<<y[num_of_spline_curve]<<",theta["<<num_of_spline_curve<<"]:"<<theta[num_of_spline_curve]<<endl;

//}
//else if(turning_points.size()==0)
//{
//    cout<<"Turning point is the end point!"<<endl;
//    x[1]=motion_chain[motion_chain.size()-1]->getData().x;
//    y[1]=motion_chain[motion_chain.size()-1]->getData().y;
//    theta[1]=motion_chain[motion_chain.size()-1]->getData().theta+init_angle_of_robot_base;
//    cout<<"The last point,x:"<<x[1]<<",y:"<<y[1]<<",theta[1]:"<<theta[1]<<endl;
//}
//int max_random_times_of_spline_curve=100;
//int num_of_points;
//int start_index;
//    p[0]= sqrt((y[1]-y[0])*(y[1]-y[0])+(x[1]-x[0])*(x[1]-x[0]))/(2*(((y[1]-y[0])*cos(theta[0])-(x[1]-x[0])*sin(theta[0]))/((y[1]-y[0])*sin(theta[0])+(x[1]-x[0])*cos(theta[0]))));
//    for(int i=0;i<num_of_spline_curve;i++)
//    {
//        cout<<"calculate the "<<i<<"th spline curve! All include "<<num_of_spline_curve<<" curves!"<<endl;
//        for(int num=0;num<max_random_times_of_spline_curve;num++)
//        {
//        bool jump=0;
//        bool Is_Colliding=0;
//        x_r[i+1]=abs((x[i+1]-x[i])*cos((theta[i]+theta[i+1])/2)+(y[i+1]-y[i])*sin((theta[i]+theta[i+1])/2));
//        cout<<"x1:"<<x_r[i+1]<<endl;

//        y_r[i+1]=(y[i+1]-y[i])*cos((theta[i]+theta[i+1])/2)-(x[i+1]-x[i])*sin((theta[i]+theta[i+1])/2);
//        cout<<"y1:"<<y_r[i+1]<<endl;

//        double k0=tan((theta[i]-theta[i+1])/2);

//        cout<<"after tan!,k0="<<k0<<endl;
//        double k1=tan((theta[i+1]-theta[i])/2);
//        cout<<"after tan!,k1="<<k1<<endl;
//        h=x_r[i+1];
//        p[i+1]=max_C*rand01()-max_C/2;
//        while(abs(p[i+1])<0.01)
//        {
//            p[i+1]=max_C*rand01()-max_C/2;
//        }

//        m1=(1+k0*k0)*sqrt(1+k0*k0)/(2*p[i]);
//        m2=(1+k1*k1)*sqrt(1+k1*k1)/(2*p[i+1]);

//        a[i]=0;
//        b[i]=k0;
//        c[i]=m1;
//        d[i]=(20*y_r[i+1]-(8*k1+12*k0)*h+(m2-6*m1)*h*h)/(2*h*h*h);
//        e[i]=(-15*y_r[i+1]+(7*k1+8*k0)*h+(3*m1-m2)*h*h)/(h*h*h*h);
//        f[i]=(12*y_r[i+1]-6*(k1+k0)*h+(m2-2*m1)*h*h)/(2*h*h*h*h*h);
//        cout<<"the "<<i<<" th curve, the"<<num<<" th calculate: b:"<<b[i]<<"c:"<<c[i]<<" d:"<<d[i]<<" e:"<<e[i]<<" f:"<<f[i]<<endl;
//        if(turning_index.size()!=0)
//        {if(i==0)
//        {
//            num_of_points=turning_index[i];
//            start_index=0;
//        }
//        else if(i==num_of_spline_curve-1)
//        {
//            num_of_points=motion_chain.size()-1-turning_index[i-1];
//            start_index=turning_index[i-1];
//        }
//        else
//        {
//            num_of_points=turning_index[i]-turning_index[i-1];
//            start_index=turning_index[i-1];
//        }}
//        else if(turning_index.size()==0)
//        {
//            num_of_points=motion_chain.size()-1;
//            start_index=0;
//        }
//        cout<<"From the "<<start_index<<"th node ! The number of points of this spline curve is:"<<num_of_points<<endl;
//        double xx_r[num_of_points+1];
//        double yy_r[num_of_points+1];
//        double ttheta_r[num_of_points+1];
//        double xx[num_of_points+1];
//        double yy[num_of_points+1];
//        double ttheta[num_of_points+1];
//        for(int j=0;j<=num_of_points;j++)
//        {
//            xx_r[j]=(j*h/num_of_points);
//            yy_r[j]=b[i]*(j*h/num_of_points)+c[i]*(j*h/num_of_points)*(j*h/num_of_points)+d[i]*(j*h/num_of_points)*(j*h/num_of_points)*(j*h/num_of_points)+e[i]*(j*h/num_of_points)*(j*h/num_of_points)*(j*h/num_of_points)*(j*h/num_of_points)+f[i]*(j*h/num_of_points)*(j*h/num_of_points)*(j*h/num_of_points)*(j*h/num_of_points)*(j*h/num_of_points);
//            ttheta_r[j]=atan(b[i]+2*c[i]*(j*h/num_of_points)+3*d[i]*(j*h/num_of_points)*(j*h/num_of_points)+4*e[i]*(j*h/num_of_points)*(j*h/num_of_points)*(j*h/num_of_points)+5*f[i]*(j*h/num_of_points)*(j*h/num_of_points)*(j*h/num_of_points)*(j*h/num_of_points));
//            xx[j]=motion_chain[start_index]->getData().x+xx_r[j]*cos((theta[i]+theta[i+1])/2)-yy_r[j]*sin((theta[i]+theta[i+1])/2);
//            yy[j]=motion_chain[start_index]->getData().y+xx_r[j]*sin((theta[i]+theta[i+1])/2)+yy_r[j]*cos((theta[i]+theta[i+1])/2);
//            ttheta[j]=(theta[i]+theta[i+1])/2+ttheta_r[j];
//        }

//        for(int j=0;j<num_of_points;j++)
//        {
//            vector<double> forward_joints;
//            vector<double> back_joints;
//            forward_joints.push_back(xx[j+1]);
//            forward_joints.push_back(yy[j+1]);
//            forward_joints.push_back(ttheta[j+1]-init_angle_of_robot_base);
//            back_joints.push_back(xx[j]);
//            back_joints.push_back(yy[j]);
//            back_joints.push_back(ttheta[j]-init_angle_of_robot_base);
//            current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
//            current_robot_state.setVariablePositions( joint_names, forward_joints);
//            Is_Colliding = current_scene->isStateColliding( current_robot_state );
//            if(Is_Colliding)
//            {
//                jump=1;
//                break;
//            }
//        }
//        if(jump==0)
//        {
//            cout<<"find a smooth spline trajectory of "<<i<<" th load !"<<endl;
//            cout<<"final a:"<<a[i]<<" final b:"<<b[i]<<" final c:"<<c[i]<<" final d:"<<d[i]<<" final e:"<<e[i]<<" final f:"<<f[i]<<endl;
//         for(int j=0;j<=num_of_points;j++)
//         {
//             cout<<"curve coordinate x:"<<xx[j]<<",y:"<<yy[j]<<",theta:"<<ttheta[j]<<endl;
//             motion_chain[start_index+j]->setX(xx[j]);
//             motion_chain[start_index+j]->setY(yy[j]);
//             motion_chain[start_index+j]->setTHETA(ttheta[j]-init_angle_of_robot_base);
//             motion_chain[start_index+j]->setABCDEF(a[i],b[i],c[i],d[i],e[i],f[i]);
//             forward[start_index+j].x=xx[j];
//             forward[start_index+j].y=yy[j];
//             forward[start_index+j].theta=ttheta[j]-init_angle_of_robot_base;
//         }
//        break;
//        }
//        else
//        {
//            continue;
//        }
//    }
//}


    return forward;

}

StartBaseTree* BaseTrajectory::getTree()
{
    return ctree;
}

vector<StartBaseTreeNode *> BaseTrajectory::getChain()
{
    return motion_chain;
}

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

#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include "PositionAttitude.h"
#include "BaseJoints.h"
#include "ExtendJoints.h"
#include "Commen.h"
#include "GraspMobilePosePlan.h"
#include <opencv2/core/eigen.hpp>
#include <ctime>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define M_pi 3.141592653
using namespace std;

geometry_msgs::Pose GraspMobilePosePlan::TransformFromObjToRobot(const geometry_msgs::Pose& obj_pose,const geometry_msgs::Pose& robot_pose)
{
    Eigen::Quaterniond qo(obj_pose.orientation.w,obj_pose.orientation.x,obj_pose.orientation.y,obj_pose.orientation.z);
    Eigen::Quaterniond qr(robot_pose.orientation.w,robot_pose.orientation.x,robot_pose.orientation.y,robot_pose.orientation.z);
    Eigen::Quaterniond qo_last = qr.inverse()*qo;
    geometry_msgs::Pose relative;

    double theta = 2*asin(robot_pose.orientation.z);
    double xi = base_init.x;
    double yi = base_init.y;
    relative.position.x = xi+(obj_pose.position.x-robot_pose.position.x)*cos(theta)+(obj_pose.position.y-robot_pose.position.y)*sin(theta);
    relative.position.y = yi-(obj_pose.position.x-robot_pose.position.x)*sin(theta)+(obj_pose.position.y-robot_pose.position.y)*cos(theta);
    relative.position.z = obj_pose.position.z;
    relative.orientation.w = qo_last.w();
    relative.orientation.x = qo_last.x();
    relative.orientation.y = qo_last.y();
    relative.orientation.z = qo_last.z();
    return relative;

}

int GraspMobilePosePlan::Get_Chosen_Grasp_Index()
{
    return index_chosen_grasp;
}

ArmJoints GraspMobilePosePlan::Get_Joint_Arm_Last()
{
    return joint_arm_last;
}

BaseJoints GraspMobilePosePlan::Get_Chosen_Mobile()
{
    return chosen_mobile;
}

PositionAttitude* GraspMobilePosePlan::Get_Targ_Pa_Last()
{
    return pa_grasps[index_chosen_grasp];
}

double GraspMobilePosePlan::Get_Center_X()
{
    return center_x;
}

double GraspMobilePosePlan::Get_Center_Y()
{
    return center_y;
}

double GraspMobilePosePlan::Get_Center_Z()
{
    return center_z;
}

double GraspMobilePosePlan::Get_Center_Theta()
{
    return center_theta;
}

double GraspMobilePosePlan::Get_Max_X()
{
    return x_max;
}

double GraspMobilePosePlan::Get_Min_X()
{
    return x_min;
}

double GraspMobilePosePlan::Get_Max_Y()
{
    return y_max;
}

double GraspMobilePosePlan::Get_Min_Y()
{
    return y_min;
}

double GraspMobilePosePlan::Get_Max_Z()
{
    return z_max;
}

double GraspMobilePosePlan::Get_Min_Z()
{
    return z_min;
}

double GraspMobilePosePlan::CalcDistToInitAndTarg(double x,double y)  //the smaller ,the better;
{
    double val;
    val = sqrt((x-base_init.x)*(x-base_init.x)+(y-base_init.y)*(y-base_init.y))+sqrt((x-base_targ.x)*(x-base_targ.x)+(y-base_targ.y)*(y-base_targ.y));
    return val;
}

double GraspMobilePosePlan::MobileEvaluate(double x, double y, double theta) //the bigger ,the better;
{
    double val;
//    double center_theta;
    double xi,yi,xo,yo;
    double L1,L2;
    double sigma_x=1.0,sigma_y=1.0,sigma_theta=0.3;
    double length_agv=0.76,width_agv=0.61;
    double x_obs1=-1.862,y_obs1=-0.432,x_obs2=2.338,y_obs2=0.778;
    double lambda_edge_1 = 1;
    double lambda_edge_2 = 1;
    double lambda_theta = 1;

    xi=base_init.x;
    yi=base_init.y;
    xo=base_targ.x;
    yo=base_targ.y;
    L1=sqrt((x-xi)*(x-xi)+(y-yi)*(y-yi));
    L2=sqrt((xo-x)*(xo-x)+(yo-y)*(yo-y));
    center_theta = 0;
//    if((y+width_agv*sin(theta)/2+length_agv*cos(theta)/2-y_obs1<0)||(y-width_agv*sin(theta)/2-length_agv*cos(theta)/2-y_obs2)>0)
//    {
//        val=(1/(2*M_PI*sigma_x*sigma_y))*exp(-0.5*(((x-center_x)/sigma_x)*((x-center_x)/sigma_x)+((y-center_y)/sigma_y)*((y-center_y)/sigma_y)))-lambda_edge_1*exp(-lambda_edge_2*abs(y-y_obs1))+lambda_theta*(1/(sqrt(2*M_PI)*sigma_theta))*exp(-0.5*((theta-center_theta)/sigma_theta)*((theta-center_theta)/sigma_theta));
//    }
//    else
//    {
//        val=(1/(2*M_PI*sigma_x*sigma_y))*exp(-0.5*(((x-center_x)/sigma_x)*((x-center_x)/sigma_x)+((y-center_y)/sigma_y)*((y-center_y)/sigma_y)))-lambda_edge_1+lambda_theta*(1/(sqrt(2*M_PI)*sigma_theta))*exp(-0.5*((theta-center_theta)/sigma_theta)*((theta-center_theta)/sigma_theta));
//    }


    if(y - y_obs1 < 0)
    {
        val=(1/(2*M_PI*sigma_x*sigma_y))*exp(-0.5*(((x-center_x)/sigma_x)*((x-center_x)/sigma_x)+((y-center_y)/sigma_y)*((y-center_y)/sigma_y)))-lambda_edge_1*exp(-lambda_edge_2*abs(y-y_obs1))+lambda_theta*(1/(sqrt(2*M_PI)*sigma_theta))*exp(-0.5*((theta-center_theta)/sigma_theta)*((theta-center_theta)/sigma_theta));
    }
    else if(y - y_obs2 > 0)
    {
        val=(1/(2*M_PI*sigma_x*sigma_y))*exp(-0.5*(((x-center_x)/sigma_x)*((x-center_x)/sigma_x)+((y-center_y)/sigma_y)*((y-center_y)/sigma_y)))-lambda_edge_1*exp(-lambda_edge_2*abs(y-y_obs2))+lambda_theta*(1/(sqrt(2*M_PI)*sigma_theta))*exp(-0.5*((theta-center_theta)/sigma_theta)*((theta-center_theta)/sigma_theta));
    }
    else
    {
        val=(1/(2*M_PI*sigma_x*sigma_y))*exp(-0.5*(((x-center_x)/sigma_x)*((x-center_x)/sigma_x)+((y-center_y)/sigma_y)*((y-center_y)/sigma_y)))-lambda_edge_1+lambda_theta*(1/(sqrt(2*M_PI)*sigma_theta))*exp(-0.5*((theta-center_theta)/sigma_theta)*((theta-center_theta)/sigma_theta));
    }
    return val;
}

double GraspMobilePosePlan::ArmEvaluate(const ArmJoints&joint_arm)
{
    double val;
    double joint1_eq=0,joint2_eq=M_PI,joint3_eq=M_PI,joint4_eq=0,joint5_eq=0,joint6_eq=0;
    //double joint1_eq = -1.4735840227881702, joint2_eq = 2.919063173960516, joint3_eq = 1.0135101613037494, joint4_eq = -2.0836849337533323, joint5_eq = 1.443466866652682, joint6_eq = 1.3149469735032022;
    double omega1 = 2.0, omega2 = 2.0, omega3 = 2.0, omega4 = 1.0, omega5 = 1.0, omega6 = 1.0;
    val=(1.0/9.0)*(omega1*((joint_arm.joint1-joint1_eq)/(2*M_PI))*((joint_arm.joint1-joint1_eq)/(2*M_PI))
                  +omega2*((joint_arm.joint2-joint2_eq)/(2*M_PI))*((joint_arm.joint2-joint2_eq)/(2*M_PI))
                  +omega3*((joint_arm.joint3-joint3_eq)/(2*M_PI))*((joint_arm.joint3-joint3_eq)/(2*M_PI))
                  +omega4*((joint_arm.joint4-joint4_eq)/(2*M_PI))*((joint_arm.joint4-joint4_eq)/(2*M_PI))
                  +omega5*((joint_arm.joint5-joint5_eq)/(2*M_PI))*((joint_arm.joint5-joint5_eq)/(2*M_PI))
                  +omega6*((joint_arm.joint6-joint6_eq)/(2*M_PI))*((joint_arm.joint6-joint6_eq)/(2*M_PI)));
    return val;
}

double GraspMobilePosePlan::GraspEvaluate(const geometry_msgs::Pose& gp, const BaseJoints& bj)
{
    Eigen::Quaterniond orientation( gp.orientation.w, gp.orientation.x, gp.orientation.y, gp.orientation.z );
    double height = 0.86;

    cv::Mat R_orient = cv::Mat(3, 3, CV_32F);
    cv::Mat nor_vec = cv::Mat(3, 1, CV_32F);
    cv::Mat point_to_base = cv::Mat(3, 1, CV_32F);
    Eigen::Matrix3d orient = orientation.toRotationMatrix();

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R_orient.at<float>(i,j)=orient(i,j);
        }
    }
    nor_vec.at<float>(0,0) = R_orient.at<float>(0,2);
    nor_vec.at<float>(1,0) = R_orient.at<float>(1,2);
    nor_vec.at<float>(2,0) = R_orient.at<float>(2,2);

    float ds = sqrt((bj.x - gp.position.x)*(bj.x - gp.position.x)+(bj.y - gp.position.y)*(bj.y - gp.position.y)+(height - gp.position.z)*(height - gp.position.z));
    point_to_base.at<float>(0,0) = (bj.x - gp.position.x)/ds;
    point_to_base.at<float>(1,0) = (bj.y - gp.position.y)/ds;
    point_to_base.at<float>(2,0) = (height - gp.position.z)/ds;

    double val1 = point_to_base.at<float>(0,0)*nor_vec.at<float>(0,0) + point_to_base.at<float>(1,0)*nor_vec.at<float>(1,0) + point_to_base.at<float>(2,0)*nor_vec.at<float>(2,0);
    double val2 = -point_to_base.at<float>(0,0)*cos(bj.theta)-point_to_base.at<float>(1,0)*sin(bj.theta);
    //double val3 = -nor_vec.at<float>(0,0)*cos(bj.theta)-nor_vec.at<float>(1,0)*sin(bj.theta);
    double val = (1.0/2.0)*(val1 + val2);
    cout<<"*********/*/*/*/*/*/base x:"<<bj.x<<",base y:"<<bj.y<<",base theta:"<<bj.theta<<",gp.orientation x:"<<nor_vec.at<float>(0,0)<<",gp.orientation y:"<<nor_vec.at<float>(1,0)<<",gp.orientation z:"<<nor_vec.at<float>(2,0)<<",point_to_base x:"<<point_to_base.at<float>(0,0)<<",point_to_base y:"<<point_to_base.at<float>(1,0)<<",value1:"<<val1<<",value2:"<<val2<<endl;
    return val;

}

void GraspMobilePosePlan::CreateMobileGraspPose()
{
    double xi,yi,xo,yo,x,y,theta;
    double val=10000,middle_val=0;

    double middle_x,middle_y,middle_theta;
    int num_iter_base=100;

    bool Is_Colliding;
    double range_r=1.4,range_alpha=2*M_PI;


    vector<PositionAttitude*>::iterator pa_iter;

    cout<<"center_x:"<<center_x<<",center_y:"<<center_y<<",center_z:"<<center_z<<endl;
    xi=base_init.x;
    yi=base_init.y;
    xo=base_targ.x;
    yo=base_targ.y;

    vector<double> forward_joints;
    vector<double> back_joints;
    //=========================Get Seed X Y THETA==========================//
    planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    robot_->setStartState(current_robot_state);


    string word[10]={"world_to_robot_x","world_to_robot_y","world_to_robot_roll","support_to_base","jaco_joint_1","jaco_joint_2","jaco_joint_3","jaco_joint_4","jaco_joint_5","jaco_joint_6"};
    std::vector<std::string> joint_names_0(word,word+10);
    for(int i=0;i<num_iter_base;i++)
    {
        double theta_rand = range_alpha*rand01()-range_alpha/2;

        middle_x=center_x+range_r*rand01()*cos(theta_rand);
        middle_y=center_y+range_r*rand01()*sin(theta_rand);
        double x_1=center_x;
        double x_0=middle_x;
        double y_1=center_y;
        double y_0=middle_y;
        if(x_1>x_0)
        {
            middle_theta=atan((y_1-y_0)/(x_1-x_0));
        }
        else if((x_1<x_0)&&(y_1>y_0))
        {
            middle_theta=M_pi+atan((y_1-y_0)/(x_1-x_0));
        }
        else if((x_1<x_0)&&(y_1<y_0))
        {
            middle_theta=atan((y_1-y_0)/(x_1-x_0))-M_pi;
        }
        else if((x_0==x_1)&&(y_1>y_0))
        {
            middle_theta=M_pi/2;
        }
        else if((x_0==x_1)&&(y_1<y_0))
        {
            middle_theta=-M_pi/2;
        }


        forward_joints.clear();

        back_joints.push_back(xi-xi);
        back_joints.push_back(yi-yi);
        back_joints.push_back(base_init.theta);
        back_joints.push_back(0);
        back_joints.push_back(arm_init.joint1);
        back_joints.push_back(arm_init.joint2);
        back_joints.push_back(arm_init.joint3);
        back_joints.push_back(arm_init.joint4);
        back_joints.push_back(arm_init.joint5);
        back_joints.push_back(arm_init.joint6);

        back_joints.clear();

        forward_joints.push_back(middle_x-xi);
        forward_joints.push_back(middle_y-yi);
        forward_joints.push_back(middle_theta-init_joint_of_robot_base);
        forward_joints.push_back(0);
        forward_joints.push_back(arm_init.joint1);
        forward_joints.push_back(arm_init.joint2);
        forward_joints.push_back(arm_init.joint3);
        forward_joints.push_back(arm_init.joint4);
        forward_joints.push_back(arm_init.joint5);
        forward_joints.push_back(arm_init.joint6);


        current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
        current_robot_state.setVariablePositions( joint_names_0, forward_joints);
        Is_Colliding = current_scene->isStateColliding( current_robot_state );
        cout<<"Is Colliding?"<<Is_Colliding<<",middle_x:"<<middle_x<<",middle_y:"<<middle_y<<"middle_theta:"<<middle_theta<<endl;
        if(Is_Colliding==0)
        {
            middle_val=-MobileEvaluate(middle_x, middle_y, middle_theta);//CalcDistToInitAndTarg(middle_x,middle_y)
            if(middle_val<val)
            {
                seed_x=middle_x;
                seed_y=middle_y;
                seed_theta=middle_theta;
                val=middle_val;
            }
        }

    }
    cout<<"seed_x:"<<seed_x<<" ,seed_y:"<<seed_y<<" ,seed_theta:"<<seed_theta<<endl;


    //===========================Iterate===========================//

    double fval=-10000,ival=-10000;
    index_chosen_grasp = 0;
    int index_chosen_grasp_next;
    BaseJoints chosen_mobile_next;
    int num_iter_base_2=40;
    int num_fi_iterate=5;
    double range_iterate_r=0.3,range_iterate_theta=M_PI;
    double range_iterate_grasp=0.6;
    double w1 = 1;
    double w2 = 2;
    double w3 = 1;


    cout<<"begin iterate"<<endl;
    chosen_mobile.setBase(seed_x,seed_y,seed_theta);
    cout<<"set base"<<endl;

    geometry_msgs::Pose robot_pose;
    robot_pose.orientation.x=0;
    robot_pose.orientation.y=0;
    robot_pose.orientation.z=sin((seed_theta-init_joint_of_robot_base)/2);//0.3127;
    robot_pose.orientation.w=cos((seed_theta-init_joint_of_robot_base)/2);

    robot_pose.position.x=seed_x;
    robot_pose.position.y=seed_y;
    robot_pose.position.z=0;
    int num=0;
    for(pa_iter=pa_grasps.begin();pa_iter!=pa_grasps.end();pa_iter++)
    {
        //go to grasp the box
         geometry_msgs::Pose obj_pose;

         obj_pose.orientation.x=(*pa_iter)->qx;
         obj_pose.orientation.y=(*pa_iter)->qy;
         obj_pose.orientation.z=(*pa_iter)->qz;
         obj_pose.orientation.w=(*pa_iter)->qw;

         obj_pose.position.x=(*pa_iter)->x;
         obj_pose.position.y=(*pa_iter)->y;
         obj_pose.position.z=(*pa_iter)->z;
         cout<<"The score of "<<num<<" th grasp pose is:"<<(*pa_iter)->score<<endl;

         cout<<num<<"th origin target_pose:x:"<<obj_pose.position.x<<",y:"<<obj_pose.position.y<<",z:"<<obj_pose.position.z<<",qw:"<<obj_pose.orientation.w<<",qx:"<<obj_pose.orientation.x<<",qy:"<<obj_pose.orientation.y<<",qz:"<<obj_pose.orientation.z<<endl;

         geometry_msgs::Pose target_pose1=TransformFromObjToRobot(obj_pose, robot_pose);

         cout<<num<<"th target_pose:x:"<<target_pose1.position.x<<",y:"<<target_pose1.position.y<<",z:"<<target_pose1.position.z<<",qw:"<<target_pose1.orientation.w<<",qx:"<<target_pose1.orientation.x<<",qy:"<<target_pose1.orientation.y<<",qz:"<<target_pose1.orientation.z<<endl;

        // //find IK solutions

         robot_state::RobotState start_state(*robot_arm->getCurrentState());
         const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(robot_arm->getName());

         bool success = start_state.setFromIK(joint_model_group, target_pose1);
         cout<<"0th stage find grasp pose:set from ik is success?:"<<success<<endl;
         if(success)
         {
            std::vector<double> joint_values;
            start_state.copyJointGroupPositions(joint_model_group, joint_values);
            const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
            for(std::size_t i = 0; i < joint_names.size(); ++i)
            {
                ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }

            back_joints.clear();

            back_joints.push_back(seed_x-xi);
            back_joints.push_back(seed_y-yi);
            back_joints.push_back(seed_theta-init_joint_of_robot_base);
            back_joints.push_back(0);
            back_joints.push_back(arm_init.joint1);
            back_joints.push_back(arm_init.joint2);
            back_joints.push_back(arm_init.joint3);
            back_joints.push_back(arm_init.joint4);
            back_joints.push_back(arm_init.joint5);
            back_joints.push_back(arm_init.joint6);

            forward_joints.clear();

            forward_joints.push_back(seed_x-xi);
            forward_joints.push_back(seed_y-yi);
            forward_joints.push_back(seed_theta-init_joint_of_robot_base);
            forward_joints.push_back(0);
            forward_joints.push_back(joint_values[0]);
            forward_joints.push_back(joint_values[1]);
            forward_joints.push_back(joint_values[2]);
            forward_joints.push_back(joint_values[3]);
            forward_joints.push_back(joint_values[4]);
            forward_joints.push_back(joint_values[5]);

            ArmJoints joint_arm(joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]);
            BaseJoints bj(seed_x, seed_y, seed_theta);
            current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
            current_robot_state.setVariablePositions( joint_names_0, forward_joints);
            Is_Colliding = current_scene->isStateColliding( current_robot_state );
            cout<<"0th stage find grasp pose:is colliding?:"<<Is_Colliding<<endl;

            cout<<"Is Colliding?"<<Is_Colliding<<",middle_x:"<<seed_x<<",middle_y:"<<seed_y<<"middle_theta:"<<seed_theta<<endl;
            if(!Is_Colliding)
            {
                 double a = 0;
                 double b = ArmEvaluate(joint_arm);
                 double c = GraspEvaluate(obj_pose, bj);
                 double d = (*pa_iter)->score;
                 middle_val = a - w1*b + w2*c + w3*d;
                 cout<<"***********ArmEvaluate:"<<a<<",Score:"<<c<<",GraspEvaluate:"<<b<<endl;

                if(middle_val>fval)
                {
                    index_chosen_grasp=num;
                    cout<<"---------------------index_chosen_grasp:"<<index_chosen_grasp<<endl;
                    fval=middle_val;                   
                    joint_arm_last.setJoint(joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]);
                }
            }



        }
         num++;
    }

    index_chosen_grasp_next = index_chosen_grasp;
    for(int i=0;i<num_fi_iterate;i++)
    {
        middle_x = chosen_mobile.x;
        middle_y = chosen_mobile.y;
        middle_theta = chosen_mobile.theta;

        for(int j=0;j<num_iter_base_2;j++)
        {
            double theta_rand = range_alpha*rand01()-range_alpha/2;
            double radius_rand = range_iterate_r*rand01();
            chosen_mobile_next.x=chosen_mobile.x+radius_rand*cos(theta_rand);
            chosen_mobile_next.y=chosen_mobile.y+radius_rand*sin(theta_rand);
            chosen_mobile_next.theta=chosen_mobile.theta+range_iterate_theta*rand01()-range_iterate_theta/2;

            back_joints.clear();

            back_joints.push_back(chosen_mobile.x-xi);
            back_joints.push_back(chosen_mobile.y-yi);
            back_joints.push_back(chosen_mobile.theta-init_joint_of_robot_base);
            back_joints.push_back(0);
            back_joints.push_back(arm_init.joint1);
            back_joints.push_back(arm_init.joint2);
            back_joints.push_back(arm_init.joint3);
            back_joints.push_back(arm_init.joint4);
            back_joints.push_back(arm_init.joint5);
            back_joints.push_back(arm_init.joint6);

            forward_joints.clear();

            forward_joints.push_back(chosen_mobile_next.x-xi);
            forward_joints.push_back(chosen_mobile_next.y-yi);
            forward_joints.push_back(chosen_mobile_next.theta-init_joint_of_robot_base);
            forward_joints.push_back(0);
            forward_joints.push_back(arm_init.joint1);
            forward_joints.push_back(arm_init.joint2);
            forward_joints.push_back(arm_init.joint3);
            forward_joints.push_back(arm_init.joint4);
            forward_joints.push_back(arm_init.joint5);
            forward_joints.push_back(arm_init.joint6);

            current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
            current_robot_state.setVariablePositions( joint_names_0, forward_joints);
            Is_Colliding = current_scene->isStateColliding( current_robot_state );

            if(!Is_Colliding)
            {
                geometry_msgs::Pose robot_pose;
                robot_pose.orientation.x=0;
                robot_pose.orientation.y=0;
                robot_pose.orientation.z=sin((chosen_mobile_next.theta-init_joint_of_robot_base)/2);//0.3127;
                robot_pose.orientation.w=cos((chosen_mobile_next.theta-init_joint_of_robot_base)/2);

                robot_pose.position.x=chosen_mobile_next.x;
                robot_pose.position.y=chosen_mobile_next.y;
                robot_pose.position.z=0;

                //go to grasp the box
                 geometry_msgs::Pose obj_pose;
                 obj_pose.orientation.x=pa_grasps[index_chosen_grasp]->qx;
                 obj_pose.orientation.y=pa_grasps[index_chosen_grasp]->qy;
                 obj_pose.orientation.z=pa_grasps[index_chosen_grasp]->qz;
                 obj_pose.orientation.w=pa_grasps[index_chosen_grasp]->qw;

                 obj_pose.position.x=pa_grasps[index_chosen_grasp]->x;
                 obj_pose.position.y=pa_grasps[index_chosen_grasp]->y;
                 obj_pose.position.z=pa_grasps[index_chosen_grasp]->z;

                 geometry_msgs::Pose target_pose1=TransformFromObjToRobot(obj_pose, robot_pose);
                 cout<<index_chosen_grasp<<"th target_pose to the"<<j<<"th base is:x:"<<target_pose1.position.x<<",y:"<<target_pose1.position.y<<",z:"<<target_pose1.position.z<<",qw:"<<target_pose1.orientation.w<<",qx:"<<target_pose1.orientation.x<<",qy:"<<target_pose1.orientation.y<<",qz:"<<target_pose1.orientation.z<<endl;


                // //find IK solutions

                 robot_state::RobotState start_state(*robot_arm->getCurrentState());
                 const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(robot_arm->getName());
                 bool success = start_state.setFromIK(joint_model_group, target_pose1);
                 cout<<i+1<<"th stage find mobile pose:set from ik is success?:"<<success<<endl;
                 std::vector<double> joint_values;
                 start_state.copyJointGroupPositions(joint_model_group, joint_values);
                 if(success)
                 {
                     const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
                     for(std::size_t i = 0; i < joint_names.size(); ++i)
                     {
                         ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
                     }
                     back_joints.clear();

                     back_joints.push_back(chosen_mobile.x-xi);
                     back_joints.push_back(chosen_mobile.y-yi);
                     back_joints.push_back(chosen_mobile.theta-init_joint_of_robot_base);
                     back_joints.push_back(0);
                     back_joints.push_back(arm_init.joint1);
                     back_joints.push_back(arm_init.joint2);
                     back_joints.push_back(arm_init.joint3);
                     back_joints.push_back(arm_init.joint4);
                     back_joints.push_back(arm_init.joint5);
                     back_joints.push_back(arm_init.joint6);

                     forward_joints.clear();

                     forward_joints.push_back(chosen_mobile_next.x-xi);
                     forward_joints.push_back(chosen_mobile_next.y-yi);
                     forward_joints.push_back(chosen_mobile_next.theta-init_joint_of_robot_base);
                     forward_joints.push_back(0);
                     forward_joints.push_back(joint_values[0]);
                     forward_joints.push_back(joint_values[1]);
                     forward_joints.push_back(joint_values[2]);
                     forward_joints.push_back(joint_values[3]);
                     forward_joints.push_back(joint_values[4]);
                     forward_joints.push_back(joint_values[5]);
                     ArmJoints joint_arm(joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]);
                     current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
                     current_robot_state.setVariablePositions( joint_names_0, forward_joints);
                     Is_Colliding = current_scene->isStateColliding( current_robot_state );
                     cout<<i+1<<"th stage find mobile pose:is colliding?:"<<Is_Colliding<<endl;

                     cout<<"Is Colliding?"<<Is_Colliding<<",middle_x:"<<chosen_mobile_next.x<<",middle_y:"<<chosen_mobile_next.y<<"middle_theta:"<<chosen_mobile_next.theta<<endl;


                     if(!Is_Colliding)
                     {
                         double a = MobileEvaluate(chosen_mobile_next.x,chosen_mobile_next.y,chosen_mobile_next.theta);
                         double b = ArmEvaluate(joint_arm);
                         double c = GraspEvaluate(obj_pose, chosen_mobile_next);
                         double d = pa_grasps[index_chosen_grasp]->score;
                         middle_val = a - w1*b + w2*c + w3*d;
                         cout<<"**********Grasp Changed**Val:"<<middle_val<<",MobileEvaluate:"<<a<<",ArmEvaluate:"<<b<<",GraspEvaluate:"<<c<<",Score:"<<d<<",Which mobile_x:"<<chosen_mobile_next.x<<",mobile_y:"<<chosen_mobile_next.y<<",mobile_theta:"<<chosen_mobile_next.theta<<endl;

                         if(middle_val>ival)
                         {
                            middle_x = chosen_mobile_next.x;
                            middle_y = chosen_mobile_next.y;
                            middle_theta = chosen_mobile_next.theta;
                            ival = middle_val;                           
                            joint_arm_last.setJoint(joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]);
                         }
                     }
                 }
            }

        }
        chosen_mobile.x = middle_x;
        chosen_mobile.y = middle_y;
        chosen_mobile.theta = middle_theta;



        num=0;
        double DIST;
        for(pa_iter=pa_grasps.begin();pa_iter!=pa_grasps.end();pa_iter++)
        {
            DIST=sqrt((pa_grasps[index_chosen_grasp]->x-(*pa_iter)->x)*(pa_grasps[index_chosen_grasp]->x-(*pa_iter)->x)+(pa_grasps[index_chosen_grasp]->y-(*pa_iter)->y)*(pa_grasps[index_chosen_grasp]->y-(*pa_iter)->y)+(pa_grasps[index_chosen_grasp]->z-(*pa_iter)->z)*(pa_grasps[index_chosen_grasp]->z-(*pa_iter)->z)+(pa_grasps[index_chosen_grasp]->qw-(*pa_iter)->qw)*(pa_grasps[index_chosen_grasp]->qw-(*pa_iter)->qw)+(pa_grasps[index_chosen_grasp]->qx-(*pa_iter)->qx)*(pa_grasps[index_chosen_grasp]->qx-(*pa_iter)->qx)+(pa_grasps[index_chosen_grasp]->qy-(*pa_iter)->qy)*(pa_grasps[index_chosen_grasp]->qy-(*pa_iter)->qy)+(pa_grasps[index_chosen_grasp]->qz-(*pa_iter)->qz)*(pa_grasps[index_chosen_grasp]->qz-(*pa_iter)->qz));
            cout<<"DIST:"<<DIST<<endl;
            if(DIST<range_iterate_grasp)
            {


                geometry_msgs::Pose robot_pose;
                robot_pose.orientation.x=0;
                robot_pose.orientation.y=0;
                robot_pose.orientation.z=sin((chosen_mobile.theta-init_joint_of_robot_base)/2);//0.3127;
                robot_pose.orientation.w=cos((chosen_mobile.theta-init_joint_of_robot_base)/2);

                robot_pose.position.x=chosen_mobile.x;
                robot_pose.position.y=chosen_mobile.y;
                robot_pose.position.z=0;

                //go to grasp the box
                geometry_msgs::Pose obj_pose;
                obj_pose.orientation.x=(*pa_iter)->qx;
                obj_pose.orientation.y=(*pa_iter)->qy;
                obj_pose.orientation.z=(*pa_iter)->qz;
                obj_pose.orientation.w=(*pa_iter)->qw;

                obj_pose.position.x=(*pa_iter)->x;
                obj_pose.position.y=(*pa_iter)->y;
                obj_pose.position.z=(*pa_iter)->z;
//                cout<<"The score of "<<num<<" th grasp pose is:"<<(*pa_iter)->score<<endl;


                 geometry_msgs::Pose target_pose1=TransformFromObjToRobot(obj_pose, robot_pose);
                 //cout<<num<<"th target_pose:x:"<<target_pose1.position.x<<",y:"<<target_pose1.position.y<<",z:"<<target_pose1.position.z<<",qw:"<<target_pose1.orientation.w<<",qx:"<<target_pose1.orientation.x<<",qy:"<<target_pose1.orientation.y<<",qz:"<<target_pose1.orientation.z<<endl;


                // //find IK solutions

                 robot_state::RobotState start_state(*robot_arm->getCurrentState());
                 const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(robot_arm->getName());
                 bool success = start_state.setFromIK(joint_model_group, target_pose1);
                 cout<<i+1<<"th stage find grasp pose:set from ik is success?:"<<success<<endl;
                 if(success)
                 {
                    std::vector<double> joint_values;
                    start_state.copyJointGroupPositions(joint_model_group, joint_values);
                    const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();
                    for(std::size_t i = 0; i < joint_names.size(); ++i)
                    {
                        ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
                    }

                    back_joints.clear();

                    back_joints.push_back(chosen_mobile.x-xi);
                    back_joints.push_back(chosen_mobile.y-yi);
                    back_joints.push_back(chosen_mobile.theta-init_joint_of_robot_base);
                    back_joints.push_back(0);
                    back_joints.push_back(arm_init.joint1);
                    back_joints.push_back(arm_init.joint2);
                    back_joints.push_back(arm_init.joint3);
                    back_joints.push_back(arm_init.joint4);
                    back_joints.push_back(arm_init.joint5);
                    back_joints.push_back(arm_init.joint6);

                    forward_joints.clear();

                    forward_joints.push_back(chosen_mobile.x-xi);
                    forward_joints.push_back(chosen_mobile.y-yi);
                    forward_joints.push_back(chosen_mobile.theta-init_joint_of_robot_base);
                    forward_joints.push_back(0);
                    forward_joints.push_back(joint_values[0]);
                    forward_joints.push_back(joint_values[1]);
                    forward_joints.push_back(joint_values[2]);
                    forward_joints.push_back(joint_values[3]);
                    forward_joints.push_back(joint_values[4]);
                    forward_joints.push_back(joint_values[5]);

                    ArmJoints joint_arm(joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]);
                    current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
                    current_robot_state.setVariablePositions( joint_names_0, forward_joints);
                    Is_Colliding = current_scene->isStateColliding( current_robot_state );

//                    cout<<i+1<<"th stage find grasp pose:is colliding?:"<<Is_Colliding<<endl;
                    cout<<"Is Colliding?"<<Is_Colliding<<",middle_x:"<<chosen_mobile.x<<",middle_y:"<<chosen_mobile.y<<"middle_theta:"<<chosen_mobile.theta<<endl;
                    if(!Is_Colliding)
                    {
                        BaseJoints bj(chosen_mobile.x, chosen_mobile.y, chosen_mobile.theta);
                        double a = MobileEvaluate(chosen_mobile.x, chosen_mobile.y, chosen_mobile.theta);
                        double b = ArmEvaluate(joint_arm);
                        double d = (*pa_iter)->score;
                        double c = GraspEvaluate(obj_pose, bj);
                        middle_val = a - w1*b +w2*c + w3*d;
                        cout<<"**********Grasp Changed****Val:"<<middle_val<<",MobileEvaluate:"<<a<<",ArmEvaluate:"<<b<<",GraspEvaluate:"<<c<<",Score:"<<d<<",Which mobile_x:"<<chosen_mobile.x<<",mobile_y:"<<chosen_mobile.y<<",mobile_theta:"<<chosen_mobile.theta<<endl;
                        if(middle_val>fval)
                        {
                            index_chosen_grasp_next=num;
                            fval=middle_val;
                            joint_arm_last.setJoint(joint_values[0],joint_values[1],joint_values[2],joint_values[3],joint_values[4],joint_values[5]);
                            cout<<"------------the index of best chosen grasp:"<<index_chosen_grasp_next<<",------------index_chosen_grasp:"<<index_chosen_grasp<<endl;
                        }
                    }



                }
            }
            num++;
        }
        index_chosen_grasp = index_chosen_grasp_next;

    }
    if( abs(joint_arm_last.joint1-4.8096) < 1.0 )
    {
        cout<<"##########joint1 of the arm is too big, lets minus 2*M_PI!"<<endl;
        cout<<"##########joint1 old:"<<joint_arm_last.joint1<<endl;
        joint_arm_last.joint1 = joint_arm_last.joint1 - 2*M_PI;
        cout<<"##########joint1 new:"<<joint_arm_last.joint1<<endl;
    }

}

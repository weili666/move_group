#ifndef _GRASPMOBILEPOSEPLAN_H
#define _GRASPMOBILEPOSEPLAN_H
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
#include "grasp_planning_msgs/EvaluateGrasps.h"
#include "grasp_planning_msgs/CallEvaluateGrasps.h"

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


#define M_pi 3.141592653
using namespace std;


class GraspMobilePosePlan
{
public:
    GraspMobilePosePlan(const ros::NodeHandle& node_handle,const BaseJoints& init_base, const BaseJoints& targ_base,const ArmJoints& arm, double init_jo_of_ro_ba,boost::shared_ptr<KDL::ChainFkSolverPos>  jnt_to_pose_solver,planning_scene_monitor::PlanningSceneMonitorPtr sc_,moveit::planning_interface::MoveGroup *ro_,moveit::planning_interface::MoveGroup *ro_arm,moveit::planning_interface::MoveGroup *ro_mobile)
    {
        nh = node_handle;
        base_init = init_base;
        base_targ = targ_base;
        init_joint_of_robot_base = init_jo_of_ro_ba;
        jnt_to_pose_solver_ = jnt_to_pose_solver;
        scene_ = sc_;
        robot_ = ro_;
        robot_mobile = ro_mobile;
        robot_arm = ro_arm;
        arm_init=arm;
        ros::ServiceClient client = nh.serviceClient<grasp_planning_msgs::CallEvaluateGrasps>("call_back_grasp_poses");
        grasp_planning_msgs::CallEvaluateGrasps srv;
        srv.request.begin = true;
        cout<<"begin call!"<<endl;
        client.call(srv);
        num_of_pose_g = srv.response.pose_g.position.size()/15;
        for(int i=0;i<num_of_pose_g-1;i++)
        {
             PositionAttitude* pa=new PositionAttitude(srv.response.pose_g.position[i*15+0],srv.response.pose_g.position[i*15+1],srv.response.pose_g.position[i*15+2],srv.response.pose_g.position[i*15+3],srv.response.pose_g.position[i*15+4],srv.response.pose_g.position[i*15+5],srv.response.pose_g.position[i*15+6],srv.response.pose_g.position[i*15+7],srv.response.pose_g.position[i*15+8],srv.response.pose_g.position[i*15+9],srv.response.pose_g.position[i*15+10],srv.response.pose_g.position[i*15+11],srv.response.pose_g.position[i*15+12],srv.response.pose_g.position[i*15+13],srv.response.pose_g.position[i*15+14]);
             pa_grasps.push_back(pa);
        }

        center_x = srv.response.pose_g.position[(num_of_pose_g-1)*15+1];
        center_y = srv.response.pose_g.position[(num_of_pose_g-1)*15+2];
        center_z = srv.response.pose_g.position[(num_of_pose_g-1)*15+3];
        x_max = srv.response.pose_g.position[(num_of_pose_g-1)*15+8];
        x_min = srv.response.pose_g.position[(num_of_pose_g-1)*15+9];
        y_max = srv.response.pose_g.position[(num_of_pose_g-1)*15+10];
        y_min = srv.response.pose_g.position[(num_of_pose_g-1)*15+11];
        z_max = srv.response.pose_g.position[(num_of_pose_g-1)*15+12];
        z_min = srv.response.pose_g.position[(num_of_pose_g-1)*15+13];

        CreateMobileGraspPose();
        cout<<"seed_x"<<seed_x<<"seed_y"<<seed_y<<"seed_theta:"<<seed_theta<<endl;
        cout<<"center_x:"<<center_x<<",center_y:"<<center_y<<",center_z:"<<center_z<<",center_theta:"<<center_theta<<endl;
        cout<<"index_chosen_grasp:"<<index_chosen_grasp<<endl;
        cout<<"joint_arm_last:"<<joint_arm_last.joint1<<","<<joint_arm_last.joint2<<","<<joint_arm_last.joint3<<","<<joint_arm_last.joint4<<","<<joint_arm_last.joint5<<","<<joint_arm_last.joint6<<endl;
        cout<<"chosen_mobile:"<<chosen_mobile.x<<","<<chosen_mobile.y<<","<<chosen_mobile.theta<<endl;
        cout<<"targ_pa_last:x:"<<Get_Targ_Pa_Last()->x<<",y:"<<Get_Targ_Pa_Last()->y<<",z:"<<Get_Targ_Pa_Last()->z<<",qw:"<<Get_Targ_Pa_Last()->qw<<",qx:"<<Get_Targ_Pa_Last()->qx<<",qy:"<<Get_Targ_Pa_Last()->qy<<",qz:"<<Get_Targ_Pa_Last()->qz<<endl;
    }
    void CreateMobileGraspPose();
    double MobileEvaluate(double x,double y,double theta);
    double ArmEvaluate(const ArmJoints& joint_arm);
    double GraspEvaluate(const geometry_msgs::Pose& gp, const BaseJoints& bj);
    double CalcDistToInitAndTarg(double x,double y);
    int Get_Chosen_Grasp_Index();
    ArmJoints Get_Joint_Arm_Last();
    BaseJoints Get_Chosen_Mobile();
    PositionAttitude* Get_Targ_Pa_Last();
    double Get_Center_X();
    double Get_Center_Y();
    double Get_Center_Z();
    double Get_Center_Theta();
    double Get_Max_X();
    double Get_Min_X();
    double Get_Max_Y();
    double Get_Min_Y();
    double Get_Max_Z();
    double Get_Min_Z();
    geometry_msgs::Pose TransformFromObjToRobot(const geometry_msgs::Pose& obj_pose,const geometry_msgs::Pose& robot_pose);
private:
    ros::NodeHandle nh;
    PositionAttitude gripper_grasp;
    BaseJoints base_init;
    BaseJoints base_grasp;
    BaseJoints base_targ;
    int num_of_pose_g;
    double seed_x,seed_y,seed_theta;
    double center_x,center_y,center_z,x_max,x_min,y_max,y_min,z_max,z_min;
    double center_theta;
    ExtendJoints grasp_combine_joints;
    planning_scene_monitor::PlanningSceneMonitorPtr scene_;
    moveit::planning_interface::MoveGroup *robot_;
    moveit::planning_interface::MoveGroup *robot_mobile;
    moveit::planning_interface::MoveGroup *robot_arm;
    double init_joint_of_robot_base;
    vector<PositionAttitude*> pa_grasps;
    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    ArmJoints arm_init;

    int index_chosen_grasp;
    ArmJoints joint_arm_last;
    BaseJoints chosen_mobile;
    PositionAttitude targ_pa_last;

};


#endif

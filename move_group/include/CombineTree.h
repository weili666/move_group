#ifndef _COMBINETREE_H
#define _COMBINETREE_H
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
//#include <thread>

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
#include"StartBaseTreeNode.h"
#include"StartBaseTree.h"
#include"PositionAttitude.h"
#include"BaseTrajectory.h"
#include"BackHandTree.h"
#include"Commen.h"

#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>

#define M_pi 3.141592653
using namespace std;

class CombineTree
{
public:
    CombineTree(const ros::NodeHandle& node_handle,const PositionAttitude& targ_pa,const BaseJoints& targ_base,const ArmJoints& targ_arm_last,const BiJoints &init_bi_joints_,double portion_,double init_jo_of_ro_ba,boost::shared_ptr<KDL::ChainFkSolverPos>  jnt_to_pose_solver,planning_scene_monitor::PlanningSceneMonitorPtr sc_,moveit::planning_interface::MoveGroup *ro_,moveit::planning_interface::MoveGroup *arm_,const BaseJoints& init_base)
    {
        double l_h=0.116;
        isReady = false;
        double default_init_x=init_bi_joints_.x;
        double default_init_y=init_bi_joints_.y;
        init_bi_joints=init_bi_joints_;
        target=targ_pa;
        portion=portion_;
        init_joint_of_robot_base=init_jo_of_ro_ba;
        nh=node_handle;
        scene_=sc_;
        robot_=ro_;
        robot_arm=arm_;
        ros::Rate rate(100);
        init.x=init_base.x-default_init_x;init.y=init_base.y-default_init_y;init.theta=init_base.theta-init_joint_of_robot_base;
        targ.setBase(targ_base.x-default_init_x,targ_base.y-default_init_y,targ_base.theta-init_joint_of_robot_base);
        targ_arm_last_=targ_arm_last;


        cout<<"hello world!!"<<endl;

        BaseTrajectory traj_base2(init,targ,init_joint_of_robot_base,scene_,robot_);
        traj_base=traj_base2;
        jnt_to_pose_solver_=jnt_to_pose_solver;

        traj_joints=traj_base.CurveAStar();

        cout<<"traj_joints.size"<<traj_joints.size()<<endl;

        traj_nodes=traj_base.getChain();

        cout<<"traj_nodes.size"<<traj_nodes.size()<<endl;

        Whole_joints_of_all_nodes.resize(traj_joints.size());

    }
    virtual vector<BiJoints> BuildTheWholeTree(const KDL::Tree &kdl_tree);
    geometry_msgs::Pose TransformFromObjToRobot(const geometry_msgs::Pose& obj_pose,const geometry_msgs::Pose& robot_pose);
    int getTrajSize(){return traj_joints.size();}


protected:
    bool isReady ;
    ros::NodeHandle nh;
    PositionAttitude target;
    ArmJoints targ_arm_last_;
    ArmJoints targ_arm_last_two;
    BaseJoints init;
    BiJoints init_bi_joints;
    BaseJoints targ;
    planning_scene_monitor::PlanningSceneMonitorPtr scene_;
    moveit::planning_interface::MoveGroup *robot_;
    moveit::planning_interface::MoveGroup *robot_arm;
    double init_joint_of_robot_base;
    BackHandTree bht;
    vector<BaseJoints> traj_joints;
    vector<PositionAttitude> pa_grasps;
    BaseTrajectory traj_base;
    vector<BiJoints> Whole_joints_of_all_nodes;
    vector<StartBaseTreeNode *> traj_nodes;
    double portion;
    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    sensor_msgs::JointState pose_grasps;
};

#endif

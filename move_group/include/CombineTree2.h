#ifndef _COMBINETREE2_H
#define _COMBINETREE2_H
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
#include"CombineTree.h"

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

class CombineTree2:public CombineTree
{
public:
    CombineTree2(const ros::NodeHandle& node_handle,const PositionAttitude& targ_pa,const BaseJoints& targ_base,const ArmJoints& targ_arm_last,const BiJoints &init_bi_joints_,double portion_,double init_jo_of_ro_ba,boost::shared_ptr<KDL::ChainFkSolverPos>  jnt_to_pose_solver,planning_scene_monitor::PlanningSceneMonitorPtr sc_,moveit::planning_interface::MoveGroup *ro_,moveit::planning_interface::MoveGroup *arm_,const BaseJoints& init_base,const ArmJoints& targ_arm_init,double portion_2):CombineTree(node_handle,targ_pa,targ_base,targ_arm_last,init_bi_joints_,portion_,init_jo_of_ro_ba,jnt_to_pose_solver,sc_,ro_,arm_,init_base)
    {
        targ_arm_init_=targ_arm_init;
        portion_i = portion_2;
    }
    vector<BiJoints> BuildTheWholeTree();
private:
    ArmJoints targ_arm_init_;
    double portion_i;
};
#endif

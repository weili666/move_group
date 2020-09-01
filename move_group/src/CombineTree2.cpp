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
#include"ExtendJoints.h"
#include"StartTreeNode.h"
#include"StartBaseTreeNode.h"
#include"StartBaseTree.h"
#include"PositionAttitude.h"
#include"BaseTrajectory.h"
#include"BackHandTree.h"
#include"CombineTree.h"
#include"CombineTree2.h"

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

vector<BiJoints> CombineTree2::BuildTheWholeTree()
{
    KDL::JntArray  q_;
    KDL::Frame     x_;
    double heigth_of_gripper;
    q_.resize(6);
    q_(0)=init_bi_joints.joint2;q_(1)=init_bi_joints.joint3;q_(2)=init_bi_joints.joint4;
    q_(3)=init_bi_joints.joint5;q_(4)=init_bi_joints.joint6;q_(5)=init_bi_joints.joint7;
    jnt_to_pose_solver_->JntToCart(q_, x_);
    heigth_of_gripper = x_.p.z();

    cout<<" forward kinematics success!!"<<endl;
    int start_idx=int(portion*traj_joints.size());
    int iend_idx=int(portion_i*traj_joints.size());
    for(int i=iend_idx;i<=start_idx;i++)
    {
        Whole_joints_of_all_nodes[i].x=traj_joints[i].x;
        Whole_joints_of_all_nodes[i].y=traj_joints[i].y;
        Whole_joints_of_all_nodes[i].base_joint=traj_joints[i].theta;
        Whole_joints_of_all_nodes[i].joint1=init_bi_joints.joint1;
        Whole_joints_of_all_nodes[i].joint2=init_bi_joints.joint2;
        Whole_joints_of_all_nodes[i].joint3=init_bi_joints.joint3;
        Whole_joints_of_all_nodes[i].joint4=init_bi_joints.joint4;
        Whole_joints_of_all_nodes[i].joint5=init_bi_joints.joint5;
        Whole_joints_of_all_nodes[i].joint6=init_bi_joints.joint6;
        Whole_joints_of_all_nodes[i].joint7=init_bi_joints.joint7;
        Whole_joints_of_all_nodes[i].finger1=init_bi_joints.finger1;
        Whole_joints_of_all_nodes[i].finger2=init_bi_joints.finger2;
        Whole_joints_of_all_nodes[i].finger3=init_bi_joints.finger3;
    }

    planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    robot_->setStartState(current_robot_state);

    double resol_0=0.0;double resol_1=0.6;double resol_2=0.6;double resol_3=0.6;double resol_4=0.6;double resol_5=0.6;double resol_6=0.6;

    double min_target_distance=10.0;
    double lase_distance_thread=2.5;
    BackTreeNode *s=new BackTreeNode;
    vector<BiJoints> Whole_joints_of_all_nodes_last;

    int NUM_CIR=100;
    double last_distance;
    bool jump=0;
    bool Is_Colliding=0;
    int k;
    int num_of_retry=5;
    int calc_collision;
    double theta_support;
    double n_proportion=3;
    double relax_law = 1.3;

    for(int i=0;i<NUM_CIR;i++)
    {


        //------------------------------------------------------------------
        Whole_joints_of_all_nodes[0].x=traj_joints[0].x;
        Whole_joints_of_all_nodes[0].y=traj_joints[0].y;
        Whole_joints_of_all_nodes[0].base_joint=traj_joints[0].theta;
        Whole_joints_of_all_nodes[0].joint1=init_bi_joints.joint1;
        Whole_joints_of_all_nodes[0].joint2=targ_arm_init_.joint1;
        Whole_joints_of_all_nodes[0].joint3=targ_arm_init_.joint2;
        Whole_joints_of_all_nodes[0].joint4=targ_arm_init_.joint3;
        Whole_joints_of_all_nodes[0].joint5=targ_arm_init_.joint4;
        Whole_joints_of_all_nodes[0].joint6=targ_arm_init_.joint5;
        Whole_joints_of_all_nodes[0].joint7=targ_arm_init_.joint6;

        for(k=1;k<iend_idx;k++)
        {
            calc_collision=0;
            for(int retry=0;retry<num_of_retry;retry++)
            {
                vector<double> forward_joints;
                vector<double> back_joints;
                    theta_support=Whole_joints_of_all_nodes[k-1].joint1+resol_0*rand01()-resol_0/2;
                    q_(0)=Whole_joints_of_all_nodes[k-1].joint2+relax_law*(2*n_proportion/(n_proportion-2))*((init_bi_joints.joint2-Whole_joints_of_all_nodes[k-1].joint2)/(iend_idx-k))*(rand01()-(1/n_proportion));//resol_1*rand01()-resol_1/2;
                    q_(1)=Whole_joints_of_all_nodes[k-1].joint3+relax_law*(2*n_proportion/(n_proportion-2))*((init_bi_joints.joint3-Whole_joints_of_all_nodes[k-1].joint3)/(iend_idx-k))*(rand01()-(1/n_proportion));
                    q_(2)=Whole_joints_of_all_nodes[k-1].joint4+relax_law*(2*n_proportion/(n_proportion-2))*((init_bi_joints.joint4-Whole_joints_of_all_nodes[k-1].joint4)/(iend_idx-k))*(rand01()-(1/n_proportion));
                    q_(3)=Whole_joints_of_all_nodes[k-1].joint5+relax_law*(2*n_proportion/(n_proportion-2))*((init_bi_joints.joint5-Whole_joints_of_all_nodes[k-1].joint5)/(iend_idx-k))*(rand01()-(1/n_proportion));
                    q_(4)=Whole_joints_of_all_nodes[k-1].joint6+relax_law*(2*n_proportion/(n_proportion-2))*((init_bi_joints.joint6-Whole_joints_of_all_nodes[k-1].joint6)/(iend_idx-k))*(rand01()-(1/n_proportion));
                    q_(5)=Whole_joints_of_all_nodes[k-1].joint7+relax_law*(2*n_proportion/(n_proportion-2))*((init_bi_joints.joint7-Whole_joints_of_all_nodes[k-1].joint7)/(iend_idx-k))*(rand01()-(1/n_proportion));

                    jnt_to_pose_solver_->JntToCart(q_, x_);


                    if(k==traj_joints.size()-1)
                    {
                        double distance_between_two_config=sqrt(10*(init_bi_joints.joint2-Whole_joints_of_all_nodes[k-1].joint2)*(init_bi_joints.joint2-Whole_joints_of_all_nodes[k-1].joint2)+
                                                                10*(init_bi_joints.joint3-Whole_joints_of_all_nodes[k-1].joint3)*(init_bi_joints.joint3-Whole_joints_of_all_nodes[k-1].joint3)+
                                                                10*(init_bi_joints.joint4-Whole_joints_of_all_nodes[k-1].joint4)*(init_bi_joints.joint4-Whole_joints_of_all_nodes[k-1].joint4)+
                                                                   (init_bi_joints.joint5-Whole_joints_of_all_nodes[k-1].joint5)*(init_bi_joints.joint5-Whole_joints_of_all_nodes[k-1].joint5)+
                                                                   (init_bi_joints.joint6-Whole_joints_of_all_nodes[k-1].joint6)*(init_bi_joints.joint6-Whole_joints_of_all_nodes[k-1].joint6)+
                                                                   (init_bi_joints.joint7-Whole_joints_of_all_nodes[k-1].joint7)*(init_bi_joints.joint7-Whole_joints_of_all_nodes[k-1].joint7));
                        last_distance=distance_between_two_config;
                        //cout<<"The "<<i+1<<" th Circle has "<<traj_joints.size()<<" steps,and the "<<k+1<<" th step distance between two config:  "<<distance_between_two_config<<endl;
                    }

                        forward_joints.clear();
                        forward_joints.push_back(theta_support);
                        for(int v=0;v<6;v++)
                        {
                            forward_joints.push_back(q_(v));
                        }



                forward_joints.insert(forward_joints.begin(),traj_joints[k].theta);
                forward_joints.insert(forward_joints.begin(),traj_joints[k].y);
                forward_joints.insert(forward_joints.begin(),traj_joints[k].x);
                back_joints.clear();
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].x);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].y);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].base_joint);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint1);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint2);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint3);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint4);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint5);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint6);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint7);
                string word[10]={"world_to_robot_x","world_to_robot_y","world_to_robot_roll","support_to_base","jaco_joint_1","jaco_joint_2","jaco_joint_3","jaco_joint_4","jaco_joint_5","jaco_joint_6"};
                std::vector<std::string> joint_names(word,word+10);
                current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
                current_robot_state.setVariablePositions( joint_names, forward_joints);
                Is_Colliding = current_scene->isStateColliding( current_robot_state );
                if( Is_Colliding )
                {
                    calc_collision++;
                    cout<<"calc_collision:"<<calc_collision<<endl;
                    if( retry < num_of_retry-1 )
                    {continue;}
                }
                else if(( !Is_Colliding )&&( k <= 3 )&&( x_.p.z() < heigth_of_gripper ))
                {
                    calc_collision++;
                    cout<<"calc_collision:"<<calc_collision<<endl;
                    if( retry < num_of_retry-1 )
                    {continue;}
                }
                else
                {
                    Whole_joints_of_all_nodes[k].x=forward_joints[0];
                    Whole_joints_of_all_nodes[k].y=forward_joints[1];
                    Whole_joints_of_all_nodes[k].base_joint=forward_joints[2];
                    Whole_joints_of_all_nodes[k].joint1=forward_joints[3];
                    Whole_joints_of_all_nodes[k].joint2=forward_joints[4];
                    Whole_joints_of_all_nodes[k].joint3=forward_joints[5];
                    Whole_joints_of_all_nodes[k].joint4=forward_joints[6];
                    Whole_joints_of_all_nodes[k].joint5=forward_joints[7];
                    Whole_joints_of_all_nodes[k].joint6=forward_joints[8];
                    Whole_joints_of_all_nodes[k].joint7=forward_joints[9];
                    break;
                }

            if(calc_collision==num_of_retry)
            {
                jump=1;
                break;
            }
            }
        }
        if(jump==1)
        {
            continue;
        }
        else if((last_distance<lase_distance_thread)&&(jump==0))
        {
            break;
        }
    }
        //------------------------------------------------------------------

        //------------------------------------------------------------------
    for(int i=0;i<NUM_CIR;i++)
    {
        for( k=start_idx+1;k<traj_joints.size();k++)
        {
            calc_collision=0;
            for(int retry=0;retry<num_of_retry;retry++)
            {
                vector<double> forward_joints;
                vector<double> back_joints;
                    theta_support=Whole_joints_of_all_nodes[k-1].joint1+resol_0*rand01()-resol_0/2;
                    q_(0)=Whole_joints_of_all_nodes[k-1].joint2+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_.joint1-Whole_joints_of_all_nodes[k-1].joint2)/(traj_joints.size()-k))*(rand01()-(1/n_proportion));//resol_1*rand01()-resol_1/2;
                    q_(1)=Whole_joints_of_all_nodes[k-1].joint3+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_.joint2-Whole_joints_of_all_nodes[k-1].joint3)/(traj_joints.size()-k))*(rand01()-(1/n_proportion));
                    q_(2)=Whole_joints_of_all_nodes[k-1].joint4+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_.joint3-Whole_joints_of_all_nodes[k-1].joint4)/(traj_joints.size()-k))*(rand01()-(1/n_proportion));
                    q_(3)=Whole_joints_of_all_nodes[k-1].joint5+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_.joint4-Whole_joints_of_all_nodes[k-1].joint5)/(traj_joints.size()-k))*(rand01()-(1/n_proportion));
                    q_(4)=Whole_joints_of_all_nodes[k-1].joint6+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_.joint5-Whole_joints_of_all_nodes[k-1].joint6)/(traj_joints.size()-k))*(rand01()-(1/n_proportion));
                    q_(5)=Whole_joints_of_all_nodes[k-1].joint7+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_.joint6-Whole_joints_of_all_nodes[k-1].joint7)/(traj_joints.size()-k))*(rand01()-(1/n_proportion));

                    if(k==traj_joints.size()-1)
                    {
                        double distance_between_two_config=sqrt(10*(targ_arm_last_.joint1-Whole_joints_of_all_nodes[k-1].joint2)*(targ_arm_last_.joint1-Whole_joints_of_all_nodes[k-1].joint2)+
                                                                10*(targ_arm_last_.joint2-Whole_joints_of_all_nodes[k-1].joint3)*(targ_arm_last_.joint2-Whole_joints_of_all_nodes[k-1].joint3)+
                                                                10*(targ_arm_last_.joint3-Whole_joints_of_all_nodes[k-1].joint4)*(targ_arm_last_.joint3-Whole_joints_of_all_nodes[k-1].joint4)+
                                                                   (targ_arm_last_.joint4-Whole_joints_of_all_nodes[k-1].joint5)*(targ_arm_last_.joint4-Whole_joints_of_all_nodes[k-1].joint5)+
                                                                   (targ_arm_last_.joint5-Whole_joints_of_all_nodes[k-1].joint6)*(targ_arm_last_.joint5-Whole_joints_of_all_nodes[k-1].joint6)+
                                                                   (targ_arm_last_.joint6-Whole_joints_of_all_nodes[k-1].joint7)*(targ_arm_last_.joint6-Whole_joints_of_all_nodes[k-1].joint7));
                        last_distance=distance_between_two_config;
                        cout<<"The "<<i+1<<" th Circle has "<<traj_joints.size()<<" steps,and the "<<k+1<<" th step distance between two config:  "<<distance_between_two_config<<endl;
                    }

                        forward_joints.clear();
                        forward_joints.push_back(theta_support);
                        for(int v=0;v<6;v++)
                        {
                            forward_joints.push_back(q_(v));
                        }



                forward_joints.insert(forward_joints.begin(),traj_joints[k].theta);
                forward_joints.insert(forward_joints.begin(),traj_joints[k].y);
                forward_joints.insert(forward_joints.begin(),traj_joints[k].x);
                back_joints.clear();
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].x);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].y);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].base_joint);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint1);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint2);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint3);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint4);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint5);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint6);
                back_joints.push_back(Whole_joints_of_all_nodes[k-1].joint7);
                string word[10]={"world_to_robot_x","world_to_robot_y","world_to_robot_roll","support_to_base","jaco_joint_1","jaco_joint_2","jaco_joint_3","jaco_joint_4","jaco_joint_5","jaco_joint_6"};
                std::vector<std::string> joint_names(word,word+10);
                current_robot_state.copyJointGroupPositions( robot_->getName(), back_joints);
                current_robot_state.setVariablePositions( joint_names, forward_joints);
                Is_Colliding = current_scene->isStateColliding( current_robot_state );
                if(Is_Colliding)
                {
                    calc_collision++;
                    cout<<"calc_collision:"<<calc_collision<<endl;
                    if(retry<num_of_retry-1)
                    {continue;}
                }
                else
                {
                    Whole_joints_of_all_nodes[k].x=forward_joints[0];
                    Whole_joints_of_all_nodes[k].y=forward_joints[1];
                    Whole_joints_of_all_nodes[k].base_joint=forward_joints[2];
                    Whole_joints_of_all_nodes[k].joint1=forward_joints[3];
                    Whole_joints_of_all_nodes[k].joint2=forward_joints[4];
                    Whole_joints_of_all_nodes[k].joint3=forward_joints[5];
                    Whole_joints_of_all_nodes[k].joint4=forward_joints[6];
                    Whole_joints_of_all_nodes[k].joint5=forward_joints[7];
                    Whole_joints_of_all_nodes[k].joint6=forward_joints[8];
                    Whole_joints_of_all_nodes[k].joint7=forward_joints[9];
                    break;
                }

            if(calc_collision==num_of_retry)
            {
                jump=1;
                break;
            }
            }
        }
        //---------------------------------------------------------------
       if(jump==1)
       {
           continue;
       }
       else if((last_distance<lase_distance_thread)&&(jump==0))
       {
           Whole_joints_of_all_nodes[traj_joints.size()-1].joint2=targ_arm_last_.joint1;
           Whole_joints_of_all_nodes[traj_joints.size()-1].joint3=targ_arm_last_.joint2;
           Whole_joints_of_all_nodes[traj_joints.size()-1].joint4=targ_arm_last_.joint3;
           Whole_joints_of_all_nodes[traj_joints.size()-1].joint5=targ_arm_last_.joint4;
           Whole_joints_of_all_nodes[traj_joints.size()-1].joint6=targ_arm_last_.joint5;
           Whole_joints_of_all_nodes[traj_joints.size()-1].joint7=targ_arm_last_.joint6;
           Whole_joints_of_all_nodes_last.clear();
           Whole_joints_of_all_nodes_last.assign(Whole_joints_of_all_nodes.begin(),Whole_joints_of_all_nodes.end());
           cout<<"min_distance:"<<min_target_distance<<endl;
           break;
       }
    }

    //---------------------------------spline-----------------------------------//
//    for(int time=0;time<5;time++)
//   {
//        vector<BiJoints> curve;
//        curve.assign(Whole_joints_of_all_nodes_last.begin(),Whole_joints_of_all_nodes_last.end());
//        for(int i=1;i<Whole_joints_of_all_nodes_last.size()-1;i++)
//        {
//        Whole_joints_of_all_nodes_last[i].joint2=(1.0/6.0)*curve[i-1].joint2+(2.0/3.0)*curve[i].joint2+(1.0/6.0)*curve[i+1].joint2;
//        Whole_joints_of_all_nodes_last[i].joint3=(1.0/6.0)*curve[i-1].joint3+(2.0/3.0)*curve[i].joint3+(1.0/6.0)*curve[i+1].joint3;
//        Whole_joints_of_all_nodes_last[i].joint4=(1.0/6.0)*curve[i-1].joint4+(2.0/3.0)*curve[i].joint4+(1.0/6.0)*curve[i+1].joint4;
//        Whole_joints_of_all_nodes_last[i].joint5=(1.0/6.0)*curve[i-1].joint5+(2.0/3.0)*curve[i].joint5+(1.0/6.0)*curve[i+1].joint5;
//        Whole_joints_of_all_nodes_last[i].joint6=(1.0/6.0)*curve[i-1].joint6+(2.0/3.0)*curve[i].joint6+(1.0/6.0)*curve[i+1].joint6;
//        Whole_joints_of_all_nodes_last[i].joint7=(1.0/6.0)*curve[i-1].joint7+(2.0/3.0)*curve[i].joint7+(1.0/6.0)*curve[i+1].joint7;
//        }
//    }
    delete s;
    cout<<"The last distance between grasp and the target is:"<<min_target_distance<<endl;
    return Whole_joints_of_all_nodes_last;
}

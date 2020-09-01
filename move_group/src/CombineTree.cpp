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


#include <opencv2/core/eigen.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#define M_pi 3.141592653
using namespace std;

geometry_msgs::Pose CombineTree::TransformFromObjToRobot(const geometry_msgs::Pose& obj_pose,const geometry_msgs::Pose& robot_pose)
{
    Eigen::Quaterniond qo(obj_pose.orientation.w,obj_pose.orientation.x,obj_pose.orientation.y,obj_pose.orientation.z);
    Eigen::Quaterniond qr(robot_pose.orientation.w,robot_pose.orientation.x,robot_pose.orientation.y,robot_pose.orientation.z);
    Eigen::Quaterniond qo_last = qr.inverse()*qo;
    geometry_msgs::Pose relative;

    double theta = 2*asin(robot_pose.orientation.z);
    double xi = init_bi_joints.x;
    double yi = init_bi_joints.y;
    relative.position.x = xi+(obj_pose.position.x-robot_pose.position.x)*cos(theta)+(obj_pose.position.y-robot_pose.position.y)*sin(theta);
    relative.position.y = yi-(obj_pose.position.x-robot_pose.position.x)*sin(theta)+(obj_pose.position.y-robot_pose.position.y)*cos(theta);
    relative.position.z = obj_pose.position.z;
    relative.orientation.w = qo_last.w();
    relative.orientation.x = qo_last.x();
    relative.orientation.y = qo_last.y();
    relative.orientation.z = qo_last.z();
    return relative;

}

vector<BiJoints> CombineTree::BuildTheWholeTree(const KDL::Tree &kdl_tree)
{
    KDL::JntArray  q_;
    KDL::Frame     x_;
    q_.resize(6);
    q_(0)=init_bi_joints.joint2;q_(1)=init_bi_joints.joint3;q_(2)=init_bi_joints.joint4;
    q_(3)=init_bi_joints.joint5;q_(4)=init_bi_joints.joint6;q_(5)=init_bi_joints.joint7;
    jnt_to_pose_solver_->JntToCart(q_, x_);

    int start_idx=int(portion*traj_joints.size());
    //cout<<"the start index:"<<start_idx<<endl;

    Whole_joints_of_all_nodes.clear();
    for(int i=0;i<traj_joints.size();i++)
    {
        //cout<<"haha1"<<endl;
        BiJoints bj;
        bj.x=traj_joints[i].x;
        bj.y=traj_joints[i].y;
        bj.base_joint=traj_joints[i].theta;
        bj.joint1=init_bi_joints.joint1;
        bj.joint2=init_bi_joints.joint2;
        bj.joint3=init_bi_joints.joint3;
        bj.joint4=init_bi_joints.joint4;
        bj.joint5=init_bi_joints.joint5;
        bj.joint6=init_bi_joints.joint6;
        bj.joint7=init_bi_joints.joint7;
        bj.finger1=init_bi_joints.finger1;
        bj.finger2=init_bi_joints.finger2;
        bj.finger3=init_bi_joints.finger3;
        //cout<<"haha2"<<endl;
        Whole_joints_of_all_nodes.push_back(bj);
        //cout<<"haha3"<<endl;
    }

    //cout<<"The size of Whole_joints_of_all_nodes:"<<Whole_joints_of_all_nodes.size()<<endl;

    for(int i=0; i<traj_joints.size()-start_idx-1; i++)
    {
        //cout<<"i="<<i<<",traj_joints.size()-start_idx-1:"<<traj_joints.size()-start_idx-1<<endl;
        BiJoints bj;
        bj.x=(traj_joints[i+start_idx].x+traj_joints[i+start_idx+1].x)/2;
        bj.y=(traj_joints[i+start_idx].y+traj_joints[i+start_idx+1].y)/2;
        bj.base_joint=(traj_joints[i+start_idx].theta+traj_joints[i+start_idx+1].theta)/2;
        bj.joint1=init_bi_joints.joint1;
        bj.joint2=init_bi_joints.joint2;
        bj.joint3=init_bi_joints.joint3;
        bj.joint4=init_bi_joints.joint4;
        bj.joint5=init_bi_joints.joint5;
        bj.joint6=init_bi_joints.joint6;
        bj.joint7=init_bi_joints.joint7;
        Whole_joints_of_all_nodes.insert(Whole_joints_of_all_nodes.begin()+start_idx+2*i+1,bj);
    }
    //cout<<"The size of Whole_joints_of_all_nodes:"<<Whole_joints_of_all_nodes.size()<<",The original size of traj_joints:"<<traj_joints.size()<<endl;

    planning_scene::PlanningScenePtr current_scene = scene_->getPlanningScene();
    robot_state::RobotState current_robot_state = current_scene->getCurrentState();
    robot_->setStartState(current_robot_state);

    double resol_0=0.0;double resol_1=0.6;double resol_2=0.6;double resol_3=0.6;double resol_4=0.6;double resol_5=0.6;double resol_6=0.6;
    bool  found=0;
    double min_target_distance=10.0;
    double lase_distance_thread=2.0;
    BackTreeNode *s=new BackTreeNode;
    vector<BiJoints> Whole_joints_of_all_nodes_last;

    //===================calc targ_arm_last_two=====================//

    double l_h = 0.13;

    geometry_msgs::Pose obj_pose;
    geometry_msgs::Pose targ_pose;
    obj_pose.orientation.w = target.qw;
    obj_pose.orientation.x = target.qx;
    obj_pose.orientation.y = target.qy;
    obj_pose.orientation.z = target.qz;
    targ_pose.orientation.w = target.qw;
    targ_pose.orientation.x = target.qx;
    targ_pose.orientation.y = target.qy;
    targ_pose.orientation.z = target.qz;

    Eigen::Quaterniond orientation( obj_pose.orientation.w, obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z );
    cv::Mat R_orient = cv::Mat(3, 3, CV_32F);
    Eigen::Matrix3d orient = orientation.toRotationMatrix();

    for(int i=0;i<3;i++)
    {
        for(int j=0;j<3;j++)
        {
            R_orient.at<float>(i,j)=orient(i,j);
        }
    }

    obj_pose.position.x = target.x + l_h*R_orient.at<double>(0,2);
    obj_pose.position.y = target.y + l_h*R_orient.at<double>(1,2);
    obj_pose.position.z = target.z + l_h*R_orient.at<double>(2,2);
    targ_pose.position.x = target.x;
    targ_pose.position.y = target.y;
    targ_pose.position.z = target.z;

    geometry_msgs::Pose robot_pose;
    robot_pose.orientation.x = 0;
    robot_pose.orientation.y = 0;
    robot_pose.orientation.z = sin((Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].base_joint-init_joint_of_robot_base)/2);//0.3127;
    robot_pose.orientation.w = cos((Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].base_joint-init_joint_of_robot_base)/2);
    robot_pose.position.x = Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].x;
    robot_pose.position.y = Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].y;
    robot_pose.position.z = 0;


    geometry_msgs::Pose obj_pose_rect = TransformFromObjToRobot(obj_pose, robot_pose);
    geometry_msgs::Pose targ_pose_rect = TransformFromObjToRobot(targ_pose, robot_pose);

    //cout<<"Geometry Pose of Hand:x:"<<obj_pose_rect.position.x<<",y:"<<obj_pose_rect.position.y<<",z:"<<obj_pose_rect.position.z<<",qw:"<<obj_pose_rect.orientation.w<<",qx:"<<obj_pose_rect.orientation.x<<",qy:"<<obj_pose_rect.orientation.y<<",qz:"<<obj_pose_rect.orientation.z<<endl;

    KDL::Chain jaco_chain;
    if(!kdl_tree.getChain("support_link", "jaco_link_hand", jaco_chain))
    {
        std::cout << "Failed to parse the kdl chain" << std::endl;
    }
    boost::shared_ptr<KDL::ChainJntToJacSolver>  jnt_to_jac_solver_;
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(jaco_chain));
    KDL::Jacobian  J_;

    q_(0) = targ_arm_last_.joint1; q_(1) = targ_arm_last_.joint2; q_(2) = targ_arm_last_.joint3;
    q_(3) = targ_arm_last_.joint4; q_(4) = targ_arm_last_.joint5; q_(5) = targ_arm_last_.joint6;
    J_.resize(6);
    jnt_to_jac_solver_->JntToJac(q_,J_);

    cv::Mat delta_x(6, 1, CV_64FC1);
    cv::Mat delta_q(6, 1, CV_64FC1);
    cv::Mat Jacobi (6, 6, CV_64FC1);

    delta_x.at<double>(0,0) = obj_pose_rect.position.x - targ_pose_rect.position.x;
    delta_x.at<double>(1,0) = obj_pose_rect.position.y - targ_pose_rect.position.y;
    delta_x.at<double>(2,0) = obj_pose_rect.position.z - targ_pose_rect.position.z;
    delta_x.at<double>(3,0) = 0;
    delta_x.at<double>(4,0) = 0;
    delta_x.at<double>(5,0) = 0;
    for(int i=0;i<6;i++)
    {
        for(int j=0;j<6;j++)
        {
             Jacobi.at<double>(i,j) = J_.data(i,j);
             //cout<<"Jacobi.at<double>(i,j)"<<Jacobi.at<double>(i,j)<<",J2_.data(i,j)"<<J2_.data(i,j)<<endl;
        }
    }
    delta_q = Jacobi.inv()*delta_x;

    targ_arm_last_two.joint1 = targ_arm_last_.joint1 + delta_q.at<double>(0,0);
    targ_arm_last_two.joint2 = targ_arm_last_.joint2 + delta_q.at<double>(1,0);
    targ_arm_last_two.joint3 = targ_arm_last_.joint3 + delta_q.at<double>(2,0);
    targ_arm_last_two.joint4 = targ_arm_last_.joint4 + delta_q.at<double>(3,0);
    targ_arm_last_two.joint5 = targ_arm_last_.joint5 + delta_q.at<double>(4,0);
    targ_arm_last_two.joint6 = targ_arm_last_.joint6 + delta_q.at<double>(5,0);

    //==============================================================//

    int NUM_CIR=100;
    for(int i=0;i<NUM_CIR;i++)
    {
        double last_distance;
        bool jump=0;
        found=0;

        bool Is_Colliding=0;
        int k;
        int num_of_retry=5;
        int calc_collision;
        double theta_support;
        double n_proportion=4;
        double relax_law = 1.3;

        for( k=start_idx+1;k!=Whole_joints_of_all_nodes.size();k++)
        {
            calc_collision=0;
            for(int retry=0;retry<num_of_retry;retry++)
            {
                vector<double> forward_joints;
                vector<double> back_joints;
                    theta_support=Whole_joints_of_all_nodes[k-1].joint1+resol_0*rand01()-resol_0/2;
                    q_(0)=Whole_joints_of_all_nodes[k-1].joint2+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_two.joint1-Whole_joints_of_all_nodes[k-1].joint2)/(Whole_joints_of_all_nodes.size()-k))*(rand01()-(1/n_proportion));
                    q_(1)=Whole_joints_of_all_nodes[k-1].joint3+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_two.joint2-Whole_joints_of_all_nodes[k-1].joint3)/(Whole_joints_of_all_nodes.size()-k))*(rand01()-(1/n_proportion));
                    q_(2)=Whole_joints_of_all_nodes[k-1].joint4+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_two.joint3-Whole_joints_of_all_nodes[k-1].joint4)/(Whole_joints_of_all_nodes.size()-k))*(rand01()-(1/n_proportion));
                    q_(3)=Whole_joints_of_all_nodes[k-1].joint5+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_two.joint4-Whole_joints_of_all_nodes[k-1].joint5)/(Whole_joints_of_all_nodes.size()-k))*(rand01()-(1/n_proportion));
                    q_(4)=Whole_joints_of_all_nodes[k-1].joint6+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_two.joint5-Whole_joints_of_all_nodes[k-1].joint6)/(Whole_joints_of_all_nodes.size()-k))*(rand01()-(1/n_proportion));
                    q_(5)=Whole_joints_of_all_nodes[k-1].joint7+relax_law*(2*n_proportion/(n_proportion-2))*((targ_arm_last_two.joint6-Whole_joints_of_all_nodes[k-1].joint7)/(Whole_joints_of_all_nodes.size()-k))*(rand01()-(1/n_proportion));

                    if(k==Whole_joints_of_all_nodes.size()-1)
                    {
                        double distance_between_two_config=sqrt(10*(targ_arm_last_two.joint1-Whole_joints_of_all_nodes[k-1].joint2)*(targ_arm_last_two.joint1-Whole_joints_of_all_nodes[k-1].joint2)+
                                                        10*(targ_arm_last_two.joint2-Whole_joints_of_all_nodes[k-1].joint3)*(targ_arm_last_two.joint2-Whole_joints_of_all_nodes[k-1].joint3)+
                                                        10*(targ_arm_last_two.joint3-Whole_joints_of_all_nodes[k-1].joint4)*(targ_arm_last_two.joint3-Whole_joints_of_all_nodes[k-1].joint4)+
                                                           (targ_arm_last_two.joint4-Whole_joints_of_all_nodes[k-1].joint5)*(targ_arm_last_two.joint4-Whole_joints_of_all_nodes[k-1].joint5)+
                                                           (targ_arm_last_two.joint5-Whole_joints_of_all_nodes[k-1].joint6)*(targ_arm_last_two.joint5-Whole_joints_of_all_nodes[k-1].joint6)+
                                                           (targ_arm_last_two.joint6-Whole_joints_of_all_nodes[k-1].joint7)*(targ_arm_last_two.joint6-Whole_joints_of_all_nodes[k-1].joint7));
                        last_distance=distance_between_two_config;
                        //cout<<"The "<<i+1<<" th Circle has "<<Whole_joints_of_all_nodes.size()<<" steps,and the "<<k+1<<" th step distance between two config:  "<<distance_between_two_config<<endl;
                    }

                        forward_joints.clear();
                        forward_joints.push_back(theta_support);
                        for(int v=0;v<6;v++)
                        {
                            forward_joints.push_back(q_(v));
                        }



                forward_joints.insert(forward_joints.begin(),Whole_joints_of_all_nodes[k].base_joint);
                forward_joints.insert(forward_joints.begin(),Whole_joints_of_all_nodes[k].y);
                forward_joints.insert(forward_joints.begin(),Whole_joints_of_all_nodes[k].x);
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
                    //cout<<"calc_collision:"<<calc_collision<<endl;
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
                //cout<<"jump at the "<<k<<" th step."<<endl;
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
           Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].joint2=targ_arm_last_two.joint1;
           Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].joint3=targ_arm_last_two.joint2;
           Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].joint4=targ_arm_last_two.joint3;
           Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].joint5=targ_arm_last_two.joint4;
           Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].joint6=targ_arm_last_two.joint5;
           Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].joint7=targ_arm_last_two.joint6;

           BiJoints bj;
           bj.x = Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].x;
           bj.y = Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].y;
           bj.base_joint = Whole_joints_of_all_nodes[Whole_joints_of_all_nodes.size()-1].base_joint;
           bj.joint1 = 0;
           bj.joint2 = targ_arm_last_.joint1; bj.joint3 = targ_arm_last_.joint2; bj.joint4 = targ_arm_last_.joint3;
           bj.joint5 = targ_arm_last_.joint4; bj.joint6 = targ_arm_last_.joint5; bj.joint7 = targ_arm_last_.joint6;
           Whole_joints_of_all_nodes.push_back(bj);

           Whole_joints_of_all_nodes_last.clear();
           Whole_joints_of_all_nodes_last.assign(Whole_joints_of_all_nodes.begin(),Whole_joints_of_all_nodes.end());
           //cout<<"min_distance:"<<min_target_distance<<endl;
           break;
       }
    }

    delete s;
    cout<<"The last distance between grasp and the target is:"<<min_target_distance<<endl;
    return Whole_joints_of_all_nodes_last;

}

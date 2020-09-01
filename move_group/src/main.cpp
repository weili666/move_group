#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/move_group_interface/move_group.h>

#include <moveit/collision_detection/collision_world.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include "BaseJoints.h"
#include "ConfigJoints.h"
#include "PositionAttitude.h"
#include "BackTreeNode.h"
#include "BackHandTree.h"
#include "StartBaseTreeNode.h"
#include "StartBaseTree.h"
#include "StartTreeNode.h"
#include "BaseTrajectory.h"
#include "Commen.h"
#include "CombineTree.h"
#include "CombineTree2.h"

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
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <GraspMobilePosePlan.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/GripperCommand.h>
#include <control_msgs/GripperCommandAction.h>
#include <move_base_agv_msgs/MoveBaseAGVAction.h>
#include <move_base_agv_msgs/AgvPose.h>
#include <move_base_agv_msgs/Move_Arm.h>

//torque command message related
#include <wpi_jaco_msgs/AngularTorqueCommand.h>
#include <wpi_jaco_msgs/CartesianForceCommand.h>
#include <wpi_jaco_msgs/GetCartesianPosition.h>
#include <wpi_jaco_msgs/GetAngularPosition.h>
#include <wpi_jaco_msgs/EulerToQuaternion.h>
#include <wpi_jaco_msgs/JacoFK.h>
#include <wpi_jaco_msgs/StartForceControl.h>
#include <wpi_jaco_msgs/StopForceControl.h>
#include <wpi_jaco_msgs/SetFingersPositionAction.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#include <time.h>
#include <grasp_planning_msgs/CallPointCloudFiltAndPub.h>
#define LARGE_ACTUATOR_VELOCITY 0.8378 //maximum velocity of large actuator (joints 1-3) (rad/s)
#define SMALL_ACTUATOR_VELOCITY 1.0472 //maximum velocity of small actuator (joints 4-6) (rad/s)
#define TIME_SCALING_FACTOR 3.0 //keep the trajectory at a followable speed

using namespace std;

class parameter_base
{
public:
    parameter_base( actionlib::SimpleActionClient<move_base_agv_msgs::MoveBaseAGVAction> *ac_, const move_base_agv_msgs::MoveBaseAGVGoal &goal_)
    {
        ac = ac_;
        goal = goal_;
        ac->waitForServer();
        cout<<"Begin Move Base!"<<endl;
        ac->sendGoal(goal);
        cout<<"End Move Base!"<<endl;
    }

    actionlib::SimpleActionClient<move_base_agv_msgs::MoveBaseAGVAction> *ac;

    move_base_agv_msgs::MoveBaseAGVGoal goal;
};




class parameter_arm
{
public:
   parameter_arm(moveit::planning_interface::MoveGroup* group_, const moveit::planning_interface::MoveGroup::Plan& plan_,ros::NodeHandle& nh, double time_)
   {
       group = group_;
       plan = plan_;
       time_delay = time_;
       time_end = false;
       n = nh;
       cout<<"Begin Move Arm!"<<endl;
       ros::ServiceServer service = n.advertiseService("move_the_arm", &parameter_arm::execute_arm,this);
       while(time_end == false)
       {
           ros::spinOnce();
       }
       cout<<"End Move Arm!"<<endl;
   }
   parameter_arm(){}
   bool execute_arm(move_base_agv_msgs::Move_Arm::Request &req, move_base_agv_msgs::Move_Arm::Response &res)
   {
       bool _return;
       if(req.can_arm_move == true)
       {
           _return = group->execute(plan);
           res.is_arm_moved = _return;
           time_end = _return;
           ROS_INFO("Visualizing simultanious plan manifold %s",_return?" ":"Failed");
       }
       else
       {
           res.is_arm_moved = false;
           time_end = false;
           ROS_INFO("Visualizing simultanious plan manifold %s",_return?" ":"Failed");
       }
       return res.is_arm_moved;
   }
   moveit::planning_interface::MoveGroup* group;
   moveit::planning_interface::MoveGroup::Plan plan;
   double time_delay;
   ros::NodeHandle n;
   bool time_end;
};



class ArmJointsSaved
{
public:
    ArmJointsSaved(ros::NodeHandle& nh):nh_(nh)
    {
        ros::Subscriber arm_sub = nh_.subscribe("/jaco_arm/joint_states",1,& ArmJointsSaved::ArmSavedCB,this);

        ros::Rate loop_rate(1);
        int num_loop = 1;
        while(num_loop)
        {
            cout<<"hello world!!"<<endl;
            ros::spinOnce();
            loop_rate.sleep();
            num_loop--;
        }
    }
    void ArmSavedCB(const sensor_msgs::JointState& msg);
    ArmJoints armjnts;
private:
    ros::NodeHandle nh_;

};

void ArmJointsSaved::ArmSavedCB(const sensor_msgs::JointState& msg)
{
   double joint[6];

    double a,b,c,d;
    cout<<"accept arm joint"<<endl;

    cout<<msg.position.at(0)<<" "<<msg.position.at(1)<<" "<<msg.position.at(2)<<" "<<msg.position.at(3)<<" "<<msg.position.at(4)<<" "<<msg.position.at(5)<<endl;
    cout<<msg.position[0]<<" "<<msg.position[1]<<" "<<msg.position[2]<<" "<<msg.position[3]<<" "<<msg.position[4]<<" "<<msg.position[5]<<";"<<endl;

    armjnts.joint1 = msg.position[0]; armjnts.joint2 = msg.position[1]; armjnts.joint3 = msg.position[2];
    armjnts.joint4 = msg.position[3]; armjnts.joint5 = msg.position[4]; armjnts.joint6 = msg.position[5];

}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "kinova_agv");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);

    clock_t startTime,endTime1,endTime2,endTime3;

    actionlib::SimpleActionClient<wpi_jaco_msgs::SetFingersPositionAction>  finger_client_("jaco_fingers/finger_positions",true);
    wpi_jaco_msgs::SetFingersPositionGoal goal_f;

    planning_scene_monitor::PlanningSceneMonitorPtr scene_(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    scene_->startStateMonitor();
    scene_->startSceneMonitor();
    scene_->startWorldGeometryMonitor();
    cout<<"scene set !"<<endl;
    moveit::planning_interface::MoveGroup *robot_ = new moveit::planning_interface::MoveGroup("combine_grasp_group");
    moveit::planning_interface::MoveGroup *group = new moveit::planning_interface::MoveGroup("arm_chain_group");
    moveit::planning_interface::MoveGroup *group2 = new moveit::planning_interface::MoveGroup("mobile");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  //使用PlanningSceneInterface类的对象，与世界沟通
    typedef actionlib::SimpleActionClient<move_base_agv_msgs::MoveBaseAGVAction> MoveBaseAGVActionClient;
    MoveBaseAGVActionClient*      ac = new MoveBaseAGVActionClient("move_base_agv",true);
    MoveBaseAGVActionClient*      ac2 = new MoveBaseAGVActionClient("move_base_agv_2", true);

    move_base_agv_msgs::MoveBaseAGVGoal goal;

    //=========================set Xtion Pro Live Camera Collision=========================//
    //------------------------box 1---------------------------//
    moveit_msgs::CollisionObject box;
    box.header.frame_id = "world_frame";
    box.id = "box";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.6;

    geometry_msgs::Pose pose;
    pose.orientation.w = 1;
    pose.position.x = -0.45;
    pose.position.y = -0.30;
    pose.position.z = 1.03;

    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(pose);
    box.operation = box.ADD;
    //--------------------------------------------------------//
    //------------------------box 2---------------------------//
    moveit_msgs::CollisionObject box2;
    box2.header.frame_id = "world_frame";
    box2.id = "box2";
    shape_msgs::SolidPrimitive primitive2;
    primitive2.type = primitive2.BOX;
    primitive2.dimensions.resize(3);
    primitive2.dimensions[0] = 1.1;
    primitive2.dimensions[1] = 0.02;
    primitive2.dimensions[2] = 0.80;

    geometry_msgs::Pose pose2;
    pose2.orientation.w = 1;
    pose2.position.x = 0.238+0.0;
    pose2.position.y = -0.45;
    pose2.position.z = 0.4;

    box2.primitives.push_back(primitive2);
    box2.primitive_poses.push_back(pose2);
    box2.operation = box2.ADD;
    //--------------------------------------------------------//
    //------------------------box 3---------------------------//
    moveit_msgs::CollisionObject box3;
    box3.header.frame_id = "world_frame";
    box3.id = "box3";
    shape_msgs::SolidPrimitive primitive3;
    primitive3.type = primitive3.BOX;
    primitive3.dimensions.resize(3);
    primitive3.dimensions[0] = 0.4;
    primitive3.dimensions[1] = 0.02;
    primitive3.dimensions[2] = 1.00;

    geometry_msgs::Pose pose3;
    pose3.orientation.w = 1;
    pose3.position.x = -0.262+0.0;
    pose3.position.y = -0.45;
    pose3.position.z = 1.30;

    box3.primitives.push_back(primitive3);
    box3.primitive_poses.push_back(pose3);
    box3.operation = box3.ADD;
    //--------------------------------------------------------//
    //------------------------box 4---------------------------//
    moveit_msgs::CollisionObject box4;
    box4.header.frame_id = "world_frame";
    box4.id = "box4";
    shape_msgs::SolidPrimitive primitive4;
    primitive4.type = primitive4.BOX;
    primitive4.dimensions.resize(3);
    primitive4.dimensions[0] = 0.4;
    primitive4.dimensions[1] = 0.02;
    primitive4.dimensions[2] = 1.00;

    geometry_msgs::Pose pose4;
    pose4.orientation.w = 1;
    pose4.position.x = 0.738+0.0;
    pose4.position.y = -0.45;
    pose4.position.z = 1.30;

    box4.primitives.push_back(primitive4);
    box4.primitive_poses.push_back(pose4);
    box4.operation = box4.ADD;
    //--------------------------------------------------------//
    //------------------------box 5---------------------------//
    moveit_msgs::CollisionObject box5;
    box5.header.frame_id = "world_frame";
    box5.id = "box5";
    shape_msgs::SolidPrimitive primitive5;
    primitive5.type = primitive5.BOX;
    primitive5.dimensions.resize(3);
    primitive5.dimensions[0] = 1.40;
    primitive5.dimensions[1] = 0.02;
    primitive5.dimensions[2] = 1.00;

    geometry_msgs::Pose pose5;
    pose5.orientation.w = 1;
    pose5.position.x = 0.238+0.00;
    pose5.position.y = -0.45;
    pose5.position.z = 2.30;

    box5.primitives.push_back(primitive5);
    box5.primitive_poses.push_back(pose5);
    box5.operation = box5.ADD;
    //--------------------------------------------------------//
    //------------------------box 6---------------------------//
    moveit_msgs::CollisionObject box6;
    box6.header.frame_id = "world_frame";
    box6.id = "box6";
    shape_msgs::SolidPrimitive primitive6;
    primitive6.type = primitive6.BOX;
    primitive6.dimensions.resize(3);
    primitive6.dimensions[0] = 0.02;
    primitive6.dimensions[1] = 0.02;
    primitive6.dimensions[2] = 1.00;

    geometry_msgs::Pose pose6;
    pose6.orientation.w = 1;
    pose6.position.x = 0.238+0.0;
    pose6.position.y = -0.45;
    pose6.position.z = 1.30;

    box6.primitives.push_back(primitive6);
    box6.primitive_poses.push_back(pose6);
    box6.operation = box6.ADD;
    //--------------------------------------------------------//
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(box);
    collision_objects.push_back(box2);
    collision_objects.push_back(box3);
    collision_objects.push_back(box4);
    collision_objects.push_back(box5);
    collision_objects.push_back(box6);
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(box);
    planning_scene.world.collision_objects.push_back(box2);
    planning_scene.world.collision_objects.push_back(box3);
    planning_scene.world.collision_objects.push_back(box4);
    planning_scene.world.collision_objects.push_back(box5);
    planning_scene.world.collision_objects.push_back(box6);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene);

    ROS_INFO("Add an object into the world");

    //=====================================================================================//

    //=========================set kdl tree and group========================//
    KDL::Tree kdl_tree;
    cout<<"hello world!!"<<endl;

    string robot_desc_string;
    node_handle.param("robot_description",robot_desc_string,string());
    if(!kdl_parser::treeFromString(robot_desc_string,kdl_tree))
    {
        cout<<"Failed to construct kdl tree"<<endl;
    }

    boost::shared_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
    boost::shared_ptr<KDL::ChainJntToJacSolver>  jnt_to_jac_solver_;
    boost::shared_ptr<KDL::ChainIdSolver>        pose_to_jnt_solver;

    KDL::Chain jaco_chain;
    if(!kdl_tree.getChain("support_link", "jaco_link_hand", jaco_chain))
    {
        std::cout << "Failed to parse the kdl chain" << std::endl;
    }
    boost::shared_ptr<KDL::Chain> kdl_chain_ptr = boost::make_shared<KDL::Chain>(jaco_chain);
    std::cout << "KDL chain has " << kdl_chain_ptr->getNrOfSegments() << " segments and " << kdl_chain_ptr->getNrOfJoints() << " joints." << std::endl;
    std::cout << "Joints: ";
    for (unsigned int i = 0; i < kdl_chain_ptr->getNrOfSegments(); i++)
        std::cout << kdl_chain_ptr->segments.at(i).getJoint().getName() << " ";
    std::cout << std::endl;

    jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(jaco_chain));
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(jaco_chain));

    KDL::JntArray  q_;
    KDL::Frame     x_;

    double a,b,c,d;
    q_.resize(6);
    q_(0)=-1.4735840227881702;q_(1)=2.919063173960516;q_(2)=1.0135101613037494;
    q_(3)=-2.0836849337533323;q_(4)=1.443466866652682;q_(5)=1.3149469735032022;
    jnt_to_pose_solver_->JntToCart(q_, x_);
    cout<<"x:"<<x_.p.x()<<" y:"<<x_.p.y()<<" z:"<<x_.p.z()<<endl;
    x_.M.GetQuaternion(a,b,c,d);
    cout<<"qw:"<<a<<" qx:"<<b<<" qy:"<<c<<" qz:"<<d<<endl;


     moveit::planning_interface::MoveGroup group_all("combine_grasp_group");
     moveit::planning_interface::MoveGroup group_arm("arm_chain_group");

     cout<<"group set !"<<endl;
    //==========================================================================//

    //=======================plan group=======================//

    cout<<"robot set !"<<endl;
    double init_joint_of_robot_base = M_pi/2;
    cout<<"pi used!"<<endl;
    double portion=0.95;
    double portion2=0.70;
    double portion3=0.30;
    //PositionAttitude target(0.16,-0.18,0.85,0.723918,0.0393173,-0.0877361,-0.683154);
    struct BiJoints init_bi_joints={-1.5,-3.432,0,0,-1.4735840227881702,2.919063173960516,1.0135101613037494,-2.0836849337533323,1.443466866652682,1.3149469735032022};
    cout<<"init x:"<<init_bi_joints.x<<" init y:"<<init_bi_joints.y<<" init base theta:"<<init_bi_joints.base_joint<<" init joint1:"<<init_bi_joints.joint1<<" init joint2:"<<init_bi_joints.joint2<<" init joint3:"<<init_bi_joints.joint3<<" init joint4:"<<init_bi_joints.joint4<<" init joint5:"<<init_bi_joints.joint5<<" init joint6:"<<init_bi_joints.joint6<<" init joint7:"<<init_bi_joints.joint7<<" init finger joint1:"<<init_bi_joints.finger1<<" init finger joint2: "<<init_bi_joints.finger2<<" init finger joint3:"<<init_bi_joints.finger3<<endl;
    vector<BiJoints> bi_joints;
    vector<BiJoints> bi_joints2;
    ArmJoints arm(-1.4735840227881702,2.919063173960516,1.0135101613037494,-2.0836849337533323,1.443466866652682,1.3149469735032022);
    BaseJoints init_base(-1.5,-3.432,M_PI/2);
    BaseJoints targ_base(1,-3.0,-M_PI/2);
    ArmJoints arm_put(0.0676,2.4003,2.7180,2.6121,0.8472,0.0000);
    cout<<"begin plan!!"<<endl;

    startTime = clock();
    GraspMobilePosePlan gmpp( node_handle,init_base,targ_base,arm,init_joint_of_robot_base,jnt_to_pose_solver_,scene_,robot_,group,group2 );
    endTime1 = clock();
    //=========================set Target Collision=========================//
    double delta_objx,delta_objy,delta_objz,center_objx,center_objy,center_objz;
    delta_objx = gmpp.Get_Max_X() - gmpp.Get_Min_X();
    delta_objy = gmpp.Get_Max_Y() - gmpp.Get_Min_Y();
    delta_objz = gmpp.Get_Max_Z() - gmpp.Get_Min_Z();
    double R_obj = std::max(delta_objx,delta_objy)/2.0;
    center_objx = gmpp.Get_Min_X() + R_obj;
    center_objy = gmpp.Get_Min_Y() + R_obj;
    center_objz = gmpp.Get_Center_Z();

    ros::Duration sleep_time(2.0);
    sleep_time.sleep();

    moveit_msgs::AttachedCollisionObject targ;
    targ.link_name = "world_frame";
    targ.object.header.frame_id = "world_frame";
    targ.object.id = "target";
    shape_msgs::SolidPrimitive primitivet;
    primitivet.type = primitivet.CYLINDER;
    primitivet.dimensions.resize(3);
    primitivet.dimensions[0] = delta_objz-0.02;
    primitivet.dimensions[1] = R_obj-0.005;

    geometry_msgs::Pose poset;
    poset.orientation.w = 1;
    poset.position.x = center_objx;
    poset.position.y = center_objy;
    poset.position.z = center_objz;

    targ.object.primitives.push_back(primitivet);
    targ.object.primitive_poses.push_back(poset);
    targ.object.operation = targ.object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects_t;
    collision_objects_t.push_back(targ.object);
    moveit_msgs::PlanningScene planning_scene_t;
    planning_scene_t.world.collision_objects.push_back(targ.object);
    planning_scene_t.is_diff = true;
    planning_scene_interface.addCollisionObjects(collision_objects_t);
    planning_scene_diff_publisher.publish(planning_scene_t);

    sleep_time.sleep();

    ROS_INFO("Add an object into the world");

    getchar();
    //=====================================================================================//

    CombineTree   comb_tree( node_handle, *gmpp.Get_Targ_Pa_Last(), gmpp.Get_Chosen_Mobile(), gmpp.Get_Joint_Arm_Last(), init_bi_joints, portion , init_joint_of_robot_base, jnt_to_pose_solver_, scene_, robot_,group, init_base                                                    );
    endTime2 = clock();
    int start_arm = int(portion*comb_tree.getTrajSize());
    cout<<"start_arm:"<<start_arm<<endl;
    getchar();
    bi_joints=comb_tree.BuildTheWholeTree(kdl_tree);
    endTime3 = clock();
    cout<<"Totle Time:"<<(double)(endTime3-startTime)/CLOCKS_PER_SEC<<"s"<<endl;
    cout<<"Time of generate grasp:"<<(double)(endTime1-startTime)/CLOCKS_PER_SEC<<"s"<<endl;
    cout<<"Time of generate A* curve path:"<<(double)(endTime2-endTime1)/CLOCKS_PER_SEC<<"s"<<endl;
    cout<<"Time of generate bi-path of arm and agv:"<<(double)(endTime3-endTime2)/CLOCKS_PER_SEC<<"s"<<endl;
    cout<<"The size of 1st step:"<<bi_joints.size()<<endl;

    //==========================================================================//


 //**************************** Simulate Move *******************************//
    //=====================after plan and public======================//

    moveit_msgs::RobotTrajectory trajectory_msg_s;
    trajectory_msgs::JointTrajectory strajectory =  trajectory_msg_s.joint_trajectory;
    strajectory.points.resize(bi_joints.size());
    strajectory.joint_names.push_back("world_to_robot_x");
    strajectory.joint_names.push_back("world_to_robot_y");
    strajectory.joint_names.push_back("world_to_robot_roll");
    strajectory.joint_names.push_back("support_to_base");
    strajectory.joint_names.push_back("jaco_joint_1");
    strajectory.joint_names.push_back("jaco_joint_2");
    strajectory.joint_names.push_back("jaco_joint_3");
    strajectory.joint_names.push_back("jaco_joint_4");
    strajectory.joint_names.push_back("jaco_joint_5");
    strajectory.joint_names.push_back("jaco_joint_6");

    int num_of_joints=10;
    int i=0;

    double tx,ty,tz;
    for(i=0;i< bi_joints.size();i++)
    {
        strajectory.points[i].positions.resize(num_of_joints);
        strajectory.points[i].velocities.resize(num_of_joints);
        strajectory.points[i].accelerations.resize(num_of_joints);

        strajectory.points[i].positions[0]= bi_joints[i].x;
        strajectory.points[i].velocities[0]=0.001;
        strajectory.points[i].accelerations[0]=0.005;

        strajectory.points[i].positions[1]=bi_joints[i].y;
        strajectory.points[i].velocities[1]=0.001;
        strajectory.points[i].accelerations[1]=0.005;

        strajectory.points[i].positions[2]=bi_joints[i].base_joint;
        strajectory.points[i].velocities[2]=0.001;
        strajectory.points[i].accelerations[2]=0.005;

        strajectory.points[i].positions[3]=bi_joints[i].joint1;
        strajectory.points[i].velocities[3]=0.001;
        strajectory.points[i].accelerations[3]=0.005;

        strajectory.points[i].positions[4]=bi_joints[i].joint2;
        strajectory.points[i].velocities[4]=0.001;
        strajectory.points[i].accelerations[4]=0.005;

        strajectory.points[i].positions[5]=bi_joints[i].joint3;
        strajectory.points[i].velocities[5]=0.001;
        strajectory.points[i].accelerations[5]=0.005;

        strajectory.points[i].positions[6]=bi_joints[i].joint4;
        strajectory.points[i].velocities[6]=0.001;
        strajectory.points[i].accelerations[6]=0.005;

        strajectory.points[i].positions[7]=bi_joints[i].joint5;
        strajectory.points[i].velocities[7]=0.001;
        strajectory.points[i].accelerations[7]=0.005;

        strajectory.points[i].positions[8]=bi_joints[i].joint6;
        strajectory.points[i].velocities[8]=0.001;
        strajectory.points[i].accelerations[8]=0.005;

        strajectory.points[i].positions[9]=bi_joints[i].joint7;
        strajectory.points[i].velocities[9]=0.001;
        strajectory.points[i].accelerations[9]=0.005;

        KDL::JntArray  q1_;
        KDL::Frame     x1_;
        q1_.resize(7);
        q1_(0)=bi_joints[i].joint1;q1_(1)=bi_joints[i].joint2;q1_(2)=bi_joints[i].joint3;q1_(3)=bi_joints[i].joint4;
        q1_(4)=bi_joints[i].joint5;q1_(5)=bi_joints[i].joint6;q1_(6)=bi_joints[i].joint7;
        jnt_to_pose_solver_->JntToCart(q1_, x1_);
        tx=bi_joints[i].x+x1_.p.x()*sin(bi_joints[i].base_joint+init_joint_of_robot_base)+x1_.p.y()*cos(bi_joints[i].base_joint+init_joint_of_robot_base)+init_bi_joints.x;
        ty=bi_joints[i].y-x1_.p.x()*cos(bi_joints[i].base_joint+init_joint_of_robot_base)+x1_.p.y()*sin(bi_joints[i].base_joint+init_joint_of_robot_base)+init_bi_joints.y;
        tz=x1_.p.z()+0.3125;
        cout<<"THE END GRASP POSITION IS: X:"<<tx<<" ,Y:"<<ty<<" ,Z:"<<tz<<".This is the "<<i<<" th step. The total number of 1 step is"<<bi_joints.size()<<endl;
    }

    moveit::planning_interface::MoveGroup::Plan mani_plan_s;
    trajectory_msg_s.joint_trajectory=strajectory;
    mani_plan_s.trajectory_=trajectory_msg_s;
    moveit_msgs::DisplayTrajectory display_trajectory;
    display_trajectory.trajectory_start=mani_plan_s.start_state_;
    display_trajectory.trajectory.push_back(mani_plan_s.trajectory_);
    display_publisher.publish(display_trajectory);
    bool _return=group_all.execute(mani_plan_s);
    ROS_INFO("Visualizing simultanious plan manifold %s",_return?" ":"Failed");
    sleep(5.0);

    //================================================================================//

    //remove object from the table
     moveit_msgs::CollisionObject remove_object;
     remove_object.id = "target";
     remove_object.header.frame_id = "world_frame";
     remove_object.operation = remove_object.REMOVE;
     ROS_INFO("Attaching the object to the right wrist and removing it from the world.");
     planning_scene_t.world.collision_objects.clear();
     planning_scene_t.world.collision_objects.push_back(remove_object);
     planning_scene_t.robot_state.attached_collision_objects.push_back(targ);
     planning_scene_diff_publisher.publish(planning_scene_t);

     sleep_time.sleep();



//     //add box to gripper
//     double l_h=0.116;//the length of the hand;
//     moveit_msgs::AttachedCollisionObject attached_object;
//     attached_object.link_name="jaco_link_hand";
//     attached_object.object.header.frame_id="jaco_link_hand";
//     attached_object.object.id="targ2";
//     shape_msgs::SolidPrimitive primitive2;
//     primitive2.type=primitive2.CYLINDER;
//     primitive2.dimensions.resize(3);
//     primitive2.dimensions[0] = delta_objz;
//     primitive2.dimensions[1] = R_obj;


//     geometry_msgs::Pose cylin_pose;
//     cylin_pose.orientation.w=0.707;
//     cylin_pose.orientation.x=0.707;
//     cylin_pose.orientation.y=0;
//     cylin_pose.orientation.z=0;
//     cylin_pose.position.x=0;
//     cylin_pose.position.y=0;
//     cylin_pose.position.z=-l_h-R_obj-0.01;

//     attached_object.object.primitives.push_back(primitive2);
//     attached_object.object.primitive_poses.push_back(cylin_pose);
//     attached_object.object.operation=attached_object.object.ADD;
//     ROS_INFO("Add an object into the world");
//     moveit_msgs::PlanningScene planning_scene_2;
//     planning_scene_2.world.collision_objects.push_back(attached_object.object);
//     planning_scene_2.is_diff=true;
//     planning_scene_diff_publisher.publish(planning_scene_2);
//     sleep_time.sleep();

    //==================================================================================//

    CombineTree2 comb_tree2( node_handle, *gmpp.Get_Targ_Pa_Last(), targ_base,                arm_put,                   init_bi_joints, portion2, init_joint_of_robot_base, jnt_to_pose_solver_, scene_, robot_,group, gmpp.Get_Chosen_Mobile(), gmpp.Get_Joint_Arm_Last(),portion3 );
    bi_joints2=comb_tree2.BuildTheWholeTree();
    cout<<"The size of 2en step:"<<bi_joints2.size()<<endl;
    cout<<"success!!"<<endl;

    //==================================================================================//

    moveit_msgs::RobotTrajectory trajectory_msg_s2;
    trajectory_msgs::JointTrajectory strajectory2 =  trajectory_msg_s2.joint_trajectory;
    strajectory2.points.resize(bi_joints2.size());
    strajectory2.joint_names.push_back("world_to_robot_x");
    strajectory2.joint_names.push_back("world_to_robot_y");
    strajectory2.joint_names.push_back("world_to_robot_roll");
    strajectory2.joint_names.push_back("support_to_base");
    strajectory2.joint_names.push_back("jaco_joint_1");
    strajectory2.joint_names.push_back("jaco_joint_2");
    strajectory2.joint_names.push_back("jaco_joint_3");
    strajectory2.joint_names.push_back("jaco_joint_4");
    strajectory2.joint_names.push_back("jaco_joint_5");
    strajectory2.joint_names.push_back("jaco_joint_6");

    for(i=0;i<bi_joints2.size();i++)
    {
        strajectory2.points[i].positions.resize(num_of_joints);
        strajectory2.points[i].velocities.resize(num_of_joints);
        strajectory2.points[i].accelerations.resize(num_of_joints);

        strajectory2.points[i].positions[0]= bi_joints2[i].x;
        strajectory2.points[i].velocities[0]=0.001;
        strajectory2.points[i].accelerations[0]=0.005;

        strajectory2.points[i].positions[1]=bi_joints2[i].y;
        strajectory2.points[i].velocities[1]=0.001;
        strajectory2.points[i].accelerations[1]=0.005;

        strajectory2.points[i].positions[2]=bi_joints2[i].base_joint;
        strajectory2.points[i].velocities[2]=0.001;
        strajectory2.points[i].accelerations[2]=0.005;

        strajectory2.points[i].positions[3]=bi_joints2[i].joint1;
        strajectory2.points[i].velocities[3]=0.001;
        strajectory2.points[i].accelerations[3]=0.005;

        strajectory2.points[i].positions[4]=bi_joints2[i].joint2;
        strajectory2.points[i].velocities[4]=0.001;
        strajectory2.points[i].accelerations[4]=0.005;

        strajectory2.points[i].positions[5]=bi_joints2[i].joint3;
        strajectory2.points[i].velocities[5]=0.001;
        strajectory2.points[i].accelerations[5]=0.005;

        strajectory2.points[i].positions[6]=bi_joints2[i].joint4;
        strajectory2.points[i].velocities[6]=0.001;
        strajectory2.points[i].accelerations[6]=0.005;

        strajectory2.points[i].positions[7]=bi_joints2[i].joint5;
        strajectory2.points[i].velocities[7]=0.001;
        strajectory2.points[i].accelerations[7]=0.005;

        strajectory2.points[i].positions[8]=bi_joints2[i].joint6;
        strajectory2.points[i].velocities[8]=0.001;
        strajectory2.points[i].accelerations[8]=0.005;

        strajectory2.points[i].positions[9]=bi_joints2[i].joint7;
        strajectory2.points[i].velocities[9]=0.001;
        strajectory2.points[i].accelerations[9]=0.005;

        KDL::JntArray  q1_;
        KDL::Frame     x1_;
        q1_.resize(7);
        q1_(0)=bi_joints2[i].joint1;q1_(1)=bi_joints2[i].joint2;q1_(2)=bi_joints2[i].joint3;q1_(3)=bi_joints2[i].joint4;
        q1_(4)=bi_joints2[i].joint5;q1_(5)=bi_joints2[i].joint6;q1_(6)=bi_joints2[i].joint7;
        jnt_to_pose_solver_->JntToCart(q1_, x1_);
        tx=bi_joints2[i].x+x1_.p.x()*sin(bi_joints2[i].base_joint+init_joint_of_robot_base)+x1_.p.y()*cos(bi_joints2[i].base_joint+init_joint_of_robot_base)+init_bi_joints.x;
        ty=bi_joints2[i].y-x1_.p.x()*cos(bi_joints2[i].base_joint+init_joint_of_robot_base)+x1_.p.y()*sin(bi_joints2[i].base_joint+init_joint_of_robot_base)+init_bi_joints.y;
        tz=x1_.p.z()+0.3125;
        cout<<"THE END GRASP POSITION IS: X:"<<tx<<" ,Y:"<<ty<<" ,Z:"<<tz<<".This is the "<<i<<" th step. The total number of 2 step is "<<bi_joints2.size()<<endl;
    }

    moveit::planning_interface::MoveGroup::Plan mani_plan_s2;
    trajectory_msg_s2.joint_trajectory=strajectory2;
    mani_plan_s2.trajectory_=trajectory_msg_s2;
    moveit_msgs::DisplayTrajectory display_trajectory2;
    display_trajectory2.trajectory_start=mani_plan_s2.start_state_;
    display_trajectory2.trajectory.push_back(mani_plan_s2.trajectory_);
    display_publisher.publish(display_trajectory2);
    bool _return_2 = group_all.execute(mani_plan_s2);
    cout<<"get gmpp.Get_Chosen_Mobile():x:"<<gmpp.Get_Chosen_Mobile().x<<",y:"<<gmpp.Get_Chosen_Mobile().y<<",theta:"<<gmpp.Get_Chosen_Mobile().theta<<endl;
    ROS_INFO("Visualizing simultanious plan2 manifold %s",_return_2?" ":"Failed");
    sleep(5.0);

// //**************************************************************************//


 //****************************** Real Move *********************************//

    //<<<<<<<<<<<<<================================ Move 1 ===============================>>>>>>>>>>>>>>>>>//
    //=====================after plan and public======================//
     //ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
     moveit_msgs::RobotTrajectory trajectory_msg_;
     trajectory_msgs::JointTrajectory trajectory =  trajectory_msg_.joint_trajectory;

     int start_arm1 = int(portion*bi_joints.size());
     trajectory.points.resize(bi_joints.size()-start_arm);
     trajectory.joint_names.push_back("jaco_joint_1");
     trajectory.joint_names.push_back("jaco_joint_2");
     trajectory.joint_names.push_back("jaco_joint_3");
     trajectory.joint_names.push_back("jaco_joint_4");
     trajectory.joint_names.push_back("jaco_joint_5");
     trajectory.joint_names.push_back("jaco_joint_6");

      num_of_joints=6;

     vector<ArmJoints> Whole_joints_of_all_nodes_last;
     for(int i=start_arm;i<bi_joints.size();i++)
     {
        ArmJoints arm;
        arm.joint1 = bi_joints[i].joint2;arm.joint2 = bi_joints[i].joint3;arm.joint3 = bi_joints[i].joint4;
        arm.joint4 = bi_joints[i].joint5;arm.joint5 = bi_joints[i].joint6;arm.joint6 = bi_joints[i].joint7;
        Whole_joints_of_all_nodes_last.push_back(arm);
     }
     vector<ArmJoints> curve;
     for(int time=0;time<6;time++)
     {

         curve.clear();
 //       curve.assign(Whole_joints_of_all_nodes_last.begin(),Whole_joints_of_all_nodes_last.end());
         for(int i = 0; i < Whole_joints_of_all_nodes_last.size()-1; i++) {
             curve.push_back(Whole_joints_of_all_nodes_last[i]);
         }
         for(int i=1;i<Whole_joints_of_all_nodes_last.size()-2;i++)
         {
         Whole_joints_of_all_nodes_last[i].joint1=(1.0/6.0)*curve[i-1].joint1+(2.0/3.0)*curve[i].joint1+(1.0/6.0)*curve[i+1].joint1;
         Whole_joints_of_all_nodes_last[i].joint2=(1.0/6.0)*curve[i-1].joint2+(2.0/3.0)*curve[i].joint2+(1.0/6.0)*curve[i+1].joint2;
         Whole_joints_of_all_nodes_last[i].joint3=(1.0/6.0)*curve[i-1].joint3+(2.0/3.0)*curve[i].joint3+(1.0/6.0)*curve[i+1].joint3;
         Whole_joints_of_all_nodes_last[i].joint4=(1.0/6.0)*curve[i-1].joint4+(2.0/3.0)*curve[i].joint4+(1.0/6.0)*curve[i+1].joint4;
         Whole_joints_of_all_nodes_last[i].joint5=(1.0/6.0)*curve[i-1].joint5+(2.0/3.0)*curve[i].joint5+(1.0/6.0)*curve[i+1].joint5;
         Whole_joints_of_all_nodes_last[i].joint6=(1.0/6.0)*curve[i-1].joint6+(2.0/3.0)*curve[i].joint6+(1.0/6.0)*curve[i+1].joint6;
         }
      }
     for(int i=start_arm;i<bi_joints.size();i++)
     {
         trajectory.points[i-start_arm].positions.resize(num_of_joints);
         trajectory.points[i-start_arm].velocities.resize(num_of_joints);
         trajectory.points[i-start_arm].accelerations.resize(num_of_joints);
         trajectory.points[i-start_arm].positions[0]= Whole_joints_of_all_nodes_last[i-start_arm].joint1;
         trajectory.points[i-start_arm].velocities[0]= 0.001;
         trajectory.points[i-start_arm].accelerations[0]= 0.005;
         trajectory.points[i-start_arm].positions[1]= Whole_joints_of_all_nodes_last[i-start_arm].joint2;
         trajectory.points[i-start_arm].velocities[1]= 0.001;
         trajectory.points[i-start_arm].accelerations[1]= 0.005;
         trajectory.points[i-start_arm].positions[2]= Whole_joints_of_all_nodes_last[i-start_arm].joint3;
         trajectory.points[i-start_arm].velocities[2]= 0.001;
         trajectory.points[i-start_arm].accelerations[2]= 0.005;
         trajectory.points[i-start_arm].positions[3]= Whole_joints_of_all_nodes_last[i-start_arm].joint4;
         trajectory.points[i-start_arm].velocities[3]= 0.001;
         trajectory.points[i-start_arm].accelerations[3]= 0.005;
         trajectory.points[i-start_arm].positions[4]= Whole_joints_of_all_nodes_last[i-start_arm].joint5;
         trajectory.points[i-start_arm].velocities[4]= 0.001;
         trajectory.points[i-start_arm].accelerations[4]= 0.005;
         trajectory.points[i-start_arm].positions[5]= Whole_joints_of_all_nodes_last[i-start_arm].joint6;
         trajectory.points[i-start_arm].velocities[5]= 0.001;
         trajectory.points[i-start_arm].accelerations[5]= 0.005;
         //cout<<"the "<<i-start_arm+1<<"th node,joint1:"<<bi_joints[i].joint2<<",joint2:"<<bi_joints[i].joint3<<",joint3:"<<bi_joints[i].joint4<<",joint4:"<<bi_joints[i].joint5<<",joint5:"<<bi_joints[i].joint6<<",joint6:"<<bi_joints[i].joint7<<endl;
     }


     //=========================pass base message=========================//

     vector<double> timePoints(bi_joints.size());
     double time_step = 0.5;
     double delta_joint = 0.001;
     double accel_joint = 0.018;

     timePoints[0] = 0;
     for(unsigned int i = 1;i <= start_arm;i++)
     {
         if( abs(bi_joints[i].base_joint - bi_joints[i-1].base_joint) > delta_joint )
         {
             timePoints[i] = timePoints[i-1] + sqrt(abs(bi_joints[i].base_joint - bi_joints[i-1].base_joint)/accel_joint);
         }
         else
         {
             if( (( i < bi_joints.size()-1 )&&( abs(bi_joints[i+1].base_joint - bi_joints[i].base_joint) > delta_joint ))||( i == bi_joints.size()-1 ) )
             {
                 timePoints[i] = timePoints[i-1] + 2*time_step;
             }
             else
             {
                 timePoints[i] = timePoints[i-1] + time_step;
             }
         }

     }

     for(unsigned int i =start_arm+1; i < bi_joints.size(); i++)
     {
       float maxTime = 0.0;

       for (unsigned int j = 0; j < num_of_joints; j++)
       {
         //calculate approximate time required to move to the next position
         float time = fabs(trajectory.points[i-start_arm].positions[j] - trajectory.points[i-1-start_arm].positions[j]);
         if (j <= 2)
           time /= LARGE_ACTUATOR_VELOCITY;
         else
           time /= SMALL_ACTUATOR_VELOCITY;

         if (time > maxTime)
           maxTime = time;

       }

       timePoints[i] = timePoints[i - 1] + maxTime * TIME_SCALING_FACTOR;
     }
     for(unsigned int i=0;i<bi_joints.size();i++)
     {
       sensor_msgs::JointState pose_g;
       std_msgs::Bool isready;
       pose_g.position.push_back(bi_joints[i].x);
       pose_g.position.push_back(bi_joints[i].y);
       pose_g.position.push_back(bi_joints[i].base_joint);
       pose_g.position.push_back(bi_joints[i].joint1);

       move_base_agv_msgs::AgvPose agv_pose;
       agv_pose.pose_g = pose_g;
       agv_pose.time = timePoints[i];
       if(i!=start_arm)
       {
           agv_pose.start_arm = false;
       }
       else if (i == start_arm)
       {
           agv_pose.start_arm = true;
       }

       goal.agv_poses.push_back(agv_pose);
     }
     cout<<"//=========================bi_joint_1.base==============================//"<<endl;
     cout<<"the size of bi_joint:"<<bi_joints.size()<<endl;
     for(unsigned int i=0;i<bi_joints.size();i++)
     {
         cout<<"x:"<<bi_joints[i].x<<",y:"<<bi_joints[i].y<<",base_joint:"<<bi_joints[i].base_joint<<endl;
     }

//     cout<<"//=========================TimePoint==============================//"<<endl;
//     for(unsigned int i=0;i<bi_joints.size();i++)
//     {
//         cout<<"time point: "<<timePoints[i]<<" ,the start time of the arm: "<<timePoints[start_arm]<<" ,point index: "<<start_arm<<endl;
//     }
     cout<<"//==========================End Output============================//"<<endl;


     //=================================drive agv and arm at the same time===================================//

     ArmJointsSaved as0(node_handle);

     trajectory.points[0].positions[0] = as0.armjnts.joint1;
     trajectory.points[0].positions[1] = as0.armjnts.joint2;
     trajectory.points[0].positions[2] = as0.armjnts.joint3;
     trajectory.points[0].positions[3] = as0.armjnts.joint4;
     trajectory.points[0].positions[4] = as0.armjnts.joint5;
     trajectory.points[0].positions[5] = as0.armjnts.joint6;


     moveit::planning_interface::MoveGroup::Plan mani_plan;
     trajectory_msg_.joint_trajectory=trajectory;
     mani_plan.trajectory_=trajectory_msg_;

     parameter_base pb(ac,goal);
     parameter_arm  pa(group,mani_plan,node_handle,timePoints[start_arm]);

     //<<<<<<<<<<<<<================================ Move 1 end ===============================>>>>>>>>>>>>>>>>>//



     //=====================grasp plan and public======================//

     if(pa.time_end = true)
     {
        finger_client_.waitForServer();
        goal_f.fingers.finger1 = 30;
        goal_f.fingers.finger2 = 30;
        goal_f.fingers.finger3 = 30;
        finger_client_.sendGoal(goal_f);
        cout<<"Grasp finish!"<<endl;
     }
     //bool finish_before_timeout = finger_client_.waitForResult(ros::Duration(5.0));

     //================================================================//



     //<<<<<<<<<<<<<================================ Move 2 start ===============================>>>>>>>>>>>>>>>>>//
     //=====================after plan and public 2======================//
      moveit_msgs::RobotTrajectory trajectory_msg_2;
      trajectory_msgs::JointTrajectory trajectory2 =  trajectory_msg_2.joint_trajectory;
      start_arm=0;
      trajectory2.points.resize(bi_joints2.size()-start_arm);
      trajectory2.joint_names.push_back("jaco_joint_1");
      trajectory2.joint_names.push_back("jaco_joint_2");
      trajectory2.joint_names.push_back("jaco_joint_3");
      trajectory2.joint_names.push_back("jaco_joint_4");
      trajectory2.joint_names.push_back("jaco_joint_5");
      trajectory2.joint_names.push_back("jaco_joint_6");

      cout<<"hello error 1~"<<endl;
      Whole_joints_of_all_nodes_last.clear();
      for(int i=start_arm;i<bi_joints2.size();i++)
      {
         ArmJoints arm;
         arm.joint1 = bi_joints2[i].joint2;arm.joint2 = bi_joints2[i].joint3;arm.joint3 = bi_joints2[i].joint4;
         arm.joint4 = bi_joints2[i].joint5;arm.joint5 = bi_joints2[i].joint6;arm.joint6 = bi_joints2[i].joint7;
         Whole_joints_of_all_nodes_last.push_back(arm);
      }
      vector<ArmJoints> curve2;
      for(int time=0;time<5;time++)
      {
          cout<<"hello error 2~"<<endl;
          curve2.clear();
          cout<<"hello error 3~"<<endl;
 //           curve.assign(Whole_joints_of_all_nodes_last.begin(),Whole_joints_of_all_nodes_last.end());
          for(int i = 0; i < Whole_joints_of_all_nodes_last.size(); i++) {
              curve2.push_back(Whole_joints_of_all_nodes_last[i]);
          }

          cout<<"hello error 4~"<<endl;
          for(int i=1;i<Whole_joints_of_all_nodes_last.size()-1;i++)
          {
          Whole_joints_of_all_nodes_last[i].joint1=(1.0/6.0)*curve2[i-1].joint1+(2.0/3.0)*curve2[i].joint1+(1.0/6.0)*curve2[i+1].joint1;
          Whole_joints_of_all_nodes_last[i].joint2=(1.0/6.0)*curve2[i-1].joint2+(2.0/3.0)*curve2[i].joint2+(1.0/6.0)*curve2[i+1].joint2;
          Whole_joints_of_all_nodes_last[i].joint3=(1.0/6.0)*curve2[i-1].joint3+(2.0/3.0)*curve2[i].joint3+(1.0/6.0)*curve2[i+1].joint3;
          Whole_joints_of_all_nodes_last[i].joint4=(1.0/6.0)*curve2[i-1].joint4+(2.0/3.0)*curve2[i].joint4+(1.0/6.0)*curve2[i+1].joint4;
          Whole_joints_of_all_nodes_last[i].joint5=(1.0/6.0)*curve2[i-1].joint5+(2.0/3.0)*curve2[i].joint5+(1.0/6.0)*curve2[i+1].joint5;
          Whole_joints_of_all_nodes_last[i].joint6=(1.0/6.0)*curve2[i-1].joint6+(2.0/3.0)*curve2[i].joint6+(1.0/6.0)*curve2[i+1].joint6;
          }
          cout<<"hello error 5~"<<endl;
       }

      for(int i=start_arm;i<bi_joints2.size();i++)
      {
          trajectory2.points[i-start_arm].positions.resize(num_of_joints);
          trajectory2.points[i-start_arm].velocities.resize(num_of_joints);
          trajectory2.points[i-start_arm].accelerations.resize(num_of_joints);
          trajectory2.points[i-start_arm].positions[0]= Whole_joints_of_all_nodes_last[i-start_arm].joint1;
          trajectory2.points[i-start_arm].velocities[0]= 0.001;
          trajectory2.points[i-start_arm].accelerations[0]= 0.005;
          trajectory2.points[i-start_arm].positions[1]= Whole_joints_of_all_nodes_last[i-start_arm].joint2;
          trajectory2.points[i-start_arm].velocities[1]= 0.001;
          trajectory2.points[i-start_arm].accelerations[1]= 0.005;
          trajectory2.points[i-start_arm].positions[2]= Whole_joints_of_all_nodes_last[i-start_arm].joint3;
          trajectory2.points[i-start_arm].velocities[2]= 0.001;
          trajectory2.points[i-start_arm].accelerations[2]= 0.005;
          trajectory2.points[i-start_arm].positions[3]= Whole_joints_of_all_nodes_last[i-start_arm].joint4;
          trajectory2.points[i-start_arm].velocities[3]= 0.001;
          trajectory2.points[i-start_arm].accelerations[3]= 0.005;
          trajectory2.points[i-start_arm].positions[4]= Whole_joints_of_all_nodes_last[i-start_arm].joint5;
          trajectory2.points[i-start_arm].velocities[4]= 0.001;
          trajectory2.points[i-start_arm].accelerations[4]= 0.005;
          trajectory2.points[i-start_arm].positions[5]= Whole_joints_of_all_nodes_last[i-start_arm].joint6;
          trajectory2.points[i-start_arm].velocities[5]= 0.001;
          trajectory2.points[i-start_arm].accelerations[5]= 0.005;
          //cout<<"the "<<i+1<<"th node,joint1:"<<bi_joints2[i].joint2<<",joint2:"<<bi_joints2[i].joint3<<",joint3:"<<bi_joints2[i].joint4<<",joint4:"<<bi_joints2[i].joint5<<",joint5:"<<bi_joints2[i].joint6<<",joint6:"<<bi_joints2[i].joint7<<endl;
      }

     //=========================pass base message 2=========================//

     move_base_agv_msgs::MoveBaseAGVGoal goal2;
     vector<double> timePoints2(bi_joints2.size());

     for(unsigned int i =0;i<=start_arm;i++)
     {
         timePoints2[i]=i*time_step;
     }

     cout<<"trajectory points number:"<<bi_joints2.size()-start_arm<<endl;
     for(unsigned int i = start_arm+1; i < bi_joints2.size(); i++)
     {
       float maxTime = 0.0;

       for (unsigned int j = 0; j < num_of_joints; j++)
       {
         //calculate approximate time required to move to the next position
         float time = fabs(trajectory2.points[i-start_arm].positions[j] - trajectory2.points[i-1-start_arm].positions[j]);
         if (j <= 2)
           time /= LARGE_ACTUATOR_VELOCITY;
         else
           time /= SMALL_ACTUATOR_VELOCITY;

         if (time > maxTime)
           maxTime = time;

       }

       timePoints2[i] = timePoints2[i - 1] + maxTime * TIME_SCALING_FACTOR;
       //cout<<"timePoints2["<<i<<"]"<<timePoints2[i]<<endl;
     }

     for(unsigned int i=0;i<bi_joints2.size();i++)
     {
       sensor_msgs::JointState pose_g;
       std_msgs::Bool isready;
       pose_g.position.push_back(bi_joints2[i].x);
       pose_g.position.push_back(bi_joints2[i].y);
       pose_g.position.push_back(bi_joints2[i].base_joint);
       pose_g.position.push_back(bi_joints2[i].joint1);

       move_base_agv_msgs::AgvPose agv_pose;
       agv_pose.pose_g = pose_g;
       agv_pose.time = timePoints2[i];

       if(i!=start_arm)
       {
           agv_pose.start_arm = false;
       }
       else if (i == start_arm)
       {
           agv_pose.start_arm = true;
       }
       goal2.agv_poses.push_back(agv_pose);
     }

     cout<<"//=========================bi_joint_2.base==============================//"<<endl;
     cout<<"the size of bi_joint2:"<<bi_joints2.size()<<endl;
     for(unsigned int i=0;i<bi_joints2.size();i++)
     {
         cout<<"x:"<<bi_joints2[i].x<<",y:"<<bi_joints2[i].y<<",base_joint:"<<bi_joints2[i].base_joint<<endl;
     }


     //=================================drive agv and arm at the same time 2===================================//

     ArmJointsSaved as(node_handle);

     trajectory2.points[0].positions[0] = as.armjnts.joint1;
     trajectory2.points[0].positions[1] = as.armjnts.joint2;
     trajectory2.points[0].positions[2] = as.armjnts.joint3;
     trajectory2.points[0].positions[3] = as.armjnts.joint4;
     trajectory2.points[0].positions[4] = as.armjnts.joint5;
     trajectory2.points[0].positions[5] = as.armjnts.joint6;


     moveit::planning_interface::MoveGroup::Plan mani_plan2;
     trajectory_msg_2.joint_trajectory=trajectory2;
     mani_plan2.trajectory_=trajectory_msg_2;

     parameter_base pb2(ac2,goal2);
     parameter_arm  pa2(group,mani_plan2,node_handle,timePoints2[start_arm]);

     //<<<<<<<<<<<<<================================ Move 2 end ===============================>>>>>>>>>>>>>>>>>//



     //=====================grasp plan and public 2======================//

     finger_client_.waitForServer();
     goal_f.fingers.finger1 = 0;
     goal_f.fingers.finger2 = 0;
     goal_f.fingers.finger3 = 0;
     finger_client_.sendGoal(goal_f);
     bool finish_before_timeout2 = finger_client_.waitForResult(ros::Duration(5.0));

     //==========================================================================//

 //****************************** Real Move *********************************//

     cout<<"hello world!"<<endl;
    ros::spin();
    return 0;


}





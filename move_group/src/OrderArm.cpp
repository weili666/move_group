#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <moveit/move_group_interface/move_group.h>
#define USE_CAN
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <stdio.h>
#include "CML.h"
#include <unistd.h>
#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_agv_msgs/MoveBaseAGVAction.h>
#include <time.h>
#include "ros/ros.h"
#include "move_base_agv_msgs/Move_Arm.h"

using namespace std;

class OrderArm
{
public:
    OrderArm(ros::NodeHandle &nh)
    {
        nh_ = nh;
        client = nh_.serviceClient<move_base_agv_msgs::Move_Arm>("move_the_arm");
        ros::Subscriber sub = nh_.subscribe("order_arm_move", 1, &OrderArm::chatterCallback, this);
        ros::spin();
    }
    void chatterCallback(const std_msgs::Bool::ConstPtr& msg);
private:
    ros::NodeHandle nh_;
    ros::ServiceClient client;
    move_base_agv_msgs::Move_Arm srv;
};

void OrderArm::chatterCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data == true)
    {
        cout<<"Get the order to move arm!!"<<endl;
        srv.request.can_arm_move = true;
        client.call(srv);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"order_arm");
    ros::NodeHandle n;
    OrderArm oa(n);
    return 0;
}

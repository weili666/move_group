#ifndef _POSITIONATTITUDE_H
#define _POSITIONATTITUDE_H
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/planning_scene/planning_scene.h>
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

#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#define M_pi 3.141592653

using namespace Eigen;
using namespace Eigen::internal;
using namespace Eigen::Architecture;

using namespace std;
class PositionAttitude
{
public:
    double x,y,z,qw,qx,qy,qz,orx,ory,orz,orqw,orqx,orqy,orqz;
    double score;
    int index;
    int Level;
    double alpha,beta,gamma;
    Eigen::Matrix3d SO3;
    Eigen::Matrix4d SE3;

    PositionAttitude()
    {
        Eigen::Matrix3d SO3;
        Eigen::Matrix4d SE3;
        x=0;y=0;z=0;qw=1;qx=0;qy=0;qz=0;alpha=0;beta=0;gamma=0;
        SO3(0,0)=1; SO3(0,1)=0; SO3(0,2)=0;
        SO3(1,0)=0; SO3(1,1)=1; SO3(1,2)=0;
        SO3(2,0)=0; SO3(2,1)=0; SO3(2,2)=1;
        SE3(0,0)=1; SE3(0,1)=0; SE3(0,2)=0; SE3(0,3)=0;
        SE3(1,0)=0; SE3(1,1)=1; SE3(1,2)=0; SE3(1,3)=0;
        SE3(2,0)=0; SE3(2,1)=0; SE3(2,2)=1; SE3(2,3)=0;
        SE3(3,0)=0; SE3(3,1)=0; SE3(3,2)=0; SE3(3,3)=1;
        alpha=0,beta=0,gamma=0;
        score=0;
    }
    PositionAttitude(double ax,double ay,double az):x(ax),y(ay),z(az)
    {
        Eigen::Matrix3d SO3;
        Eigen::Matrix4d SE3;
        qw=1;qx=0;qy=0;qz=0;alpha=0;beta=0;gamma=0;
        SO3(0,0)=1; SO3(0,1)=0; SO3(0,2)=0;
        SO3(1,0)=0; SO3(1,1)=1; SO3(1,2)=0;
        SO3(2,0)=0; SO3(2,1)=0; SO3(2,2)=1;
        SE3(0,0)=1; SE3(0,1)=0; SE3(0,2)=0; SE3(0,3)=0;
        SE3(1,0)=0; SE3(1,1)=1; SE3(1,2)=0; SE3(1,3)=0;
        SE3(2,0)=0; SE3(2,1)=0; SE3(2,2)=1; SE3(2,3)=0;
        SE3(3,0)=0; SE3(3,1)=0; SE3(3,2)=0; SE3(3,3)=1;
    }
    PositionAttitude(const Eigen::Matrix3d& R,double ax,double ay,double az):x(ax),y(ay),z(az)
    {
	qw=0.5*sqrt(1+R(0,0)+R(1,1)+R(2,2));
	qx=0.5*sqrt(1+R(0,0)-R(1,1)-R(2,2))*(2*((R(1,2)-R(2,1))>0)-1);
	qy=0.5*sqrt(1-R(0,0)+R(1,1)-R(2,2))*(2*((R(1,2)-R(2,1))>0)-1);
	qz=0.5*sqrt(1-R(0,0)-R(1,1)+R(2,2))*(2*((R(1,2)-R(2,1))>0)-1);
	alpha=atan(2*(qw*qx+qy*qz)/(1-2*(qx*qx+qy*qy)));
        beta=atan(2*(qw*qy-qz*qx));
        gamma=atan(2*(qw*qz+qx*qy)/(1-2*(qy*qy+qz*qz)));
    }
    PositionAttitude(double ax,double ay,double az,double aqw,double aqx,double aqy,double aqz):x(ax),y(ay),z(az),qw(aqw),qx(aqx),qy(aqy),qz(aqz)
    {
        Eigen::Matrix3d SO3;
        Eigen::Matrix4d SE3;
        Eigen::Quaterniond l_quat(qw,qx,qy,qz);
        Eigen::Matrix3d l_rot(l_quat);
        SO3=l_rot;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SE3(i,j)=SO3(i,j);
            }
        }
        SE3(0,3)=x;
        SE3(1,3)=y;
        SE3(2,3)=z;
        SE3(3,0)=0;
        SE3(3,1)=0;
        SE3(3,2)=0;
        SE3(3,3)=1;
    alpha=atan(2*(qw*qx+qy*qz)/(1-2*(qx*qx+qy*qy)));
    beta=atan(2*(qw*qy-qz*qx));
    gamma=atan(2*(qw*qz+qx*qy)/(1-2*(qy*qy+qz*qz)));

    }
    PositionAttitude(double sc,double ax,double ay,double az,double aqw,double aqx,double aqy,double aqz):score(sc),x(ax),y(ay),z(az),qw(aqw),qx(aqx),qy(aqy),qz(aqz)
    {
        Eigen::Matrix3d SO3;
        Eigen::Matrix4d SE3;
        Eigen::Quaterniond l_quat(qw,qx,qy,qz);
        Eigen::Matrix3d l_rot(l_quat);
        SO3=l_rot;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SE3(i,j)=SO3(i,j);
            }
        }
        SE3(0,3)=x;
        SE3(1,3)=y;
        SE3(2,3)=z;
        SE3(3,0)=0;
        SE3(3,1)=0;
        SE3(3,2)=0;
        SE3(3,3)=1;
    alpha=atan(2*(qw*qx+qy*qz)/(1-2*(qx*qx+qy*qy)));
    beta=atan(2*(qw*qy-qz*qx));
    gamma=atan(2*(qw*qz+qx*qy)/(1-2*(qy*qy+qz*qz)));

    }
    PositionAttitude(double sc,double ax,double ay,double az,double aqw,double aqx,double aqy,double aqz,double aorx,double aory,double aorz,double aorqw,double aorqx,double aorqy,double aorqz):score(sc),x(ax),y(ay),z(az),qw(aqw),qx(aqx),qy(aqy),qz(aqz),orx(aorx),ory(aory),orz(aorz),orqw(aorqw),orqx(aorqx),orqy(aorqy),orqz(aorqz)
    {
        Eigen::Matrix3d SO3;
        Eigen::Matrix4d SE3;
        Eigen::Quaterniond l_quat(qw,qx,qy,qz);
        Eigen::Matrix3d l_rot(l_quat);
        SO3=l_rot;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SE3(i,j)=SO3(i,j);
            }
        }
        SE3(0,3)=x;
        SE3(1,3)=y;
        SE3(2,3)=z;
        SE3(3,0)=0;
        SE3(3,1)=0;
        SE3(3,2)=0;
        SE3(3,3)=1;
    alpha=atan(2*(qw*qx+qy*qz)/(1-2*(qx*qx+qy*qy)));
    beta=atan(2*(qw*qy-qz*qx));
    gamma=atan(2*(qw*qz+qx*qy)/(1-2*(qy*qy+qz*qz)));

    }
    PositionAttitude(double ax,double ay,double az,double aqw,double aqx,double aqy,double aqz,int num,int Lev):x(ax),y(ay),z(az),qw(aqw),qx(aqx),qy(aqy),qz(aqz)
    {
        Eigen::Matrix3d SO3;
        Eigen::Matrix4d SE3;
        Eigen::Quaterniond l_quat(qw,qx,qy,qz);
        Eigen::Matrix3d l_rot(l_quat);
        SO3=l_rot;
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SE3(i,j)=SO3(i,j);
            }
        }
        SE3(0,3)=x;
        SE3(1,3)=y;
        SE3(2,3)=z;
        SE3(3,0)=0;
        SE3(3,1)=0;
        SE3(3,2)=0;
        SE3(3,3)=1;
    alpha=atan(2*(qw*qx+qy*qz)/(1-2*(qx*qx+qy*qy)));
    beta=atan(2*(qw*qy-qz*qx));
    gamma=atan(2*(qw*qz+qx*qy)/(1-2*(qy*qy+qz*qz)));
    index=num;
    Level=Lev;
    }

    PositionAttitude(double ax,double ay,double az,double aalpha,double abeta,double agamma):x(ax),y(ay),z(az),alpha(aalpha),beta(abeta),gamma(agamma)
    {
        Eigen::Matrix3d SO3;
        Eigen::Matrix4d SE3;
        SO3(0,0)=cos(alpha)*cos(beta);
        SO3(1,0)=sin(alpha)*cos(beta);
        SO3(2,0)=-sin(beta);
        SO3(0,1)=-sin(alpha)*cos(gamma)+cos(alpha)*sin(beta)*sin(gamma);
        SO3(1,1)=cos(alpha)*cos(gamma)+sin(alpha)*sin(beta)*sin(gamma);
        SO3(2,1)=cos(beta)*sin(gamma);
        SO3(0,2)=sin(alpha)*sin(gamma)+cos(alpha)*sin(beta)*cos(gamma);
        SO3(1,2)=-cos(alpha)*sin(gamma)+sin(alpha)*sin(beta)*cos(gamma);
        SO3(2,2)=cos(beta)*cos(gamma);
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                SE3(i,j)=SO3(i,j);
            }
        }
        SE3(0,3)=x;
        SE3(1,3)=y;
        SE3(2,3)=z;
        SE3(3,0)=0;
        SE3(3,1)=0;
        SE3(3,2)=0;
        SE3(3,3)=1;
    qw=cos(alpha/2)*cos(beta/2)*cos(gamma/2)+sin(alpha/2)*sin(beta/2)*sin(gamma/2);
    qx=sin(alpha/2)*cos(beta/2)*cos(gamma/2)-cos(alpha/2)*sin(beta/2)*sin(gamma/2);
    qy=cos(alpha/2)*sin(beta/2)*cos(gamma/2)+sin(alpha/2)*cos(beta/2)*sin(gamma/2);
    qz=cos(alpha/2)*cos(beta/2)*sin(gamma/2)-sin(alpha/2)*sin(beta/2)*cos(gamma/2);

    }

    PositionAttitude(const PositionAttitude& pa)
    {
        x=pa.x;y=pa.y;z=pa.z;
        qw=pa.qw;qx=pa.qx;qy=pa.qy;qz=pa.qz;
        alpha=pa.alpha;beta=pa.beta;gamma=pa.gamma;
        index=pa.index;Level=pa.Level;
    }
};

#endif


#include "ros/ros.h"

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#include <opencv2/core/eigen.hpp>

#include <moveit/move_group_interface/move_group.h>
#include "ompl/util/Time.h"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/robot_state.h>


typedef move_group_interface::MoveGroup C_moveGroup;
typedef moveit::planning_interface::MoveGroup::Plan S_Plan;
typedef moveit::planning_interface::PlanningSceneInterface S_PlanningSceneInterface;

using namespace std;

int n = 0;
class PubCircumstance
{
public:
    PubCircumstance(ros::NodeHandle&  nh_):nh(nh_)
    {
        display_Point3 = nh.advertise<sensor_msgs::PointCloud2>("/camera/depth/points_filter", 1, true);
        display_Marker = nh.advertise<visualization_msgs::MarkerArray>("/marker/depth/points_filter", 1, true);
        planning_scene_diff_publisher = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
        ros::Subscriber sub = nh.subscribe("/xtion/depth_registered/points", 1, & PubCircumstance::Cloud_Cb, this);
        ros::spin();
    }
    void Cloud_Cb( const sensor_msgs::PointCloud2ConstPtr& input );
    void Read_Rt();
private:
    ros::NodeHandle  nh;
    ros::Publisher   display_Point3;
    ros::Publisher   display_Marker;
    ros::Publisher planning_scene_diff_publisher;
    cv::Mat R_external;
    cv::Mat t_external;
    cv::Mat R_checkerboard;
    cv::Mat t_checkerboard;
    cv::Mat R_real;
    cv::Mat t_real;
};

void PubCircumstance::Cloud_Cb( const sensor_msgs::PointCloud2ConstPtr& input )
{
    pcl::PointCloud < pcl::PointXYZRGB > cloud;
    pcl::PointCloud < pcl::PointXYZRGB > cloudrt;
    visualization_msgs::MarkerArray markerArray;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    moveit_msgs::PlanningScene planning_scene;

    double height_table = 0.73;

    pcl::fromROSMsg(*input,cloud);

    Read_Rt();

    for( int i = 0; i < cloud.points.size(); i ++ )
    {
        cv::Mat xp(3,1,CV_32F);
        cv::Mat xnew(3,1,CV_32F);
        xp.at<float>(0,0) = cloud.points[i].x;
        xp.at<float>(1,0) = cloud.points[i].y;
        xp.at<float>(2,0) = cloud.points[i].z;
        xnew = R_real*xp + t_real;

        pcl::PointXYZRGB p;
        p.x = xnew.at<float>(0,0);
        p.y = xnew.at<float>(1,0);
        p.z = xnew.at<float>(2,0) + height_table;
        if(p.z > height_table)
        {
            p.r = cloud.points[i].r;
            p.g = cloud.points[i].g;
            p.b = cloud.points[i].b;
            cloudrt.push_back(p);
        }
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloudrt,output);
    output.header.frame_id = "world_frame";
    display_Point3.publish(output);

}

void PubCircumstance::Read_Rt()
{
    ifstream ifile;
    ifile.open("/home/weili/simple_kinova/src/camera_calibration/saved_image/extern_matrix.txt");
    char buffer[256];
    int num=0;
    int num_of_ros = 6;
    vector< vector<char> > data_;
    vector<char> data_iter;
    if(!ifile.is_open())
    {cout << "Error opening file"; exit (1);}
    while(num_of_ros)
    {
        data_iter.clear();
        ifile.getline(buffer,100);
        for(int i=0;i<256;i++)
        {
            if((buffer[i]=='[')||(buffer[i]==' '))
            {
                continue;
            }
            else if(buffer[i]==',')
            {
                data_.push_back(data_iter);
                data_iter.clear();
                num++;
                continue;
            }
            else if(buffer[i]==';')
            {
                data_.push_back(data_iter);
                data_iter.clear();
                num++;
                break;
            }
            else if(buffer[i]==']')
            {
                data_.push_back(data_iter);
                data_iter.clear();
                num++;
                break;
            }
            else
            {
                data_iter.push_back(buffer[i]);
            }
        }
        num_of_ros--;
    }
    double ddata[12];
    for(int i=0;i<12;i++)
    {
        string data;
        for(int j=0;j<data_[i].size();j++)
        {
            data+=data_[i][j];
        }
        ddata[i]=atof(data.c_str());
    }

    ros::Rate  loop_rate(10000);
    R_external.create(3,3,CV_32F);
    t_external.create(3,1,CV_32F);
    R_external.at<float>(0,0) = ddata[0];
    R_external.at<float>(0,1) = ddata[1];
    R_external.at<float>(0,2) = ddata[2];
    R_external.at<float>(1,0) = ddata[3];
    R_external.at<float>(1,1) = ddata[4];
    R_external.at<float>(1,2) = ddata[5];
    R_external.at<float>(2,0) = ddata[6];
    R_external.at<float>(2,1) = ddata[7];
    R_external.at<float>(2,2) = ddata[8];
    t_external.at<float>(0,0) = ddata[9]/1000;
    t_external.at<float>(1,0) = ddata[10]/1000;
    t_external.at<float>(2,0) = ddata[11]/1000;

    R_checkerboard=R_external.inv();

    t_checkerboard = -R_checkerboard*t_external;
    cv::Mat R_ch2re(3,3,CV_32F);
    R_ch2re.at<float>(0,0) = 0;
    R_ch2re.at<float>(0,1) = 1;
    R_ch2re.at<float>(0,2) = 0;
    R_ch2re.at<float>(1,0) = -1;
    R_ch2re.at<float>(1,1) = 0;
    R_ch2re.at<float>(1,2) = 0;
    R_ch2re.at<float>(2,0) = 0;
    R_ch2re.at<float>(2,1) = 0;
    R_ch2re.at<float>(2,2) = 1;

    R_real = R_ch2re*R_checkerboard;
    //cout<<"R_real:"<<R_real<<endl;
    t_real = R_ch2re*t_checkerboard;
    //cout<<"t_real:"<<t_real<<endl;
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "pub_point_cloud_collision");
    ros::NodeHandle node_handle;
    PubCircumstance pubc(node_handle);

    return 0;
}

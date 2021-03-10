// Author of SSL_SLAM2: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>

#include "mapOptimizationClass.h"

std::mutex mutex_lock;
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
ros::Publisher map_pub;
lidar::Lidar lidar_param;
double scan_period= 0.1;
double map_resolution = 0.4;
double min_map_update_frame = 8;
double min_map_update_angle = 30;
double min_map_update_distance = 1.0;
MapOptimizationClass mapOptimization;
std::string map_path;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();
}
void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();

int stamp = 0;
bool saveMapCallback(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
    
    mapOptimization.optimizeGraph(stamp, mapOptimization.getFrameNum()-1);
    stamp = mapOptimization.getFrameNum();
    mapOptimization.saveMap(map_path);
    //ROS_WARN("write feature map to folder ssl_slam2/map ...");
    res.success = true;
    res.message = "write feature map to folder ssl_slam2/map ...";
    return true;
}

int update_count = 0;
int total_frame=0;

void map_optimization(){
    while(1){
        if(!odometryBuf.empty() && !pointCloudSurfBuf.empty() && !pointCloudEdgeBuf.empty()){

            //read data
            mutex_lock.lock();
            if(!odometryBuf.empty() && (odometryBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || odometryBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                ROS_WARN("time stamp unaligned error and odom discarded, pls check your data --> map optimization"); 
                odometryBuf.pop();
                mutex_lock.unlock();
                continue;              
            }

            if(!pointCloudSurfBuf.empty() && (pointCloudSurfBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudSurfBuf.front()->header.stamp.toSec()<pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudSurfBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> map optimization");
                mutex_lock.unlock();
                continue;  
            }

            if(!pointCloudEdgeBuf.empty() && (pointCloudEdgeBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period || pointCloudEdgeBuf.front()->header.stamp.toSec()<pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period)){
                pointCloudEdgeBuf.pop();
                ROS_INFO("time stamp unaligned with extra point cloud, pls check your data --> map optimization");
                mutex_lock.unlock();
                continue;  
            }

            //if time aligned 
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
            pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
            Eigen::Isometry3d current_pose = Eigen::Isometry3d::Identity();
            current_pose.rotate(Eigen::Quaterniond(odometryBuf.front()->pose.pose.orientation.w,odometryBuf.front()->pose.pose.orientation.x,odometryBuf.front()->pose.pose.orientation.y,odometryBuf.front()->pose.pose.orientation.z));  
            current_pose.pretranslate(Eigen::Vector3d(odometryBuf.front()->pose.pose.position.x,odometryBuf.front()->pose.pose.position.y,odometryBuf.front()->pose.pose.position.z));
            ros::Time pointcloud_time = (odometryBuf.front())->header.stamp;
            pointCloudEdgeBuf.pop();
            pointCloudSurfBuf.pop();
            odometryBuf.pop();
            mutex_lock.unlock();
            total_frame++;   
            if(total_frame%10 == 0) 
                ROS_INFO("total_frame %d", total_frame);
            update_count++;


            Eigen::Isometry3d delta_transform = last_pose.inverse() * current_pose;
            double displacement = delta_transform.translation().squaredNorm();
            double angular_change = delta_transform.linear().eulerAngles(2,1,0)[0]* 180 / M_PI;
            if(angular_change>90) angular_change = fabs(180 - angular_change);
            if(displacement>min_map_update_distance || angular_change>min_map_update_angle || update_count>min_map_update_frame){
                last_pose = current_pose;
                update_count=0;
                mapOptimization.addPoseToGraph(pointcloud_edge_in, pointcloud_surf_in, current_pose);
            }

            if(total_frame%30 ==0){
                sensor_msgs::PointCloud2 PointsMsg;
                pcl::toROSMsg(*(mapOptimization.edgeMap)+*(mapOptimization.surfMap), PointsMsg);
                ROS_INFO("Edge Map size:%d, Surf Map Size:%d", (mapOptimization.edgeMap)->points.size(), (mapOptimization.surfMap)->points.size());
                PointsMsg.header.stamp = pointcloud_time;
                PointsMsg.header.frame_id = "map";
                map_pub.publish(PointsMsg);
            } 

        }
        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    int scan_line = 64;
    double vertical_angle = 2.0;

    double max_dis = 60.0;
    double min_dis = 2.0;

    nh.getParam("/scan_period", scan_period); 
    nh.getParam("/vertical_angle", vertical_angle); 
    nh.getParam("/max_dis", max_dis);
    nh.getParam("/min_dis", min_dis);
    nh.getParam("/scan_line", scan_line);
    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/map_path", map_path);
    nh.getParam("/min_map_update_distance", min_map_update_distance);
    nh.getParam("/min_map_update_angle", min_map_update_angle);
    nh.getParam("/min_map_update_frame", min_map_update_frame);
    
    mapOptimization.init(map_resolution);
    last_pose.translation().x() = 100;
    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf", 100, velodyneSurfHandler);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, odomCallback);

    //map saving service
    ros::ServiceServer srv_save = nh.advertiseService("save_map", saveMapCallback);

    map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    std::thread map_optimization_process{map_optimization};

    ros::spin();

    return 0;
}

// Author of SSL_SLAM2: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#ifndef _ISC_OPTIMIZATION_CLASS_H_
#define _ISC_OPTIMIZATION_CLASS_H_

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/ndt.h>

//GTSAM
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

//ros
#include <ros/ros.h>

//local lib
#include "lidarOptimization.h"
#include "lidar.h"

#define LOOPCLOSURE_THRESHOLD 1.60


class MapOptimizationClass
{
    public:
        MapOptimizationClass();

        void init(double map_resolution_in);

        //return true if global optimized
        void addPoseToGraph(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_edge_in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_surf_in, Eigen::Isometry3d& odom_in);
        bool optimizeGraph(int matched_id, int current_id);
        int getFrameNum(void);
        void saveMap(std::string map_path);

        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointcloud_surf_arr;
        std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointcloud_edge_arr;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr edgeMap; 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr surfMap;
        
        Eigen::Isometry3d map_to_odom;
    private:
        std::vector<gtsam::Pose3> pose_optimized_arr;
        double map_resolution=0.0;
        gtsam::noiseModel::Diagonal::shared_ptr priorModel;
        gtsam::noiseModel::Diagonal::shared_ptr odomModel;
        gtsam::noiseModel::Diagonal::shared_ptr loopModel;

        pcl::VoxelGrid<pcl::PointXYZRGB> downSizeEdgeFilter;
        pcl::VoxelGrid<pcl::PointXYZRGB> downSizeSurfFilter;
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> noiseFilter;

        Eigen::Isometry3d pose3ToEigen(const gtsam::Pose3& pose3);
        gtsam::Pose3 eigenToPose3(const Eigen::Isometry3d& pose_eigen);

        bool geometryConsistencyVerification(int matched_id, int current_id, Eigen::Isometry3d& transform);
        double estimateOdom(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_source_edge, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_source_surf, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_target_edge, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_target_surf, Eigen::Isometry3d& transform);
        void updateMap(void);
};


#endif // _ISC_OPTIMIZATION_CLASS_H_ 
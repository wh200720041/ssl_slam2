// Author of SSL_SLAM2: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _ODOM_ESTIMATION_CLASS_H_
#define _ODOM_ESTIMATION_CLASS_H_

//std lib
#include <string>
#include <math.h>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>

//ceres
#include <ceres/ceres.h>
#include <ceres/rotation.h>

//eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

//LOCAL LIB
#include "lidar.h"
#include "lidarOptimization.h"
#include <ros/ros.h>

class OdomEstimationClass 
{

    public:
    	OdomEstimationClass();
    	
		void init(lidar::Lidar lidar_param, double map_resolution, std::string map_path);	
		void matchPointsToMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& surf_in);
		void setPose(double x_in, double y_in, double z_in, double roll_in, double pitch_in, double yaw_in);
		Eigen::Isometry3d odom;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudCornerMap;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudSurfMap;
	private:
		//optimization variable
		double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
		Eigen::Map<Eigen::Quaterniond> q_w_curr = Eigen::Map<Eigen::Quaterniond>(parameters);
		Eigen::Map<Eigen::Vector3d> t_w_curr = Eigen::Map<Eigen::Vector3d>(parameters + 4);
		Eigen::Isometry3d last_odom;

		//kd-tree
		pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtreeEdgeMap;
		pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtreeSurfMap;

		//points downsampling before add to map
		pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterEdge;
		pcl::VoxelGrid<pcl::PointXYZRGB> downSizeFilterSurf;

		//optimization count 
		int optimization_count;

		//function
		bool addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		bool addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, ceres::Problem& problem, ceres::LossFunction *loss_function);
		void downSamplingToMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& surf_pc_out);
		void pointAssociateToMap(pcl::PointXYZRGB const *const pi, pcl::PointXYZRGB *const po);
};

#endif // _ODOM_ESTIMATION_CLASS_H_


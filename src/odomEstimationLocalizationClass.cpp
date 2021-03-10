// Author of SSL_SLAM2: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "odomEstimationLocalizationClass.h"

void OdomEstimationClass::init(lidar::Lidar lidar_param, double map_resolution, std::string map_path){

    //init local map
    laserCloudCornerMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    laserCloudSurfMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    //load map 
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (map_path+"edge_map.pcd", *laserCloudCornerMap) == -1){
        ROS_ERROR("Couldn't read edge map file %sedge_map.pcd \n",map_path.c_str());
    }
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (map_path+"surf_map.pcd", *laserCloudSurfMap) == -1){
        ROS_ERROR("Couldn't read edge map file %surf_map.pcd \n",map_path.c_str());
    }

    if(laserCloudCornerMap->points.size()<100 || laserCloudSurfMap->points.size()<200){
        ROS_ERROR("not enough points in map to associate, map error");
    }

    //kd-tree
    kdtreeEdgeMap = pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());
    kdtreeSurfMap = pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());
    kdtreeEdgeMap->setInputCloud(laserCloudCornerMap);
    kdtreeSurfMap->setInputCloud(laserCloudSurfMap);

    //downsampling size
    downSizeFilterEdge.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeFilterSurf.setLeafSize(map_resolution * 2, map_resolution * 2, map_resolution * 2);

    odom = Eigen::Isometry3d::Identity();

    last_odom = odom;
    optimization_count=6;
}

void OdomEstimationClass::setPose(double x_in, double y_in, double z_in, double roll_in, double pitch_in, double yaw_in){

    odom = Eigen::Isometry3d::Identity();
    odom = Eigen::AngleAxisd(yaw_in, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pitch_in, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(roll_in, Eigen::Vector3d::UnitX()) * odom;
    odom.translation() = Eigen::Vector3d(x_in,y_in,z_in);
    last_odom = odom;
}

void OdomEstimationClass::matchPointsToMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& edge_in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& surf_in){
    if(optimization_count>2)
        optimization_count--;

    Eigen::Isometry3d odom_prediction = odom * (last_odom.inverse() * odom);
    last_odom = odom;
    odom = odom_prediction;

    q_w_curr = Eigen::Quaterniond(odom.rotation());
    t_w_curr = odom.translation();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledEdgeCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledSurfCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    downSamplingToMap(edge_in,downsampledEdgeCloud,surf_in,downsampledSurfCloud);

    bool edge_point_flag = false;
    bool surf_point_flag = false;
    for (int iterCount = 0; iterCount < optimization_count; iterCount++){
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);

        problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
        
        edge_point_flag = addEdgeCostFactor(downsampledEdgeCloud,problem,loss_function);
        surf_point_flag = addSurfCostFactor(downsampledSurfCloud,problem,loss_function);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        options.check_gradients = false;
        options.gradient_check_relative_precision = 1e-4;
        ceres::Solver::Summary summary;

        ceres::Solve(options, &problem, &summary);
    }

    if(edge_point_flag && surf_point_flag){
        odom = Eigen::Isometry3d::Identity();
        odom.linear() = q_w_curr.toRotationMatrix();
        odom.translation() = t_w_curr; 
    }else{
        ROS_WARN("insufficient points matched");
    }


}
void OdomEstimationClass::downSamplingToMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& edge_pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& edge_pc_out, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& surf_pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& surf_pc_out){
    downSizeFilterEdge.setInputCloud(edge_pc_in);
    downSizeFilterEdge.filter(*edge_pc_out);
    downSizeFilterSurf.setInputCloud(surf_pc_in);
    downSizeFilterSurf.filter(*surf_pc_out);    
}

bool OdomEstimationClass::addEdgeCostFactor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int corner_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZRGB point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeEdgeMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis); 
        if (pointSearchSqDis[4] < 1.0)
        {
            std::vector<Eigen::Vector3d> nearCorners;
            Eigen::Vector3d center(0, 0, 0);
            for (int j = 0; j < 5; j++)
            {
                Eigen::Vector3d tmp(laserCloudCornerMap->points[pointSearchInd[j]].x,
                                    laserCloudCornerMap->points[pointSearchInd[j]].y,
                                    laserCloudCornerMap->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
            }
            center = center / 5.0;

            Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
            for (int j = 0; j < 5; j++)
            {
                Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

            Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
            { 
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d point_a, point_b;
                point_a = 0.1 * unit_direction + point_on_line;
                point_b = -0.1 * unit_direction + point_on_line;

                ceres::CostFunction *cost_function = new EdgeAnalyticCostFunction(curr_point, point_a, point_b);  
                problem.AddResidualBlock(cost_function, loss_function, parameters);
                corner_num++;   
            }                           
        }
    }
    if(corner_num<20){
        return false;
    }else{
        return true;
    }

}
void OdomEstimationClass::pointAssociateToMap(pcl::PointXYZRGB const *const pi, pcl::PointXYZRGB *const po)
{
    Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
    Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
    po->x = point_w.x();
    po->y = point_w.y();
    po->z = point_w.z();
    //po->intensity = 1.0;
}
bool OdomEstimationClass::addSurfCostFactor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_in, ceres::Problem& problem, ceres::LossFunction *loss_function){
    int surf_num=0;
    for (int i = 0; i < (int)pc_in->points.size(); i++)
    {
        pcl::PointXYZRGB point_temp;
        pointAssociateToMap(&(pc_in->points[i]), &point_temp);
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeSurfMap->nearestKSearch(point_temp, 5, pointSearchInd, pointSearchSqDis);

        Eigen::Matrix<double, 5, 3> matA0;
        Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
        if (pointSearchSqDis[4] < 1.0)
        {
            
            for (int j = 0; j < 5; j++)
            {
                matA0(j, 0) = laserCloudSurfMap->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfMap->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfMap->points[pointSearchInd[j]].z;
            }
            // find the norm of plane
            Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
            double negative_OA_dot_norm = 1 / norm.norm();
            norm.normalize();

            bool planeValid = true;
            for (int j = 0; j < 5; j++)
            {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) * laserCloudSurfMap->points[pointSearchInd[j]].x +
                         norm(1) * laserCloudSurfMap->points[pointSearchInd[j]].y +
                         norm(2) * laserCloudSurfMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                {
                    planeValid = false;
                    break;
                }
            }
            Eigen::Vector3d curr_point(pc_in->points[i].x, pc_in->points[i].y, pc_in->points[i].z);
            if (planeValid)
            {
                ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);    
                problem.AddResidualBlock(cost_function, loss_function, parameters);

                surf_num++;
            }
        }

    }
    if(surf_num<20){
        return false;
    }else{
        return true;
    }

}

OdomEstimationClass::OdomEstimationClass(){

}
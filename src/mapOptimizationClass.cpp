// Author of SSL_SLAM2: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "mapOptimizationClass.h"

MapOptimizationClass::MapOptimizationClass()
{
    
}

void MapOptimizationClass::init(double map_resolution_in){

    map_resolution = map_resolution_in;
    edgeMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    surfMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>()); 

    map_to_odom = Eigen::Isometry3d::Identity();
    pointcloud_surf_arr.clear();
    pointcloud_edge_arr.clear();

    // A prior factor consists of a mean value and a noise model (covariance matrix)
    priorModel = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
    // odometry measurement noise model (covariance matrix)
    odomModel = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) <<  0.10, 0.10, 0.10, 0.10, 0.10, 0.10).finished());
    //loop noise model
    loopModel = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.10, 0.10, 0.10, 0.10, 0.10, 0.10).finished());
    //attention, here is x y z r p y order

    downSizeEdgeFilter.setLeafSize(map_resolution, map_resolution, map_resolution);
    downSizeSurfFilter.setLeafSize(map_resolution*2, map_resolution*2, map_resolution*2);

    // Create the filtering object
    noiseFilter.setMeanK (10);
    noiseFilter.setStddevMulThresh (1.0);
 
}

int MapOptimizationClass::getFrameNum(void){
    return pointcloud_edge_arr.size();
}

void MapOptimizationClass::saveMap(std::string map_path){
    pcl::io::savePCDFileASCII (map_path+"edge_map.pcd", *edgeMap);
    pcl::io::savePCDFileASCII (map_path+"surf_map.pcd", *surfMap);  

}

void MapOptimizationClass::addPoseToGraph(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_edge_in, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointcloud_surf_in, Eigen::Isometry3d& odom_in){

    Eigen::Isometry3d transformed_pose = map_to_odom * odom_in;

    downSizeEdgeFilter.setInputCloud(pointcloud_edge_in);
    downSizeSurfFilter.setInputCloud(pointcloud_surf_in);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_edge_in(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_surf_in(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    downSizeEdgeFilter.filter(*filtered_edge_in);
    downSizeSurfFilter.filter(*filtered_surf_in);

    pointcloud_edge_arr.push_back(filtered_edge_in);
    pointcloud_surf_arr.push_back(filtered_surf_in);
    
    
    gtsam::Pose3 pose3_current = eigenToPose3(transformed_pose);
    pose_optimized_arr.push_back(pose3_current);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc_edge(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc_surf(new pcl::PointCloud<pcl::PointXYZRGB>()); 

    pcl::transformPointCloud(*filtered_edge_in, *transformed_pc_edge, transformed_pose.cast<float>());
    pcl::transformPointCloud(*filtered_surf_in, *transformed_pc_surf, transformed_pose.cast<float>());

    *edgeMap += *transformed_pc_edge;
    *surfMap += *transformed_pc_surf;

}

bool MapOptimizationClass::optimizeGraph(int matched_id, int current_id){
    //check consistency, if fail return false
    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    if(geometryConsistencyVerification(matched_id, current_id, transform)==false)
        return false;
    //init everything
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initials;
    graph.push_back(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('x', 1), pose_optimized_arr[matched_id], priorModel));   
    initials.insert(gtsam::Symbol('x', 1), pose_optimized_arr[matched_id]);
    for(int i = matched_id+1;i<=current_id;i++){
        initials.insert(gtsam::Symbol('x', i + 1 - matched_id), pose_optimized_arr[i]);
        graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', i - matched_id), gtsam::Symbol('x', i + 1 - matched_id), pose_optimized_arr[i-1].between(pose_optimized_arr[i]), odomModel));
    }
    graph.push_back(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::Symbol('x', 1), gtsam::Symbol('x', current_id - matched_id +1), eigenToPose3(transform), loopModel));

    gtsam::Values result = gtsam::LevenbergMarquardtOptimizer(graph, initials).optimize();

    map_to_odom = pose3ToEigen(pose_optimized_arr[current_id].between(result.at<gtsam::Pose3>(gtsam::Symbol('x',current_id - matched_id+1))));
    //update pose
    for(int i =0;i<(int)result.size();i++){
        pose_optimized_arr[matched_id+i]= result.at<gtsam::Pose3>(gtsam::Symbol('x',i+1));
    }
    for(int i =current_id+1;i<getFrameNum();i++){
        pose_optimized_arr[i]=eigenToPose3(map_to_odom) * pose_optimized_arr[i];
    }
    //update map
    updateMap();

    return true;
}
void MapOptimizationClass::updateMap(void){

    edgeMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    surfMap = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    for(int i =0;i<getFrameNum();i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc_edge(new pcl::PointCloud<pcl::PointXYZRGB>()); 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_pc_surf(new pcl::PointCloud<pcl::PointXYZRGB>()); 
        pcl::transformPointCloud(*pointcloud_edge_arr[i], *transformed_pc_edge, pose3ToEigen(pose_optimized_arr[i]).cast<float>());
        pcl::transformPointCloud(*pointcloud_surf_arr[i], *transformed_pc_surf, pose3ToEigen(pose_optimized_arr[i]).cast<float>());

        *edgeMap += *transformed_pc_edge;
        *surfMap += *transformed_pc_surf;
    }

}


Eigen::Isometry3d MapOptimizationClass::pose3ToEigen(const gtsam::Pose3& pose3){
    Eigen::Isometry3d pose_eigen = Eigen::Isometry3d::Identity();
    gtsam::Quaternion q_temp = pose3.rotation().toQuaternion();
    pose_eigen.rotate(Eigen::Quaterniond(q_temp.w(),q_temp.x(),q_temp.y(),q_temp.z()));
    pose_eigen.pretranslate(Eigen::Vector3d(pose3.translation().x(),pose3.translation().y(),pose3.translation().z()));
    return pose_eigen;
}

gtsam::Pose3 MapOptimizationClass::eigenToPose3(const Eigen::Isometry3d& pose_eigen){
    Eigen::Quaterniond q(pose_eigen.rotation());
    return gtsam::Pose3(gtsam::Rot3::Quaternion(q.w(), q.x(), q.y(), q.z()), gtsam::Point3(pose_eigen.translation().x(), pose_eigen.translation().y(), pose_eigen.translation().z()));
}

bool MapOptimizationClass::geometryConsistencyVerification(int matched_id, int current_id, Eigen::Isometry3d& transform){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_surf_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_edge_temp(new pcl::PointCloud<pcl::PointXYZRGB>());  
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_surf(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_edge(new pcl::PointCloud<pcl::PointXYZRGB>()); 

    for(int i = -10; i <=10; i=i+5){
        if(matched_id+i>= current_id || matched_id+i<0)
            continue;
        Eigen::Isometry3d transform_pose = pose3ToEigen(pose_optimized_arr[matched_id+i]);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pointcloud_surf_arr[matched_id+i], *transformed_temp, transform_pose.cast<float>());
        *map_surf_temp+=*transformed_temp;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_temp2(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pointcloud_edge_arr[matched_id+i], *transformed_temp2, transform_pose.cast<float>());
        *map_edge_temp+=*transformed_temp2;
    }

    Eigen::Isometry3d transform_pose = pose3ToEigen(pose_optimized_arr[matched_id]);
    pcl::transformPointCloud(*map_edge_temp, *map_edge, transform_pose.cast<float>().inverse());
    pcl::transformPointCloud(*map_surf_temp, *map_surf, transform_pose.cast<float>().inverse());
    //ROS_INFO("tag31");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scan_surf_temp(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scan_edge_temp(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scan_surf(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_scan_edge(new pcl::PointCloud<pcl::PointXYZRGB>()); 
    for(int i = 0; i <=0; i=i+3){
        if(current_id-i<0)
            continue;
        Eigen::Isometry3d transform_pose = pose3ToEigen(pose_optimized_arr[current_id+i]);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_temp(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pointcloud_surf_arr[current_id-i], *transformed_temp, transform_pose.cast<float>());
        *current_scan_surf_temp+=*transformed_temp;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_temp2(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pointcloud_edge_arr[current_id-i], *transformed_temp2, transform_pose.cast<float>());
        *current_scan_edge_temp+=*transformed_temp2;
    }

    transform_pose = pose3ToEigen(pose_optimized_arr[current_id]);
    pcl::transformPointCloud(*current_scan_edge_temp, *current_scan_edge, transform_pose.cast<float>().inverse());
    pcl::transformPointCloud(*current_scan_surf_temp, *current_scan_surf, transform_pose.cast<float>().inverse());

    double match_score = estimateOdom(map_edge,map_surf,current_scan_edge,current_scan_surf,transform);
    //ROS_WARN("global optimization geometry consistency matched score %f, if it is too large, global optimization will not enabled",match_score);

    if(match_score < LOOPCLOSURE_THRESHOLD / map_resolution){
        return true;
    }
    else{
        ROS_WARN("loop rejected due to geometry verification current_id%d matched_id %d, score: %f solution",current_id,matched_id,match_score);
        ROS_WARN("No loop closure identified, the map will be generated without global optimization. if you want to force a loop closure happen, change your loop closure threshold higher");
        return false;
    }
}

double MapOptimizationClass::estimateOdom(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_source_edge, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_source_surf, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_target_edge, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_target_surf, Eigen::Isometry3d& transform){
    Eigen::Quaterniond init_q(transform.rotation());
    Eigen::Vector3d init_t(0,0,0);
    double parameters[7] = {init_q.x(), init_q.y(), init_q.z(), init_q.w(),init_t.x(), init_t.y(), init_t.z()};
    Eigen::Map<Eigen::Quaterniond> q_temp = Eigen::Map<Eigen::Quaterniond>(parameters);
    Eigen::Map<Eigen::Vector3d> t_temp = Eigen::Map<Eigen::Vector3d>(parameters + 4);
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtreeCorner = pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr kdtreeSurf = pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZRGB>());
    kdtreeCorner->setInputCloud(pc_source_edge);
    kdtreeSurf->setInputCloud(pc_source_surf);
    double total_cost = 300;
    for (int opti_counter = 0; opti_counter < 10; opti_counter++)
    {
        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
        ceres::Problem::Options problem_options;

        ceres::Problem problem(problem_options);
        problem.AddParameterBlock(parameters, 7, new PoseSE3Parameterization());
 
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translation() = t_temp;
        transform.linear() = q_temp.toRotationMatrix();
        //add edge cost factor
        int corner_num=0;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tranformed_edge(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pc_target_edge, *tranformed_edge, transform);
        for (int i = 0; i < (int) tranformed_edge->points.size(); i++)
        {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeCorner->nearestKSearch(tranformed_edge->points[i], 5, pointSearchInd, pointSearchSqDis);
            if (pointSearchSqDis[4] < 2.0)
            {
                std::vector<Eigen::Vector3d> nearCorners;
                Eigen::Vector3d center(0, 0, 0);
                for (int j = 0; j < 5; j++)
                {
                    Eigen::Vector3d tmp(pc_source_edge->points[pointSearchInd[j]].x,
                                        pc_source_edge->points[pointSearchInd[j]].y,
                                        pc_source_edge->points[pointSearchInd[j]].z);
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
                Eigen::Vector3d curr_point(pc_target_edge->points[i].x, pc_target_edge->points[i].y, pc_target_edge->points[i].z);
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
            ROS_INFO("not enough corresponding points");
            return 300.0;
        }
        //add surf cost factor
        int surf_num=0; 
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr tranformed_surf(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::transformPointCloud(*pc_target_surf, *tranformed_surf, transform);    
        // find correspondence for plane features
        for (int i = 0; i <(int) tranformed_surf->points.size(); ++i)
        {
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            kdtreeSurf->nearestKSearch(tranformed_surf->points[i], 5, pointSearchInd, pointSearchSqDis);
            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (pointSearchSqDis[4] < 2.0)
            {
                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = pc_source_surf->points[pointSearchInd[j]].x;
                    matA0(j, 1) = pc_source_surf->points[pointSearchInd[j]].y;
                    matA0(j, 2) = pc_source_surf->points[pointSearchInd[j]].z;
                }
                
                Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                double negative_OA_dot_norm = 1 / norm.norm();
                norm.normalize();
                bool planeValid = true;
                
                for (int j = 0; j < 5; j++)
                {
                    
                    if (fabs(norm(0) * pc_source_surf->points[pointSearchInd[j]].x +
                             norm(1) * pc_source_surf->points[pointSearchInd[j]].y +
                             norm(2) * pc_source_surf->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }

                Eigen::Vector3d curr_point(pc_target_surf->points[i].x, pc_target_surf->points[i].y, pc_target_surf->points[i].z);
                if (planeValid)
                {
                    ceres::CostFunction *cost_function = new SurfNormAnalyticCostFunction(curr_point, norm, negative_OA_dot_norm);    
                    problem.AddResidualBlock(cost_function, loss_function, parameters);
                    surf_num++;
                }

            }
        }

        if(surf_num<20){
            ROS_INFO("not enough corresponding points");
            return 300.0;
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if(summary.final_cost<total_cost)
            total_cost = summary.final_cost;
    }
    //transform = Eigen::Isometry3d::Identity();
    transform.linear() = q_temp.toRotationMatrix();
    transform.translation() = t_temp;
    return total_cost;

}
#include "pcl_registration.hpp"
#include <pcl/features/fpfh.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
using namespace PCL_Handle;
void PclIcp_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    auto params = searchSelfJson(content);

    // fmt::print({}, params.dump(4));
    auto pointcloud_name = params["input"]["pointcloud_name"].get<std::string>();
    std::string transform_name = params["input"]["transform_name"].get<std::string>();
    std::string output_transform_name = params["output"]["transform_name"].get<std::string>();
    bool random_icp = params["random_icp"].get<bool>();
    int max_iter = params["max_iterations"].get<int>();
    float max_correspondence_distance = params["max_correspondence_distance"].get<float>();
    float transformation_epsilon = params["transformation_epsilon"].get<float>();
    float euclidean_fitness_epsilon = params["euclidean_fitness_epsilon"].get<float>();
    float yaw_offset = params["yaw_offset"].get<float>();
    float xy_offset = params["xy_offset"].get<float>();
    float yaw_resolution = params["yaw_resolution"].get<float>();
    // 先读取transform
    auto transform = content["running"][transform_name].get<std::vector<float>>();
    Eigen::Matrix4f matrix = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(transform.data());
    _transform_vec.push_back(matrix);
    // 读取点云
    auto pointcloud_ptr = std::any_cast<std::shared_ptr<PointCloud_t>>((*_global_ptr)[pointcloud_name]);
    if (random_icp)
    {
        // 按照yaw_offset,xy_offset,yaw_resolution生成候选点
        for (float yaw = -yaw_offset; yaw < yaw_offset; yaw += yaw_resolution)
        {
            for (float x = -xy_offset; x < xy_offset; x += xy_offset)
            {
                for (float y = -xy_offset; y < xy_offset; y += xy_offset)
                {
                    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
                    transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ()).toRotationMatrix();
                    transform.block<3, 1>(0, 3) = Eigen::Vector3f(x, y, 0);
                    _transform_vec.push_back(transform);
                }
            }
        }
    }
    // 进行icp
    Eigen::Matrix4f output_transform;
    pcl::IterativeClosestPoint<Point_t, Point_t> icp;
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setMaximumIterations(max_iter);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    for (auto &transform : _transform_vec)
    {
        icp.setInputSource(PointInput_ptr);
        icp.setInputTarget(pointcloud_ptr);
        icp.align(*PointInput_ptr, transform);
        if (icp.hasConverged())
        {
            output_transform = icp.getFinalTransformation();
            fmt::print("icp converged\n");
            fmt::print("score: {}\n", icp.getFitnessScore());
            fmt::print("transform:\n{}\n", output_transform);
        }
        else
        {
            fmt::print("icp not converged\n");
        }
    }
    // 保存transform
    std::vector<float> transform_vec(output_transform.data(), output_transform.data() + 16);
    content["running"][output_transform_name] = transform_vec;
}
void PclFPFHIcp_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    auto params = searchSelfJson(content);
    auto pointcloud_name = params["input"]["map_pointcloud_name"].get<std::string>();
    std::string transform_name = params["input"]["transform_name"].get<std::string>();
    std::string output_transform_name = params["output"]["transform_name"].get<std::string>();
    std::string fpfh_source_name = params["input"]["fpfh_source"].get<std::string>();
    std::string fpfh_target_name = params["input"]["fpfh_map"].get<std::string>();
    // 提取fpfh特征与点云
    auto fpfh_source_ptr = std::any_cast<std::shared_ptr<pcl::PointCloud<pcl::FPFHSignature33>>>((*_global_ptr)[fpfh_source_name]);
    auto fpfh_target_ptr = std::any_cast<std::shared_ptr<pcl::PointCloud<pcl::FPFHSignature33>>>((*_global_ptr)[fpfh_target_name]);
    auto pointcloud_ptr = std::any_cast<std::shared_ptr<PointCloud_t>>((*_global_ptr)[pointcloud_name]);
    // 读取transform
    auto transform = content["running"][transform_name].get<std::vector<float>>();
    Eigen::Matrix4f matrix = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(transform.data());
    // 进行粗配准
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
    est.setInputSource(fpfh_source_ptr);            // 设置源点云
    est.setInputTarget(fpfh_target_ptr);            // 设置目标点云
    est.determineCorrespondences(*correspondences); // 计算对应关系
    pcl::registration::TransformationEstimationSVD<Point_t, Point_t> trans_estimation_svd;
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();

    // 使用对应关系计算变换矩阵
    trans_estimation_svd.estimateRigidTransformation(*PointInput_ptr, *pointcloud_ptr, *correspondences, transformation_matrix);
    // 保存transform
    std::vector<float> transform_vec(transformation_matrix.data(), transformation_matrix.data() + 16);
    content["running"][output_transform_name] = transform_vec;
    // pcl::registration::CorrespondenceEstimation<pcl::FPFHEstimation<Point_t, pcl::Normal, pcl::FPFHSignature33>, pcl::FPFHSignature33> est;
    // est.setInputSource(fpfh_source_ptr);
    // est.setInputTarget(fpfh_target_ptr);
    // est.determineCorrespondences(*correspondences);
}
void PclNdt_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    auto params = searchSelfJson(content);
    auto pointcloud_name = params["input"]["map_pointcloud_name"].get<std::string>();
    std::string transform_name = params["input"]["transform_name"].get<std::string>();
    std::string output_transform_name = params["output"]["transform_name"].get<std::string>();
    float resolution = params["resolution"].get<float>();
    int iterations = params["iterations"].get<int>();
    float step_size = params["step_size"].get<float>();
    float epsilon = params["epsilon"].get<float>();
    auto pointcloud_ptr = std::any_cast<std::shared_ptr<PointCloud_t>>((*_global_ptr)[pointcloud_name]);
    auto transform = content["running"][transform_name].get<std::vector<float>>();
    Eigen::Matrix4f matrix = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(transform.data());
    // 进行ndt
    pcl::NormalDistributionsTransform<Point_t, Point_t> ndt;
    ndt.setTransformationEpsilon(epsilon);
    ndt.setStepSize(step_size);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(iterations);
    ndt.setInputSource(PointInput_ptr);
    ndt.setInputTarget(pointcloud_ptr);
    ndt.align(*PointInput_ptr, matrix);
    auto transform_matrix = ndt.getFinalTransformation();
    if (ndt.hasConverged())
    {
        fmt::print("ndt converged\n");
        fmt::print("score: {}\n", ndt.getFitnessScore());
        fmt::print("transform:\n{}\n", transform_matrix);
    }
    else
    {
        fmt::print("ndt not converged\n");
    }
    // 保存transform
    std::vector<float> transform_vec(transform_matrix.data(), transform_matrix.data() + 16);
    content["running"][output_transform_name] = transform_vec;
}

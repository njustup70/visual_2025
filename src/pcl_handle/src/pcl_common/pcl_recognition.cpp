#include "pcl_recognition.hpp"
#include <fmt/ostream.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
using namespace PCL_Handle;

void PclSegmentation_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    if (PointInput_ptr->empty())
    {
        return;
    }
    // double distance_threshold = content["pcl_segmentation"]["distance_threshold"].get<double>();
    double distance_threshold = searchParam("distance_threshold", content).get<double>();
    pcl::SACSegmentation<Point_t> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);        // 设置平面模型
    seg.setMethodType(pcl::SAC_RANSAC);           // 使用 RANSAC 方法
    seg.setDistanceThreshold(distance_threshold); // 平面和点的最大距离

    // 输入点云数据
    seg.setInputCloud(PointInput_ptr);

    // 创建存储提取出来的平面点的对象
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);
    pcl::ExtractIndices<Point_t> extract;
    extract.setInputCloud(PointInput_ptr); // 输入原始点云
    extract.setIndices(inliers);           // 输入平面上的点的索引
    extract.setNegative(false);            // 提取平面上的点
    extract.filter(*PointInput_ptr);       // 将提取的点存储到 cloud_plane
    // 将平面方程保存
    std::vector<float> plane_coefficients;
    for (auto &coefficient : coefficients->values)
    {
        plane_coefficients.push_back(coefficient);
    }
    auto plane_name = searchParam("output", content)["plane_name"].get<std::string>();
    content["running"][plane_name] = plane_coefficients;
}
void PclCenterTransform_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    if (PointInput_ptr->empty())
    {
        return;
    }

    // _print = content["pcl_centerTransform"]["print"].get<bool>();
    _print = searchParam("print", content).get<bool>();
    // 提取中心点
    Eigen::Vector4f centroid;
    // 算出最大最小的x,y,z值
    float min_x = std::numeric_limits<float>::max();
    float max_x = std::numeric_limits<float>::lowest();
    float min_y = std::numeric_limits<float>::max();
    float max_y = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (auto &point : PointInput_ptr->points)
    {
        min_x = std::min(min_x, point.x);
        max_x = std::max(max_x, point.x);
        min_y = std::min(min_y, point.y);
        max_y = std::max(max_y, point.y);
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    //  pcl::compute3DCentroid(*PointInput_ptr, centroid);
    centroid[0] = (min_x + max_x) / 2;
    centroid[1] = (min_y + max_y) / 2;
    centroid[2] = (min_z + max_z) / 2;
    if (_print)
    {
        fmt::print("centroid:{}\n", centroid);
    }
    PointCloud_t::Ptr center_points(new PointCloud_t);
    Point_t center_point;
    center_point.x = centroid[0];
    center_point.y = centroid[1];
    center_point.z = centroid[2];
    center_points->push_back(center_point);
    //  提取平面方程
    auto plane_name = searchParam("inputs", content)["plane_name"].get<std::string>();
    std::vector<float>
        plane_coefficients = content["running"][plane_name].get<std::vector<float>>();
    Eigen::Vector3f normal(plane_coefficients[0], plane_coefficients[1], plane_coefficients[2]);
    // 提取参考点
    Eigen::Vector3f ref_point_vec(centroid[0], centroid[1], centroid[2]);
    // 算出变换矩阵
    Eigen::Vector3f x_axis(1, 0, 0);
    if (fabs(normal.dot(x_axis)) > 0.9)
    {
        x_axis = Eigen::Vector3f(0, 1, 0); // 如果法向量与X轴接近平行，选择Y轴
    }

    // 计算Y轴，保证X, Y, Z轴正交
    Eigen::Vector3f y_axis = normal.cross(x_axis).normalized();
    // 重新计算X轴，保证正交
    x_axis = y_axis.cross(normal).normalized();

    // Step 4.2: 构建旋转矩阵
    Eigen::Matrix3f rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = normal;

    // Step 4.3: 构建平移向量
    Eigen::Vector3f translation = ref_point_vec; // 使用参考点作为平移向量

    // Step 4.4: 构建4x4齐次变换矩阵
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = rotation;    // 填充旋转矩阵
    transform.block<3, 1>(0, 3) = translation; // 填充平移向量
    if (_print)
    {
        fmt::print("transform:\n{}\n", transform);
    }
    // 将变换存到json上下文中
    auto output = searchParam("output", content);
    std::string transform_name = output["transform_name"].get<std::string>();
    std::string center_points_name = output["center_points_name"].get<std::string>();
    content["running"][transform_name] = std::vector<float>(transform.data(), transform.data() + transform.size());
    // content["running"]["transform"] = std::vector<float>(transform.data(), transform.data() + transform.size());
    content["running"]["pointclouds"].push_back(center_points_name);
    (*(_global_ptr))["center_points"] = center_points;
}
#include "pcl_filter.hpp"
#include "format_eigen.hpp"
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace PCL_Handle;
void PclFilterZero_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    if (PointInput_ptr->empty())
    {
        return;
    }
    float max_z = searchParam("max_z", content).get<float>();
    float min_z = searchParam("min_z", content).get<float>();
    float zero_z = searchParam("zero_z", content).get<float>();
    float max_r = 0;
    try
    {

        max_r = searchParam("max_r", content).get<float>();
    }
    catch (const std::exception &e)
    {
        fmt::print("use default r=0\n");
    }
    // if (!_matrix.isIdentity())
    // {
    // pcl::transformPointCloud(*PointInput_ptr, *PointInput_ptr, _matrix);
    // }
    // 先变换到世界坐标系

    fmt::print("zero_z:{}\n", zero_z);
    pcl::PassThrough<Point_t> pass;
    pass.setInputCloud(PointInput_ptr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(min_z - zero_z, max_z - zero_z);
    pass.filter(*PointInput_ptr);
    // 定义圆心和半径
    // Point_t center(_matrix(0, 3), _matrix(1, 3), _matrix(2, 3));
    Point_t center(0, 0, zero_z);
    // 传入kd树的是半径的平方
    double radius = max_r;

    // 创建k-d树
    pcl::KdTreeFLANN<Point_t> kdtree;
    kdtree.setInputCloud(PointInput_ptr);

    // 用于存储搜索结果
    std::vector<int> point_indices;
    std::vector<float> point_squared_distances;
    if (max_r == 0.0f)
    {
        return;
    }
    // 使用半径搜索，找到所有距离中心小于等于radius的点
    kdtree.radiusSearch(center, radius, point_indices, point_squared_distances);

    // 创建一个新点云用于存储过滤后的点
    auto filtered_cloud(new pcl::PointCloud<Point_t>);
    // 将搜索到的点存入新点云
    for (int idx : point_indices)
    {
        filtered_cloud->push_back(PointInput_ptr->points[idx]);
    }
    // 将新点云赋值给原始点云
    PointInput_ptr.reset(filtered_cloud);
}

void PclVoxel_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    float voxel_size = searchParam("voxel_size", content).get<float>();
    auto voxel_ptr = std::make_shared<PointCloud_t>();
    // 创建滤波器
    pcl::VoxelGrid<Point_t> sor;
    sor.setInputCloud(PointInput_ptr);
    sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    sor.filter(*voxel_ptr);
    PointInput_ptr = voxel_ptr;
}
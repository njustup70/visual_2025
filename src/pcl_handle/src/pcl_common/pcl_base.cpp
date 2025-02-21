#include "pcl_base.hpp"
#include <fmt/core.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
using namespace PCL_Handle;
void PclStage_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    auto name = searchParam("output", content)["pointcloud_name"].get<std::string>();
    bool use_copy = searchParam("copy", content).get<bool>();
    if (use_copy)
    {
        auto PointStage_ptr = std::make_shared<PointCloud_t>(*PointInput_ptr);
        (*(_global_ptr))[name] = PointStage_ptr;
    }
    else
    {
        (*(_global_ptr))[name] = PointInput_ptr;
    }
    content["running"]["pointclouds"].push_back(name);
}
void PclPop_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    auto name = searchSelfJson(content)["input"]["pointcloud_name"].get<std::string>();
    PointInput_ptr = std::any_cast<std::shared_ptr<PointCloud_t>>((*_global_ptr)[name]);
}
void JsonPrint_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    (void)PointInput_ptr;
    fmt::print("json print:{}\n", content["running"].dump(4));
}
void PcdRead::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    (void)PointInput_ptr;
    auto pcd = searchParam("pcd", content).get<std::string>();
    std::string pointcloud_name = searchParam("output", content)["pointcloud_name"].get<std::string>();
    std::string path = std::filesystem::path(__FILE__).parent_path().string() + pcd;
    // 化简path
    path = std::filesystem::canonical(std::filesystem::path(path)).string();
    fmt::print("pcd path is {}\n", path);
    auto pointcloud_map_ptr = std::make_shared<PointCloud_t>();
    pcl::io::loadPCDFile<Point_t>(path, *pointcloud_map_ptr);
    (*_global_ptr)[pointcloud_name] = pointcloud_map_ptr;
    content["running"]["pointclouds"].push_back(pointcloud_name);
    PointInput_ptr = pointcloud_map_ptr;
}
void PclTransform_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    auto transform_name = searchParam("input", content)["transform_name"].get<std::string>();
    auto transform = content["running"][transform_name].get<std::vector<float>>();
    Eigen::Matrix4f matrix = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::RowMajor>>(transform.data());
    pcl::transformPointCloud(*PointInput_ptr, *PointInput_ptr, matrix);
}
void PclSave_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    auto pcd = "/output.pcd";
    std::string path = std::filesystem::path(__FILE__).parent_path().string() + pcd;
    // 化简path
    // path = std::filesystem::canonical(std::filesystem::path(path)).string();
    fmt::print("pcd path is {}\n", path);
    if (!std::filesystem::exists(path))
    {
        // 文件不存在，创建它
        std::ofstream file(path);
    }
    pcl::io::savePCDFileBinary(path, *PointInput_ptr);
}
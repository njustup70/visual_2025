#include "pcl_visual.hpp"
#include "pcl/visualization/pcl_visualizer.h"
using namespace PCL_Handle;
void PclVisual_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    try
    {
        for (int i = 0; i < 3; ++i)
        {
            _background_color[i] = searchParam("background_color", content)[i].get<uint8_t>();
            _point_color[i] = searchParam("point_color", content)[i].get<uint8_t>();
        }
        _show_axes = searchParam("show_axes", content).get<bool>();
        _show_normals = searchParam("show_normals", content).get<bool>();
    }
    catch (const std::exception &e)
    {
        fmt::print("use default color\n");
    }
    (void)content;
    (void)PointInput_ptr;
    // 将点云数据复制一份,深拷贝
    auto PointShow_ptr = std::make_shared<PointCloud_t>(*PointInput_ptr);
    // 创建visualization对象
    if (_viewer_ptr.get() == nullptr)
    {
        _viewer_ptr = std::make_shared<pcl::visualization::PCLVisualizer>("viewer");
        _viewer_ptr->setBackgroundColor(_background_color[0], _background_color[1], _background_color[2]);
        // 设置点云颜色
        pcl::visualization::PointCloudColorHandlerCustom<Point_t> single_color(PointShow_ptr, _point_color[0], _point_color[1], _point_color[2]);
        // 添加点云
        _viewer_ptr->addPointCloud(PointShow_ptr, single_color, "cloud");
        if (_show_axes)
        {
            _viewer_ptr->addCoordinateSystem(1.0);
        }
    }
    else
    {
        // 更新点云
        pcl::visualization::PointCloudColorHandlerCustom<Point_t> single_color(PointShow_ptr, _point_color[0], _point_color[1], _point_color[2]);
        _viewer_ptr->updatePointCloud(PointShow_ptr, single_color, "cloud");
    }
    // 设置背景颜色

    // 显示法线
    if (_show_normals)
    {
        // auto normal_ptr_name = content["running"]["normal"].get<std::string>();
        auto normal_ptr_name = searchParam("input", content)["normal_name"].get<std::string>();
        if (_global_ptr->find(normal_ptr_name) != _global_ptr->end())
        {
            auto normal_ptr = std::any_cast<pcl::PointCloud<pcl::Normal>::Ptr>(_global_ptr->at(normal_ptr_name));
            _viewer_ptr->removePointCloud("normals");
            _viewer_ptr->addPointCloudNormals<Point_t, pcl::Normal>(PointShow_ptr, normal_ptr, 1, 0.1, "normals");
        }
        else
        {
            fmt::print("normal not found\n");
        }
        // viewer.addPointCloudNormals<Point_t>(PointShow_ptr, 1, 0.1, "normals");
    }

    // 创建执行器
}
void PclVisual_t::spinOnce(json &content)
{
    (void)content;
    if (_viewer_ptr)
    {
        _viewer_ptr->spinOnce(20);
    }
}
void PclMultiVisual_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    // _pointcloud_name = content["running"]["pointclouds"].get<std::vector<std::string>>();
    auto addtion_content = searchParam("addition_settings", content);
    if (_first)
    {
        _viewer_ptr = std::make_shared<pcl::visualization::PCLVisualizer>("viewer");
        _first = false;
        // auto color_setings = searchParam("addition_settings", content).get<std::vector<std::vector<uint8_t>>>();
        auto background_color = searchParam("background_color", content).get<std::vector<uint8_t>>();
        for (auto &content : addtion_content)
        {
            std::string name = content["name"].get<std::string>();
            auto color = content["color"].get<std::vector<uint8_t>>();
            uint8_t size = content["size"].get<uint8_t>();
            pcl::visualization::PointCloudColorHandlerCustom<Point_t> single_color(PointInput_ptr, color[0], color[1], color[2]);
            _viewer_ptr->addPointCloud(PointInput_ptr, single_color, name);
            _viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
        }
        if (_show_default_pointcloud)
        {
            auto color = searchParam("default_color", content).get<std::vector<uint8_t>>();
            auto PointShow_ptr = PointInput_ptr;
            // 设置点云颜色
            pcl::visualization::PointCloudColorHandlerCustom<Point_t> single_color(PointShow_ptr, color[0], color[1], color[2]);
            // 添加点云
            _viewer_ptr->addPointCloud(PointShow_ptr, single_color, "default");
        }
        if (_show_axes)
        {
            _viewer_ptr->addCoordinateSystem(1.0);
        }
        _viewer_ptr->setBackgroundColor(background_color[0], background_color[1], background_color[2]);
    }
    else
    {
        for (auto &content : addtion_content)
        {
            std::string name = content["name"].get<std::string>();
            auto color = content["color"].get<std::vector<uint8_t>>();
            uint8_t size = content["size"].get<uint8_t>();
            auto PointShow_ptr = std::any_cast<std::shared_ptr<PointCloud_t>>(_global_ptr->at(name));
            // 设置点云颜色
            pcl::visualization::PointCloudColorHandlerCustom<Point_t> single_color(PointShow_ptr, color[0], color[1], color[2]);
            // 更新点云
            _viewer_ptr->updatePointCloud(PointShow_ptr, single_color, name);
            _viewer_ptr->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, name);
        }
        if (_show_default_pointcloud)
        {
            auto color = searchParam("default_color", content).get<std::vector<uint8_t>>();
            pcl::visualization::PointCloudColorHandlerCustom<Point_t> single_color(PointInput_ptr, color[0], color[1], color[2]);
            auto PointShow_ptr = PointInput_ptr;
            _viewer_ptr->updatePointCloud(PointShow_ptr, single_color, "default");
            // _viewer_ptr->updatePointCloud(PointShow_ptr, "default");
            // 更新点云
        }
    }
}
void PclMultiVisual_t::spinOnce(json &content)
{
    (void)content;
    if (_viewer_ptr)
    {
        _viewer_ptr->spinOnce(20);
    }
}

#ifndef __PCL_VISUAL_HPP
#define __PCL_VISUAL_HPP
#include "pcl_interface.hpp"
namespace pcl
{
namespace visualization
{
class PCLVisualizer;
}
} // namespace pcl
namespace PCL_Handle
{
/**
 * @brief pcl可视化类,将点云可视化
 * @param json里需要有background_color,point_color,show_normals,show_axes 分别表示背景颜色,点云颜色,是否显示法线,是否显示坐标轴
 * 在show_normals为true的情况下,需要有json["output"]["normal"]表示法线实例名字,并在全局共享变量中有这个实例
 * @return 没有返回值
 */
class PclVisual_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    void spinOnce(json &content) override;
    std::string getName() override
    {
        return "PclVisual";
    }

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> _viewer_ptr = nullptr;
    uint8_t _background_color[3] = {0, 0, 0};
    uint8_t _point_color[3] = {255, 255, 255};
    bool _show_normals = false;
    bool _show_axes = true;
};
/**
 * @brief pcl多点云可视化类,将多个点云可视化,会可视化["output"]["pointclouds"]中的点云
 * @param json里面需要有["output"]["pointclouds"]
 */
class PclMultiVisual_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    void spinOnce(json &content) override;
    std::string getName() override
    {
        return "PclMultiVisual";
    }

private:
    std::shared_ptr<pcl::visualization::PCLVisualizer> _viewer_ptr = nullptr;
    bool _first = true;
    bool _show_default_pointcloud = true;
    bool _show_axes = true;
};
} // namespace PCL_Handle
#endif
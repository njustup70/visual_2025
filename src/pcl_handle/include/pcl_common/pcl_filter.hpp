#ifndef __PCL_FILTER_HPP
#define __PCL_FILTER_HPP
#include "pcl_interface.hpp"
namespace PCL_Handle
{
/**
 * @brief 过滤类,将点云中z轴在[min_z-zero_z,max_z-zero_z]之间的点云保留下来,在传入radius的情况下,将圆心为(0,0,zero_z)的半径为radius的点云保留下来
 * @param json里需要有max_z,min_z,zero_z,max_r,max_r是可选的默认为0,表示不进行半径滤波
 * @return 没有返回值
 */
class PclFilterZero_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclFilterZero";
    }

private:
};

class PclVoxel_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclVoxel";
    }
};
} // namespace PCL_Handle
#endif
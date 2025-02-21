#ifndef __PCL_RECOGNITION_HPP
#define __PCL_RECOGNITION_HPP
#include "pcl_interface.hpp"
namespace PCL_Handle
{
/**
 * @brief 分割类,将点云分割成平面
 * @param json里需要有distance_threshold,表示距离阈值
 * @return 会在json["output"]["plane_coefficients"]返回平面方程,类型为std::vector<std::vector<float>>
 */
class PclSegmentation_t final : public PclInterface_t
{
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclSegmentation";
    }
};
/**
 * @brief 从平面点和平面方程算出中心点并算出到世界坐标系的变换矩阵
 * @param json里需要有plane_coefficients,表示平面方程,print表示是否打印 plane_coefficients可以通过PclSegmentation_t得到
 * @return json["output"]["transform"]返回变换矩阵
 */
class PclCenterTransform_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclCenterTransform";
    }

private:
    bool _print = false;
};
} // namespace PCL_Handle
#endif
#ifndef __PCL_BASE_HPP
#define __PCL_BASE_HPP
#include "pcl_interface.hpp"
namespace PCL_Handle
{

class PclStage_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclStage";
    }
};
/**
 * @brief 将存起来的点云数据更新到管道中
 *
 */
class PclPop_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclPop";
    }
};

class JsonPrint_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "JsonPrint";
    }
};
class PcdRead final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PcdRead";
    }
};
class PclTransform_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclTransform";
    }
};
class PclSave_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclSave";
    }
};
} // namespace PCL_Handle
#endif
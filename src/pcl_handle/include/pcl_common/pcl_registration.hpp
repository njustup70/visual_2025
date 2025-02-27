#ifndef __PCL_REGISTRATION_HPP
#define __PCL_REGISTRATION_HPP
#include "pcl_interface.hpp"
namespace PCL_Handle
{
class PclFPFHIcp_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclFPFHIcp";
    }
};
class PclIcp_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclIcp";
    }

private:
    std::vector<Eigen::Matrix4f> _transform_vec;
};
class PclNdt_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclNdt";
    }
};
} // namespace PCL_Handle
#endif
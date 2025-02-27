#ifndef __PCL_FEATURE_HPP
#define __PCL_FEATURE_HPP
#include "pcl_interface.hpp"
namespace PCL_Handle
{
class PclFeaturesExtract_t final : public PclInterface_t
{
public:
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) override;
    std::string getName() override
    {
        return "PclFeaturesExtract";
    }
};

} // namespace PCL_Handle
#endif

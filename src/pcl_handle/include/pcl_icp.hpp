#ifndef PCL_ICP_HPP
#define PCL_ICP_HPP

#include "pcl_handle.hpp"

class PclIcp : public PclHandle
{
public:
    explicit PclIcp(const rclcpp::NodeOptions &options);

private:
    json _map_init_json;
    PCL_Handle::PclFactory_t _MapPclPipeLine;
    std::shared_ptr<rclcpp::TimerBase> _timer;
};

#endif // PCL_ICP_HPP

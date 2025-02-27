#include "pcl_factory.hpp"
#include "pcl_base.hpp"
#include "pcl_feature.hpp"
#include "pcl_filter.hpp"
#include "pcl_recognition.hpp"
#include "pcl_registration.hpp"
#include "pcl_visual.hpp"
#if USE_ROS
#include "pcl_implementation_ros.hpp"
#endif
using namespace PCL_Handle;
void PclFactory_t::createHandle(std::string name, json &content)
{
    if (name == "PclFilterZero")
    {
        _PclPipeLine.push_back(std::make_shared<PclFilterZero_t>());
    }
    else if (name == "PclSegmentation")
    {
        _PclPipeLine.push_back(std::make_shared<PclSegmentation_t>());
    }
    else if (name == "PclVisual")
    {
        _PclPipeLine.push_back(std::make_shared<PclVisual_t>());
    }
    else if (name == "PclCenterTransform")
    {
        _PclPipeLine.push_back(std::make_shared<PclCenterTransform_t>());
    }
    else if (name == "PclMultiVisual")
    {
        _PclPipeLine.push_back(std::make_shared<PclMultiVisual_t>());
    }
    else if (name == "PclStage")
    {
        _PclPipeLine.push_back(std::make_shared<PclStage_t>());
    }
    else if (name == "JsonPrint")
    {
        _PclPipeLine.push_back(std::make_shared<JsonPrint_t>());
    }
    else if (name == "PclIcp")
    {
        _PclPipeLine.push_back(std::make_shared<PclIcp_t>());
    }
    // else if (name == "PclExtract")
    // {
    // _PclPipeLine.push_back(std::make_shared<PCLExtract_t>());
    // }
    else if (name == "PclPop")
    {
        _PclPipeLine.push_back(std::make_shared<PclPop_t>());
    }
    else if (name == "PcdRead")
    {
        _PclPipeLine.push_back(std::make_shared<PcdRead>());
    }
    else if (name == "PclVoxel")
    {
        _PclPipeLine.push_back(std::make_shared<PclVoxel_t>());
    }
    else if (name == "PclFeaturesExtract")
    {
        _PclPipeLine.push_back(std::make_shared<PclFeaturesExtract_t>());
    }
    else if (name == "PclFPFHIcp")
    {
        _PclPipeLine.push_back(std::make_shared<PclFPFHIcp_t>());
    }
    else if (name == "PclNdt")
    {
        _PclPipeLine.push_back(std::make_shared<PclNdt_t>());
    }
    else if (name == "PclTransform")
    {
        _PclPipeLine.push_back(std::make_shared<PclTransform_t>());
    }
    else if (name == "PclSave")
    {
        _PclPipeLine.push_back(std::make_shared<PclSave_t>());
    }
#if USE_ROS
    else if (name == "PclPublish")
    {
        _PclPipeLine.push_back(std::make_shared<PclPublish_t>());
    }
#endif
    else
    {
        fmt::print("not found type {}_t \n", name);
    }
    if (_PclPipNum.count(name) == 0)
    {
        _PclPipNum[name] = 0;
    }
    else
    {
        _PclPipNum[name]++;
    }
    // 对新加进去的handle进行初始化
    _PclPipeLine.back()->init(content, &_global_ptr, _PclPipNum[name]);
    fmt::print("create {}_t\n", name);
    if (name != _PclPipeLine.back()->getName())
    {
        fmt::print("warring name not match\n");
        fmt::print("create name is :{}\n", name);
        fmt::print("pcl node name is :{}\n", _PclPipeLine.back()->getName());
    }
}
void PclFactory_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    for (auto &pclHandle : _PclPipeLine)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> start;
        if (_print_time)
        {
            start = std::chrono::high_resolution_clock::now();
        }
        pclHandle->handlePCL(PointInput_ptr, content);
        if (_print_time)
        {
            auto end = std::chrono::high_resolution_clock::now();
            auto duration_ms = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end - start);
            fmt::print("handle {} time: {:.1f} ms\n", pclHandle->getName(), duration_ms.count());
        }
    }
}
void PclFactory_t::spinOnce(json &content)
{
    for (auto &pclHandle : _PclPipeLine)
    {
        pclHandle->spinOnce(content);
    }
}
void PclFactory_t::addGlobalPtr(std::string name, std::any ptr)
{
    _global_ptr[name] = ptr;
}
void PclFactory_t::enablePrintTime(bool print_time)
{
    _print_time = print_time;
}
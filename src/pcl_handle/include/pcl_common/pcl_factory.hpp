#ifndef __PCL_FACTORY_HPP
#define __PCL_FACTORY_HPP
#define USE_ROS 1
#include "pcl_interface.hpp"
namespace PCL_Handle
{
/**
 * @brief Pcl工厂类,用于创建Pcl处理类,包括管线的初始化和处理
 *
 */
class PclFactory_t
{
public:
    PclFactory_t(std::unordered_map<std::string, std::any> &global_ptr) : _global_ptr(global_ptr)
    {
    }
    /**
     * @brief 创建具体的处理类
     *
     * @param name 通过名字来创建对应的处理类
     * @param content json配置
     */
    void createHandle(std::string name, json &content);
    /**
     * @brief 处理点云数据,会把点云和json配置传入到每个处理类中
     *
     * @param PointInput_ptr 输入点云
     * @param content  json配置
     */
    void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content);
    /**
     * @brief 主函数循环一次
     *
     * @param content json配置
     */
    void spinOnce(json &content);
    void addGlobalPtr(std::string name, std::any ptr);
    void enablePrintTime(bool print_time);

private:
    std::vector<std::shared_ptr<PclInterface_t>> _PclPipeLine; // 处理类的容器/管线
    std::unordered_map<std::string, std::any> &_global_ptr;    // 全局变量存放区
    std::unordered_map<std::string, int> _PclPipNum;           // 记录每个处理类的数量
    json _content;
    bool _print_time;
};
} // namespace PCL_Handle
#endif
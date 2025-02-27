/**
 * @file pcl_interface.hpp
 * @author Elaina (1463967532@qq.com)
 * @brief pcl处理接口，包括父类虚函数和点云数据类型
 * @version 0.1
 * @date 2024-12-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef PCL_INTERFACE_HPP_
#define PCL_INTERFACE_HPP_
#include "../3rd_packages/json.hpp"
// #include "format_eigen.hpp"
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
using json = nlohmann::json;
namespace PCL_Handle
{
/**
 * @brief 点云数据类型
 *
 */
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud_t;
/**
 * @brief 一个点的数据类型
 *
 */
typedef pcl::PointXYZI Point_t;
/**
 * @brief mid360的点云数据类型
 *
 */
struct EIGEN_ALIGN16 PointXYZITR
{
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
/**
 * @brief pcl处理接口类,所有的pcl处理类都要继承这个类
 *
 */
class PclInterface_t
{
public:
    virtual ~PclInterface_t() = default;
    /**
     * @brief 初始化函数
     *
     * @param content json配置
     * @param global_ptr  全局变量存放区指针
     * @param id 管线中同样处理节点第几个的id
     */
    virtual void init(nlohmann::json &content, std::unordered_map<std::string, std::any> *global_ptr, int id = 0)
    {
        (void)content;
        _global_ptr = global_ptr;
        _id = id;
    }
    /**
     * @brief 处理点云数据,需要实现
     *
     * @param PointInput_ptr 输入点云
     * @param content json配置
     */
    virtual void handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content) = 0;
    /**
     * @brief 注册处理节点的名字,需要实现
     *
     * @return std::string 节点名字
     */
    virtual std::string getName() = 0;
    /**
     * @brief 在主循环中调用的函数,默认为空
     *
     * @param content json配置
     */
    virtual void spinOnce(nlohmann::json &content) { (void)content; }

protected:
    /**
     * @brief 通过参数名字到json文件中查找对应命名空间下,对应id的参数
     *  例如: 通过 "max_z" 到 json文件中查找
     *  "PclFilterZero"->"default"->"max_z"
     *  或者 "PclFilterZero"->"id1"->"max_z"
     * @param name 参数名字
     * @param content 要查找的json文件
     * @return nlohmann::basic_json<> json类,和正常json[]操作一样,需要判断是否为空
     */
    nlohmann::basic_json<> searchParam(const std::string &name, json &content, int id = -1)
    {
        // 先找到处理节点的命名空间
        auto node_name = getName();
        nlohmann::basic_json<> node_namespace;

        if (content.contains(node_name))
        {
            node_namespace = content[node_name];
        }
        else
        {
            fmt::print("not found {} in {}\n", node_name, content.dump());
            return nullptr;
        }

        // 先找 default 的值
        nlohmann::basic_json<> param;

        if (node_namespace.contains("default"))
        {
            param = node_namespace["default"][name];
        }
        else
        {
            fmt::print("not found param  {}\n", name);
        }
        if (id == -1)
        {
            id = _id;
        }
        if (id != 0)
        {
            if (node_namespace.contains("id" + std::to_string(id)))
            {
                param = node_namespace["id" + std::to_string(id)][name];
            }
            // else
            // {
            //     // fmt::print("not found id{} in {}\n", _id, node_namespace.dump());
            //     return nullptr;
            // }
        }

        return param;
    }
    nlohmann::basic_json<> searchSelfJson(json &content, int id = -1)
    {
        auto node_name = getName();
        nlohmann::basic_json<> node_namespace;

        if (content.contains(node_name))
        {
            node_namespace = content[node_name];
        }
        else
        {
            fmt::print("not found {} in {}\n", node_name, content.dump());
            return nullptr;
        }
        nlohmann::basic_json<> param;

        if (node_namespace.contains("default"))
        {
            param = node_namespace["default"];
        }
        if (id == -1)
        {
            id = _id;
        }
        if (id != 0)
        {
            if (node_namespace.contains("id" + std::to_string(id)))
            {
                param = node_namespace["id" + std::to_string(id)];
            }
            // else
            // {
            //     // fmt::print("not found id{} in {}\n", _id, node_namespace.dump());
            //     return nullptr;
            // }
        }

        return param;
    }
    std::unordered_map<std::string, std::any> *_global_ptr; // 全局变量存放区指针
    int _id = 0;                                            // 管线中同样处理节点第几个的id
};

} // namespace PCL_Handle
POINT_CLOUD_REGISTER_POINT_STRUCT(PCL_Handle::PointXYZITR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring));
#endif

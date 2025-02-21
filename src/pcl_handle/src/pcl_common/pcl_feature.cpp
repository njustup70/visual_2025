#include "pcl_feature.hpp"
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
using namespace PCL_Handle;
void PclFeaturesExtract_t::handlePCL(std::shared_ptr<PointCloud_t> &PointInput_ptr, json &content)
{
    auto normal_name = searchParam("output", content)["normal_name"].get<std::string>();
    float radius = searchParam("radius", content).get<float>();
    bool extractfphf = searchParam("extractfphf", content).get<bool>();
    // 创建法线估计对象
    pcl::NormalEstimation<Point_t, pcl::Normal> ne;
    ne.setInputCloud(PointInput_ptr);
    pcl::search::KdTree<Point_t>::Ptr tree(new pcl::search::KdTree<Point_t>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(radius);
    ne.compute(*normals);
    (*_global_ptr)[normal_name] = normals;
    content["running"]["pointclouds"].push_back(normal_name);
    if (extractfphf)
    {
        auto fphf_name = searchParam("output", content)["fphf_name"].get<std::string>();
        // 创建FPFH估计对象
        pcl::FPFHEstimation<Point_t, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(PointInput_ptr);
        fpfh.setInputNormals(normals);
        pcl::search::KdTree<Point_t>::Ptr tree_fpfh(new pcl::search::KdTree<Point_t>);
        fpfh.setSearchMethod(tree_fpfh);
        // pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs(new pcl::PointCloud<pcl::FPFHSignature33>());
        auto fpfhs = std::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();
        fpfh.setRadiusSearch(radius);
        fpfh.compute(*fpfhs);
        (*_global_ptr)[fphf_name] = fpfhs;
        content["running"]["pointclouds"].push_back(fphf_name);
    }
}
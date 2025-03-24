#include <memory>
#include <mutex>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>

#include "rclcpp_components/register_node_macro.hpp"

namespace hdl_localization {

class GlobalmapServer : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZI;

  explicit GlobalmapServer(const rclcpp::NodeOptions & options)
  : Node("globalmap_server", options)
  {
    // 声明参数
    this->declare_parameter<std::string>("globalmap_pcd", "");
    this->declare_parameter<bool>("convert_utm_to_local", true);
    this->declare_parameter<double>("downsample_resolution", 0.1);

    // 初始化全局地图
    initialize_params();

    // 使用 transient_local QoS 实现 latched 发布
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).transient_local();
    globalmap_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/globalmap", qos);

    // 订阅地图更新请求
    map_update_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/map_request/pcd", 10,
      std::bind(&GlobalmapServer::map_update_callback, this, std::placeholders::_1));

    // 创建一次性定时器，延时 1 秒后发布一次全局地图
    globalmap_pub_timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&GlobalmapServer::pub_once_cb, this));
  }

private:
  // 从 PCD 文件加载全局地图，并进行下采样和 UTM 坐标转换
  void initialize_params() {
    std::string globalmap_pcd;
    this->get_parameter("globalmap_pcd", globalmap_pcd);
    globalmap_.reset(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPCDFile(globalmap_pcd, *globalmap_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "无法读取文件：%s", globalmap_pcd.c_str());
      return;
    }
    globalmap_->header.frame_id = "map";

    // 如果参数允许且存在 .utm 文件，则对地图点进行 UTM 坐标转换
    bool convert_utm_to_local;
    this->get_parameter("convert_utm_to_local", convert_utm_to_local);
    if (convert_utm_to_local) {
      std::ifstream utm_file(globalmap_pcd + ".utm");
      if (utm_file.is_open()) {
        double utm_easting, utm_northing, altitude;
        utm_file >> utm_easting >> utm_northing >> altitude;
        utm_file.close();
        for (auto& pt : globalmap_->points) {
          pt.x -= static_cast<float>(utm_easting);
          pt.y -= static_cast<float>(utm_northing);
          pt.z -= static_cast<float>(altitude);
        }
        RCLCPP_INFO(this->get_logger(),
                    "全局地图已根据 UTM 坐标 (x = %f, y = %f, z = %f) 进行偏移",
                    utm_easting, utm_northing, altitude);
      }
    }
    // 对全局地图进行下采样
    double downsample_resolution;
    this->get_parameter("downsample_resolution", downsample_resolution);
    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid.setInputCloud(globalmap_);
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid.filter(*filtered);
    globalmap_ = filtered;
  }

  // 定时器回调：发布一次全局地图，然后取消定时器（实现一次性发布）
  void pub_once_cb() {
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*globalmap_, output);
    output.header.stamp = this->now();
    globalmap_pub_->publish(output);
    globalmap_pub_timer_->cancel();
  }

  // 地图更新请求回调：根据传入的 PCD 文件路径加载新地图，并进行下采样后发布
  void map_update_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "收到地图更新请求，地图路径: %s", msg->data.c_str());
    std::string globalmap_pcd = msg->data;
    globalmap_.reset(new pcl::PointCloud<PointT>());
    if (pcl::io::loadPCDFile(globalmap_pcd, *globalmap_) == -1) {
      RCLCPP_ERROR(this->get_logger(), "无法读取文件：%s", globalmap_pcd.c_str());
      return;
    }
    globalmap_->header.frame_id = "map";

    double downsample_resolution;
    this->get_parameter("downsample_resolution", downsample_resolution);
    pcl::VoxelGrid<PointT> voxelgrid;
    voxelgrid.setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);
    voxelgrid.setInputCloud(globalmap_);
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    voxelgrid.filter(*filtered);
    globalmap_ = filtered;

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*globalmap_, output);
    output.header.stamp = this->now();
    globalmap_pub_->publish(output);
  }

  // 成员变量
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr map_update_sub_;
  rclcpp::TimerBase::SharedPtr globalmap_pub_timer_;
  pcl::PointCloud<PointT>::Ptr globalmap_;
};

}  // namespace hdl_localization

// 注册组件，使其可以通过组件管理器加载
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::GlobalmapServer)

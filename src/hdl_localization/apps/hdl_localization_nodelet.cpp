#include <mutex>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <atomic>
#include <chrono>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "pcl_ros/transforms.hpp"
#include <pcl_conversions/pcl_conversions.h>


#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "std_srvs/srv/empty.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/string.hpp"

#include <pcl/filters/voxel_grid.h>

#include <pclomp/ndt_omp.h>
#include <fast_gicp/ndt/ndt_cuda.hpp>

#include <hdl_localization/pose_estimator.hpp>
#include <hdl_localization/delta_estimater.hpp>

#include <hdl_localization/msg/scan_matching_status.hpp>
#include <hdl_global_localization/srv/set_global_map.hpp>
#include <hdl_global_localization/srv/query_global_localization.hpp>

namespace hdl_localization {

class HdlLocalizationNode : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZI;

  explicit HdlLocalizationNode(const rclcpp::NodeOptions & options)
  : Node("hdl_localization_node", options),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(this)
  {
    // 参数声明
    this->declare_parameter<std::string>("robot_odom_frame_id", "robot_odom");
    this->declare_parameter<std::string>("odom_child_frame_id", "base_link");
    this->declare_parameter<bool>("use_imu", true);
    this->declare_parameter<bool>("invert_acc", false);
    this->declare_parameter<bool>("invert_gyro", false);
    this->declare_parameter<double>("downsample_resolution", 0.1);
    this->declare_parameter<std::string>("reg_method", "NDT_OMP");
    this->declare_parameter<std::string>("ndt_neighbor_search_method", "DIRECT7");
    this->declare_parameter<double>("ndt_neighbor_search_radius", 2.0);
    this->declare_parameter<double>("ndt_resolution", 1.0);
    this->declare_parameter<bool>("specify_init_pose", true);
    this->declare_parameter<double>("init_pos_x", 0.0);
    this->declare_parameter<double>("init_pos_y", 0.0);
    this->declare_parameter<double>("init_pos_z", 0.0);
    this->declare_parameter<double>("init_ori_w", 1.0);
    this->declare_parameter<double>("init_ori_x", 0.0);
    this->declare_parameter<double>("init_ori_y", 0.0);
    this->declare_parameter<double>("init_ori_z", 0.0);
    this->declare_parameter<double>("cool_time_duration", 0.5);
    this->declare_parameter<bool>("enable_robot_odometry_prediction", false);
    this->declare_parameter<double>("status_max_correspondence_dist", 0.5);
    this->declare_parameter<double>("status_max_valid_point_dist", 25.0);
    this->declare_parameter<bool>("use_global_localization", true);

    // 参数获取
    robot_odom_frame_id_ = this->get_parameter("robot_odom_frame_id").as_string();
    odom_child_frame_id_ = this->get_parameter("odom_child_frame_id").as_string();
    use_imu_ = this->get_parameter("use_imu").as_bool();
    invert_acc_ = this->get_parameter("invert_acc").as_bool();
    invert_gyro_ = this->get_parameter("invert_gyro").as_bool();
    use_global_localization_ = this->get_parameter("use_global_localization").as_bool();

    // 初始化相关参数（滤波、配准、位姿估计等）
    initialize_params();

    // 订阅
    if(use_imu_) {
      RCLCPP_INFO(this->get_logger(), "enable imu-based prediction");
      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/gpsimu_driver/imu_data", 256,
        std::bind(&HdlLocalizationNode::imu_callback, this, std::placeholders::_1));
    }
    points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 5,
      std::bind(&HdlLocalizationNode::points_callback, this, std::placeholders::_1));
    globalmap_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/globalmap", 1,
      std::bind(&HdlLocalizationNode::globalmap_callback, this, std::placeholders::_1));
    initialpose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 8,
      std::bind(&HdlLocalizationNode::initialpose_callback, this, std::placeholders::_1));

    // 发布
    pose_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
    aligned_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_points", 5);
    status_pub_ = this->create_publisher<msg::ScanMatchingStatus>("/status", 5);

    // 全局定位相关
    if(use_global_localization_) {
      RCLCPP_INFO(this->get_logger(), "wait for global localization services");
      set_global_map_client_ = this->create_client<hdl_global_localization::srv::SetGlobalMap>("/hdl_global_localization/set_global_map");
      query_global_localization_client_ = this->create_client<hdl_global_localization::srv::QueryGlobalLocalization>("/hdl_global_localization/query");
      if(!set_global_map_client_->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Service /hdl_global_localization/set_global_map not available");
      }
      if(!query_global_localization_client_->wait_for_service(std::chrono::seconds(10))) {
        RCLCPP_ERROR(this->get_logger(), "Service /hdl_global_localization/query not available");
      }
      relocalize_srv_ = this->create_service<std_srvs::srv::Empty>(
        "/relocalize",
        std::bind(&HdlLocalizationNode::relocalize, this, std::placeholders::_1, std::placeholders::_2));
    }
  }

private:
  // 创建配准方法（支持 NDT_OMP 或 NDT_CUDA）
  pcl::Registration<PointT, PointT>::Ptr create_registration() const {
    std::string reg_method = this->get_parameter("reg_method").as_string();
    std::string ndt_neighbor_search_method = this->get_parameter("ndt_neighbor_search_method").as_string();
    double ndt_neighbor_search_radius = this->get_parameter("ndt_neighbor_search_radius").as_double();
    double ndt_resolution = this->get_parameter("ndt_resolution").as_double();

    if(reg_method == "NDT_OMP") {
      RCLCPP_INFO(this->get_logger(), "NDT_OMP is selected");
      pclomp::NormalDistributionsTransform<PointT, PointT>::Ptr ndt(new pclomp::NormalDistributionsTransform<PointT, PointT>());

      ndt->setTransformationEpsilon(0.01);
      ndt->setResolution(ndt_resolution);
      if (ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT1);
      } else if (ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
      } else {
        if (ndt_neighbor_search_method == "KDTREE") {
          RCLCPP_INFO(this->get_logger(), "search_method KDTREE is selected");
        } else {
          RCLCPP_WARN(this->get_logger(), "invalid search method was given, default method KDTREE is selected");
        }
        ndt->setNeighborhoodSearchMethod(pclomp::KDTREE);
      }
      return ndt;
    } else if(reg_method.find("NDT_CUDA") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "NDT_CUDA is selected");
      // auto ndt = boost::make_shared<fast_gicp::NDTCuda<PointT, PointT>>();
      std::shared_ptr<fast_gicp::NDTCuda<PointT, PointT>> ndt = std::make_shared<fast_gicp::NDTCuda<PointT, PointT>>();

      ndt->setResolution(ndt_resolution);
      if(reg_method.find("D2D") != std::string::npos) {
        ndt->setDistanceMode(fast_gicp::NDTDistanceMode::D2D);
      } else if (reg_method.find("P2D") != std::string::npos) {
        ndt->setDistanceMode(fast_gicp::NDTDistanceMode::P2D);
      }
      if(ndt_neighbor_search_method == "DIRECT1") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT1 is selected");
        ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT1);
      } else if(ndt_neighbor_search_method == "DIRECT7") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT7 is selected");
        ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT7);
      } else if(ndt_neighbor_search_method == "DIRECT_RADIUS") {
        RCLCPP_INFO(this->get_logger(), "search_method DIRECT_RADIUS is selected : %f", ndt_neighbor_search_radius);
        ndt->setNeighborSearchMethod(fast_gicp::NeighborSearchMethod::DIRECT_RADIUS, ndt_neighbor_search_radius);
      } else {
        RCLCPP_WARN(this->get_logger(), "invalid search method was given");
      }
      return ndt;
    }
    RCLCPP_ERROR(this->get_logger(), "unknown registration method: %s", reg_method.c_str());
    return nullptr;
  }

  // 参数初始化：下采样滤波器、配准方法、位姿估计器等
  void initialize_params() {
    double downsample_resolution = this->get_parameter("downsample_resolution").as_double();

    downsample_filter_ = std::make_shared<pcl::VoxelGrid<PointT>>();
    downsample_filter_->setLeafSize(downsample_resolution, downsample_resolution, downsample_resolution);

    RCLCPP_INFO(this->get_logger(), "create registration method for localization");
    registration_ = create_registration();

    // 初始化全局定位 fallback 及增量估计器
    relocalizing_.store(false);
    delta_estimater_ = std::make_unique<DeltaEstimater>(create_registration());

    // 如果指定了初始位姿，则初始化位姿估计器
    if(this->get_parameter("specify_init_pose").as_bool()) {
      RCLCPP_INFO(this->get_logger(), "initialize pose estimator with specified parameters!!");
      Eigen::Vector3f init_pos(
        this->get_parameter("init_pos_x").as_double(),
        this->get_parameter("init_pos_y").as_double(),
        this->get_parameter("init_pos_z").as_double());
      Eigen::Quaternionf init_ori(
        this->get_parameter("init_ori_w").as_double(),
        this->get_parameter("init_ori_x").as_double(),
        this->get_parameter("init_ori_y").as_double(),
        this->get_parameter("init_ori_z").as_double());
      double cool_time_duration = this->get_parameter("cool_time_duration").as_double();
      pose_estimator_ = std::make_unique<hdl_localization::PoseEstimator>(registration_, init_pos, init_ori, cool_time_duration);
    }
  }

  // 回调：IMU 数据
  void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg) {
    std::lock_guard<std::mutex> lock(imu_data_mutex_);
    imu_data_.push_back(imu_msg);
  }

  // 回调：点云数据
  void points_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    if(!globalmap_) {
      RCLCPP_ERROR(this->get_logger(), "globalmap has not been received!!");
      return;
    }
    rclcpp::Time stamp = points_msg->header.stamp;
    pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *pcl_cloud);
    if(pcl_cloud->empty()) {
      RCLCPP_ERROR(this->get_logger(), "cloud is empty!!");
      return;
    }
    // 将点云转换到 odom_child_frame_id_ 坐标系下
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    try {
      auto transformStamped = tf_buffer_.lookupTransform(odom_child_frame_id_, points_msg->header.frame_id,
                                                           stamp, std::chrono::duration<double>(0.1));
      if(!pcl_ros::transformPointCloud(odom_child_frame_id_, *pcl_cloud, *cloud, tf_buffer_)) {
        RCLCPP_ERROR(this->get_logger(), "point cloud cannot be transformed into target frame!!");
        return;
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
      return;
    }
    auto filtered = downsample(cloud);
    last_scan_ = filtered;

    if(relocalizing_.load()) {
      delta_estimater_->add_frame(filtered);
    }

    {
      std::lock_guard<std::mutex> estimator_lock(pose_estimator_mutex_);
      if(!pose_estimator_) {
        RCLCPP_ERROR(this->get_logger(), "waiting for initial pose input!!");
        return;
      }
      // 预测：若不使用 IMU 则直接预测，否则遍历 IMU 数据进行预测
      if(!use_imu_) {
        pose_estimator_->predict(stamp);
      } else {
        std::lock_guard<std::mutex> lock(imu_data_mutex_);
        auto imu_iter = imu_data_.begin();
        for(; imu_iter != imu_data_.end(); ++imu_iter) {
          if(stamp < (*imu_iter)->header.stamp) {
            break;
          }
          const auto& acc = (*imu_iter)->linear_acceleration;
          const auto& gyro = (*imu_iter)->angular_velocity;
          double acc_sign = invert_acc_ ? -1.0 : 1.0;
          double gyro_sign = invert_gyro_ ? -1.0 : 1.0;
          pose_estimator_->predict((*imu_iter)->header.stamp,
                                   acc_sign * Eigen::Vector3f(acc.x, acc.y, acc.z),
                                   gyro_sign * Eigen::Vector3f(gyro.x, gyro.y, gyro.z));
        }
        imu_data_.erase(imu_data_.begin(), imu_iter);
      }

      // 若启用机器人里程计预测，则尝试根据 tf 查找转换并进行预测
      rclcpp::Time last_correction_time = pose_estimator_->last_correction_time();
      if(this->get_parameter("enable_robot_odometry_prediction").as_bool() &&
         last_correction_time.seconds() != 0.0)
      {
        geometry_msgs::msg::TransformStamped odom_delta;
        try {
          odom_delta = tf_buffer_.lookupTransform(odom_child_frame_id_, last_correction_time,
                                                    odom_child_frame_id_, stamp, robot_odom_frame_id_,
                                                    std::chrono::duration<double>(0.1));
        } catch (tf2::TransformException &ex) {
          try {
            odom_delta = tf_buffer_.lookupTransform(odom_child_frame_id_, last_correction_time,
                                                    odom_child_frame_id_, rclcpp::Time(0), robot_odom_frame_id_,
                                                    std::chrono::duration<double>(0));
          } catch (tf2::TransformException &ex2) {
            RCLCPP_WARN(this->get_logger(), "failed to look up transform between %s and %s",
                        points_msg->header.frame_id.c_str(), robot_odom_frame_id_.c_str());
          }
        }
        if(odom_delta.header.stamp.sec != 0) {
          Eigen::Isometry3d delta = tf2::transformToEigen(odom_delta);
          pose_estimator_->predict_odom(delta.cast<float>().matrix());
        }
      }

      // 校正步骤：调用位姿估计器进行 scan matching
      auto aligned = pose_estimator_->correct(stamp, filtered);
      if(aligned_pub_->get_subscription_count() > 0) {
        aligned->header.frame_id = "map";
        aligned->header.stamp = cloud->header.stamp;
        sensor_msgs::msg::PointCloud2 aligned_msg;
        pcl::toROSMsg(*aligned, aligned_msg);
        aligned_pub_->publish(aligned_msg);
      }
      if(status_pub_->get_subscription_count() > 0) {
        publish_scan_matching_status(points_msg->header, aligned);
      }
      publish_odometry(points_msg->header, pose_estimator_->matrix());
    }
  }

  // 回调：全局地图输入
  void globalmap_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr points_msg) {
    RCLCPP_INFO(this->get_logger(), "globalmap received!");
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*points_msg, *cloud);
    globalmap_ = cloud;
    registration_->setInputTarget(globalmap_);
    if(use_global_localization_) {
      RCLCPP_INFO(this->get_logger(), "set globalmap for global localization!");
      auto request = std::make_shared<hdl_global_localization::srv::SetGlobalMap::Request>();
      pcl::toROSMsg(*globalmap_, request->global_map);

      auto result = set_global_map_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(rclcpp::Node::SharedPtr(this), result) != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(get_logger(), "Failed to call SetGlobalMap service");
      }

    }
  }

  // 服务回调：全局重定位（relocalize）
  void relocalize(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                  std::shared_ptr<std_srvs::srv::Empty::Response> res) {
    (void) req;
    (void) res;
    if(last_scan_ == nullptr) {
      RCLCPP_INFO(this->get_logger(), "no scan has been received");
      return;
    }
    relocalizing_.store(true);
    delta_estimater_->reset();
    pcl::PointCloud<PointT>::ConstPtr scan = last_scan_;
    auto request = std::make_shared<hdl_global_localization::srv::QueryGlobalLocalization::Request>();
    pcl::toROSMsg(*scan, request->cloud);
    request->max_num_candidates = 1;
    auto future = query_global_localization_client_->async_send_request(request);
    if(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
      relocalizing_.store(false);
      RCLCPP_INFO(this->get_logger(), "global localization failed");
      return;
    }
    auto response = future.get();
    if(response->poses.empty()) {
      relocalizing_.store(false);
      RCLCPP_INFO(this->get_logger(), "global localization failed");
      return;
    }
    const auto & result = response->poses[0];
    RCLCPP_INFO(this->get_logger(), "--- Global localization result ---");
    RCLCPP_INFO(this->get_logger(), "Trans : %f %f %f", result.position.x, result.position.y, result.position.z);
    RCLCPP_INFO(this->get_logger(), "Quat  : %f %f %f %f", result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w);
    RCLCPP_INFO(this->get_logger(), "Error : %f", response->errors[0]);
    RCLCPP_INFO(this->get_logger(), "Inlier: %f", response->inlier_fractions[0]);
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = Eigen::Quaternionf(result.orientation.w, result.orientation.x, result.orientation.y, result.orientation.z).toRotationMatrix();
    pose.translation() = Eigen::Vector3f(result.position.x, result.position.y, result.position.z);
    pose = pose * delta_estimater_->estimated_delta();
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      double cool_time_duration = this->get_parameter("cool_time_duration").as_double();
      pose_estimator_ = std::make_unique<hdl_localization::PoseEstimator>(
        registration_,
        pose.translation(),
        Eigen::Quaternionf(pose.linear()),
        cool_time_duration);
    }
    relocalizing_.store(false);
  }

  // 回调：初始位姿输入（例如 rviz “2D Pose Estimate”）
  void initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr pose_msg) {
    RCLCPP_INFO(this->get_logger(), "initial pose received!!");
    std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
    const auto& p = pose_msg->pose.pose.position;
    const auto& q = pose_msg->pose.pose.orientation;
    double cool_time_duration = this->get_parameter("cool_time_duration").as_double();
    pose_estimator_ = std::make_unique<hdl_localization::PoseEstimator>(
      registration_,
      Eigen::Vector3f(p.x, p.y, p.z),
      Eigen::Quaternionf(q.w, q.x, q.y, q.z),
      cool_time_duration);
  }

  // 下采样
  pcl::PointCloud<PointT>::ConstPtr downsample(const pcl::PointCloud<PointT>::ConstPtr& cloud) const {
    if(!downsample_filter_) {
      return cloud;
    }
    pcl::PointCloud<PointT>::Ptr filtered(new pcl::PointCloud<PointT>());
    downsample_filter_->setInputCloud(cloud);
    downsample_filter_->filter(*filtered);
    filtered->header = cloud->header;
    return filtered;
  }

  // 发布里程计（以及 TF）
  void publish_odometry(const std_msgs::msg::Header & stamp_header, const Eigen::Matrix4f & pose) {
    try {
      if(tf_buffer_.canTransform(robot_odom_frame_id_, odom_child_frame_id_, rclcpp::Time(0))) {
        geometry_msgs::msg::TransformStamped map_wrt_frame = tf2::eigenToTransform(Eigen::Isometry3d(pose.inverse().cast<double>()));
        map_wrt_frame.header.stamp = stamp_header.stamp;
        map_wrt_frame.header.frame_id = odom_child_frame_id_;
        map_wrt_frame.child_frame_id = "map";

        geometry_msgs::msg::TransformStamped frame_wrt_odom = tf_buffer_.lookupTransform(robot_odom_frame_id_, odom_child_frame_id_,
                                                                                           rclcpp::Time(0), std::chrono::duration<double>(0.1));
        Eigen::Matrix4f frame2odom = tf2::transformToEigen(frame_wrt_odom).cast<float>().matrix();

        geometry_msgs::msg::TransformStamped map_wrt_odom;
        tf2::doTransform(map_wrt_frame, map_wrt_odom, frame_wrt_odom);
        tf2::Transform odom_wrt_map;
        tf2::fromMsg(map_wrt_odom.transform, odom_wrt_map);
        odom_wrt_map = odom_wrt_map.inverse();

        geometry_msgs::msg::TransformStamped odom_trans;
        odom_trans.transform = tf2::toMsg(odom_wrt_map);
        odom_trans.header.stamp = stamp_header.stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = robot_odom_frame_id_;
        tf_broadcaster_.sendTransform(odom_trans);
      } else {
        geometry_msgs::msg::TransformStamped odom_trans = tf2::eigenToTransform(Eigen::Isometry3d(pose.cast<double>()));
        odom_trans.header.stamp = stamp_header.stamp;
        odom_trans.header.frame_id = "map";
        odom_trans.child_frame_id = odom_child_frame_id_;
        tf_broadcaster_.sendTransform(odom_trans);
      }
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Transform error in publish_odometry: %s", ex.what());
    }
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp_header.stamp;
    odom.header.frame_id = "map";
    odom.child_frame_id = odom_child_frame_id_;
    odom.pose.pose = tf2::toMsg(Eigen::Isometry3d(pose.cast<double>()));
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.angular.z = 0.0;
    pose_pub_->publish(odom);
  }

  // 发布 scan matching 状态信息
  void publish_scan_matching_status(const std_msgs::msg::Header & header, pcl::PointCloud<PointT>::ConstPtr aligned) {
    msg::ScanMatchingStatus status;
    status.header = header;
    status.has_converged = registration_->hasConverged();
    status.matching_error = 0.0;
    double max_correspondence_dist = this->get_parameter("status_max_correspondence_dist").as_double();
    double max_valid_point_dist = this->get_parameter("status_max_valid_point_dist").as_double();
    int num_inliers = 0;
    int num_valid_points = 0;
    std::vector<int> k_indices;
    std::vector<float> k_sq_dists;
    for (size_t i = 0; i < aligned->size(); i++) {
      const auto & pt = aligned->at(i);
      if (pt.getVector3fMap().norm() > max_valid_point_dist) {
        continue;
      }
      num_valid_points++;
      registration_->getSearchMethodTarget()->nearestKSearch(pt, 1, k_indices, k_sq_dists);
      if(!k_sq_dists.empty() && k_sq_dists[0] < max_correspondence_dist * max_correspondence_dist) {
        status.matching_error += k_sq_dists[0];
        num_inliers++;
      }
    }
    if(num_inliers > 0)
      status.matching_error /= num_inliers;
    status.inlier_fraction = static_cast<float>(num_inliers) / std::max(1, num_valid_points);
    status.relative_pose = tf2::eigenToTransform(Eigen::Isometry3d(registration_->getFinalTransformation().cast<double>())).transform;
    status.prediction_labels.reserve(2);
    status.prediction_errors.reserve(2);
    if(pose_estimator_ && pose_estimator_->wo_prediction_error()) {
      std_msgs::msg::String label;
      label.data = "without_pred";
      status.prediction_labels.push_back(label);
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator_->wo_prediction_error().get().cast<double>())).transform);
    }
    if(pose_estimator_ && pose_estimator_->imu_prediction_error()) {
      std_msgs::msg::String label;
      label.data = use_imu_ ? "imu" : "motion_model";
      status.prediction_labels.push_back(label);
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator_->imu_prediction_error().get().cast<double>())).transform);
    }
    if(pose_estimator_ && pose_estimator_->odom_prediction_error()) {
      std_msgs::msg::String label;
      label.data = "odom";
      status.prediction_labels.push_back(label);
      status.prediction_errors.push_back(tf2::eigenToTransform(Eigen::Isometry3d(pose_estimator_->odom_prediction_error().get().cast<double>())).transform);
    }
    status_pub_->publish(status);
  }

private:
  // 成员变量
  std::string robot_odom_frame_id_;
  std::string odom_child_frame_id_;

  bool use_imu_;
  bool invert_acc_;
  bool invert_gyro_;
  bool use_global_localization_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr globalmap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<msg::ScanMatchingStatus>::SharedPtr status_pub_;

  rclcpp::Client<hdl_global_localization::srv::SetGlobalMap>::SharedPtr set_global_map_client_;
  rclcpp::Client<hdl_global_localization::srv::QueryGlobalLocalization>::SharedPtr query_global_localization_client_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr relocalize_srv_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::mutex imu_data_mutex_;
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> imu_data_;

  pcl::PointCloud<PointT>::Ptr globalmap_;
  std::shared_ptr<pcl::VoxelGrid<pcl::PointXYZI>> downsample_filter_;
  pcl::Registration<PointT, PointT>::Ptr registration_;

  std::mutex pose_estimator_mutex_;
  std::unique_ptr<hdl_localization::PoseEstimator> pose_estimator_;

  std::atomic_bool relocalizing_;
  std::unique_ptr<DeltaEstimater> delta_estimater_;

  pcl::PointCloud<PointT>::ConstPtr last_scan_;
};

}  // namespace hdl_localization

// 注册为组件
RCLCPP_COMPONENTS_REGISTER_NODE(hdl_localization::HdlLocalizationNode)

#ifndef POSE_ESTIMATOR_HPP
#define POSE_ESTIMATOR_HPP

#include <memory>
#include <boost/optional.hpp>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace kkl {
  namespace alg {
    template<typename T, class System> class UnscentedKalmanFilterX;
  }
}

namespace hdl_localization {

class PoseSystem;
class OdomSystem;

/**
 * @brief 基于扫描匹配的位姿估计器
 */
class PoseEstimator {
public:
  using PointT = pcl::PointXYZI;

  /**
   * @brief 构造函数
   * @param registration        配准方法
   * @param pos                 初始位置
   * @param quat                初始姿态
   * @param cool_time_duration  在“冷却时间”期间不进行预测，单位为秒
   */
  PoseEstimator(pcl::Registration<PointT, PointT>::Ptr& registration,
                const Eigen::Vector3f& pos,
                const Eigen::Quaternionf& quat,
                double cool_time_duration = 1.0);
  ~PoseEstimator();

  /**
   * @brief 预测
   * @param stamp    时间戳
   */
  void predict(const rclcpp::Time& stamp);

  /**
   * @brief 预测
   * @param stamp    时间戳
   * @param acc      加速度
   * @param gyro     角速度
   */
  void predict(const rclcpp::Time& stamp, const Eigen::Vector3f& acc, const Eigen::Vector3f& gyro);

  /**
   * @brief 更新基于里程计的位姿估计状态
   */
  void predict_odom(const Eigen::Matrix4f& odom_delta);

  /**
   * @brief 校正
   * @param cloud   输入点云
   * @return 对齐到全局地图的点云
   */
  pcl::PointCloud<PointT>::Ptr correct(const rclcpp::Time& stamp, const pcl::PointCloud<PointT>::ConstPtr& cloud);

  /* getter 接口 */
  rclcpp::Time last_correction_time() const;

  Eigen::Vector3f pos() const;
  Eigen::Vector3f vel() const;
  Eigen::Quaternionf quat() const;
  Eigen::Matrix4f matrix() const;

  Eigen::Vector3f odom_pos() const;
  Eigen::Quaternionf odom_quat() const;
  Eigen::Matrix4f odom_matrix() const;

  const boost::optional<Eigen::Matrix4f>& wo_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& imu_prediction_error() const;
  const boost::optional<Eigen::Matrix4f>& odom_prediction_error() const;

private:
  rclcpp::Time init_stamp;             // 初始化时的时间
  rclcpp::Time prev_stamp;             // 上一次更新的时间
  rclcpp::Time last_correction_stamp;  // 最近一次校正的时间
  double cool_time_duration;           // 冷却时间

  Eigen::MatrixXf process_noise;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, PoseSystem>> ukf;
  std::unique_ptr<kkl::alg::UnscentedKalmanFilterX<float, OdomSystem>> odom_ukf;

  Eigen::Matrix4f last_observation;
  boost::optional<Eigen::Matrix4f> wo_pred_error;
  boost::optional<Eigen::Matrix4f> imu_pred_error;
  boost::optional<Eigen::Matrix4f> odom_pred_error;

  pcl::Registration<PointT, PointT>::Ptr registration;
};

}  // namespace hdl_localization

#endif  // POSE_ESTIMATOR_HPP

#ifndef ODOM_SYSTEM_HPP
#define ODOM_SYSTEM_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <kkl/alg/unscented_kalman_filter.hpp>  // 请确保该头文件在 ROS2 环境下可用

namespace hdl_localization {

/**
 * @brief 本类用于根据机器人里程计建模传感器位姿估计
 * @note 状态向量 state = [px, py, pz, qw, qx, qy, qz]
 *       观测向量 observation = [px, py, pz, qw, qx, qy, qz]
 *       也可以考虑采用指数映射(expmap)方式
 */
class OdomSystem {
public:
  using T = float;
  using Vector3t = Eigen::Matrix<T, 3, 1>;
  using Vector4t = Eigen::Matrix<T, 4, 1>;
  using Matrix4t = Eigen::Matrix<T, 4, 4>;
  using VectorXt = Eigen::Matrix<T, Eigen::Dynamic, 1>;
  using Quaterniont = Eigen::Quaternion<T>;

public:
  // 系统方程
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
    Matrix4t pt = Matrix4t::Identity();
    pt.block<3, 1>(0, 3) = Vector3t(state[0], state[1], state[2]);
    pt.block<3, 3>(0, 0) = Quaterniont(state[3], state[4], state[5], state[6]).normalized().toRotationMatrix();

    Matrix4t delta = Matrix4t::Identity();
    delta.block<3, 1>(0, 3) = Vector3t(control[0], control[1], control[2]);
    delta.block<3, 3>(0, 0) = Quaterniont(control[3], control[4], control[5], control[6]).normalized().toRotationMatrix();

    Matrix4t pt_ = pt * delta;
    Quaterniont quat_(pt_.block<3, 3>(0, 0));

    VectorXt next_state(7);
    next_state.head<3>() = pt_.block<3, 1>(0, 3);
    next_state.tail<4>() = Vector4t(quat_.w(), quat_.x(), quat_.y(), quat_.z());

    return next_state;
  }

  // 观测方程
  VectorXt h(const VectorXt& state) const {
    return state;
  }
};

}  // namespace hdl_localization

#endif  // ODOM_SYSTEM_HPP

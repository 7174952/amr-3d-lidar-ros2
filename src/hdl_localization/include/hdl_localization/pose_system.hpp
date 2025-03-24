#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief 使用 UKF 估计的系统定义
 * @note 状态向量 state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 */
class PoseSystem {
public:
  using T = float;
  using Vector3t = Eigen::Matrix<T, 3, 1>;
  using Matrix4t = Eigen::Matrix<T, 4, 4>;
  using VectorXt = Eigen::Matrix<T, Eigen::Dynamic, 1>;
  using Quaterniont = Eigen::Quaternion<T>;

public:
  PoseSystem() : dt(0.01) {}

  // 系统方程（无输入）
  VectorXt f(const VectorXt& state) const {
    VectorXt next_state(16);

    Vector3t pt = state.middleRows(0, 3);
    Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]);
    qt.normalize();

    Vector3t acc_bias = state.middleRows(10, 3);
    Vector3t gyro_bias = state.middleRows(13, 3);

    // 位置
    next_state.middleRows(0, 3) = pt + vt * dt;

    // 速度
    next_state.middleRows(3, 3) = vt;

    // 姿态
    Quaterniont qt_ = qt;
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();
    next_state.middleRows(10, 3) = state.middleRows(10, 3);  // 加速度偏置保持不变
    next_state.middleRows(13, 3) = state.middleRows(13, 3);  // 陀螺仪偏置保持不变

    return next_state;
  }

  // 系统方程（带输入）
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
    VectorXt next_state(16);

    Vector3t pt = state.middleRows(0, 3);
    Vector3t vt = state.middleRows(3, 3);
    Quaterniont qt(state[6], state[7], state[8], state[9]);
    qt.normalize();

    Vector3t acc_bias = state.middleRows(10, 3);
    Vector3t gyro_bias = state.middleRows(13, 3);

    Vector3t raw_acc = control.middleRows(0, 3);
    Vector3t raw_gyro = control.middleRows(3, 3);

    // 位置
    next_state.middleRows(0, 3) = pt + vt * dt;

    // 速度
    Vector3t g(0.0f, 0.0f, 9.80665f);
    Vector3t acc_ = raw_acc - acc_bias;
    Vector3t acc = qt * acc_;
    next_state.middleRows(3, 3) = vt + (acc - g) * dt;
    // 若加速度噪声较大，也可直接使用 vt

    // 姿态
    Vector3t gyro = raw_gyro - gyro_bias;
    Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);
    dq.normalize();
    Quaterniont qt_ = (qt * dq).normalized();
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();

    next_state.middleRows(10, 3) = state.middleRows(10, 3);  // 加速度偏置保持不变
    next_state.middleRows(13, 3) = state.middleRows(13, 3);  // 陀螺仪偏置保持不变

    return next_state;
  }

  // 观测方程
  VectorXt h(const VectorXt& state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();

    return observation;
  }

  double dt;
};

}  // namespace hdl_localization

#endif  // POSE_SYSTEM_HPP

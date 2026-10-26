/**
 * @file quaternion.cpp
 * @author qingyu
 * @brief 基于四元数的 IMU 姿态扩展 Kalman 滤波器实现
 * @version 0.1
 * @date 2026-06-28
 */

#pragma message "Compiling Algorithm/Filter/QuaternionEkf"

#include "quaternion.hpp"
#include "imu_device_layer.hpp"
#include <zephyr/kernel.h>
#include <math.h>
#include <algorithm>
#include <Eigen/Dense>

namespace {

constexpr float kRadToDeg = 57.295779513f;
constexpr float kYawBiasStaticThreshold = 0.02f;
constexpr uint16_t kYawBiasStaticSamples = 100;

} // namespace

namespace alg::attitude {

/**
 * @brief 初始化滤波器
 * @param config 配置参数（Qq/Qb/R/lambda/alpha/chi2_th）
 */
void QuaternionEkf::Init(const Config& config)
{
    config_ = config;
    if (config_.lambda > 1.0f) config_.lambda = 1.0f;

    state_ = {};
    state_.init    = true;
    state_.q[0]    = 1.0f;
    state_.Qq      = config_.Qq;
    state_.Qb      = config_.Qb;
    state_.R       = config_.R;
    state_.lambda  = config_.lambda;
    state_.alpha   = config_.alpha;
    state_.chi2_th = config_.chi2_th;

    ekf_.SetSystemFunc(SystemFunc);
    ekf_.SetObserveFunc(ObserveFunc);

    EKF::Cov Q = EKF::Cov::Zero();
    Q(0,0) = Q(1,1) = Q(2,2) = Q(3,3) = config_.Qq;
    Q(4,4) = Q(5,5) = config_.Qb;

    EKF::ObsCov R = EKF::ObsCov::Identity() * config_.R;

    EKF::Cov P0;
    P0 << 100000.0f, 0.1f,      0.1f,      0.1f,      0.1f,     0.1f,
          0.1f,      100000.0f, 0.1f,      0.1f,      0.1f,     0.1f,
          0.1f,      0.1f,      100000.0f, 0.1f,      0.1f,     0.1f,
          0.1f,      0.1f,      0.1f,      100000.0f, 0.1f,     0.1f,
          0.1f,      0.1f,      0.1f,      0.1f,      100.0f,   0.1f,
          0.1f,      0.1f,      0.1f,      0.1f,      0.1f,     100.0f;

    ekf_.Init(Q, R, P0);

    EKF::State x0;
    x0 << 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
    ekf_.SetState(x0);

    state_.q[0] = 1.0f;
}

/**
 * @brief 用首帧加速度做倾角对齐
 *
 * 计算 roll/pitch（yaw=0），同时把 P 从初始化大值降为合理小值。
 *
 * @param sample 第一帧 IMU 样本
 */
void QuaternionEkf::InitFromAccel(const Sample& sample)
{
    const float a_norm = sqrtf(sample.accel[0] * sample.accel[0] +
                               sample.accel[1] * sample.accel[1] +
                               sample.accel[2] * sample.accel[2]);
    if (a_norm > 0.0f)
    {
        const float nx = sample.accel[0] / a_norm;
        const float ny = sample.accel[1] / a_norm;
        const float nz = sample.accel[2] / a_norm;

        const float roll  = atan2f(ny, nz);
        const float pitch = -asinf(nx);

        const float cr = cosf(roll * 0.5f),  sr = sinf(roll * 0.5f);
        const float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);

        EKF::State x0;
        x0(0) =  cp * cr;
        x0(1) =  cp * sr;
        x0(2) =  sp * cr;
        x0(3) = -sp * sr;
        x0(4) = 0.0f;
        x0(5) = 0.0f;
        ekf_.SetState(x0);

        {
            auto P = ekf_.GetP();
            P.block<4,4>(0,0).setIdentity();
            P.block<4,4>(0,0) *= 1e-3f;
            P.block<2,2>(4,4).setIdentity();
            P.block<2,2>(4,4) *= 1e-1f;
            P.block<4,2>(0,4).setZero();
            P.block<2,4>(4,0).setZero();
            ekf_.SetCovariance(P);
        }

        state_.q[0] = x0(0); state_.q[1] = x0(1);
        state_.q[2] = x0(2); state_.q[3] = x0(3);
    }

    state_.a[0] = sample.accel[0];
    state_.a[1] = sample.accel[1];
    state_.a[2] = sample.accel[2];
}

/**
 * @brief 姿态更新
 *
 * 流程：
 *   去偏 + Q → 倾角对齐（首帧）→ 加速度 LPF + 观测 z
 *   → 稳定性 + bgz 跟踪 → EKF 预测/校正/门控 → 欧拉角
 *
 * acc 观测不修正 yaw（q3 保持预测值）。
 *
 * @param sample IMU 样本（gyro/accel/dt）
 */
void QuaternionEkf::Update(const Sample& sample)
{
    if (!state_.init) Init(Config{});

    const float dt = sample.dt;
    if (dt <= 0.0f) return;
    state_.dt = dt;

    // 去偏 + 过程噪声
    state_.w[0] = sample.gyro[0] - state_.bg[0];
    state_.w[1] = sample.gyro[1] - state_.bg[1];
    state_.w[2] = sample.gyro[2] - state_.bg[2];

    {
        EKF::Cov Q = EKF::Cov::Zero();
        Q(0,0) = Q(1,1) = Q(2,2) = Q(3,3) = state_.Qq * dt;
        Q(4,4) = Q(5,5) = state_.Qb * dt;
        ekf_.SetQ(Q);
    }

    if (state_.upd_cnt == 0) InitFromAccel(sample);

    // 加速度 LPF + 观测 z
    {
        const float den = dt + state_.alpha;
        if (den > 0.0f) {
            const float lpf_a = state_.alpha / den;
            const float lpf_b = dt / den;
            state_.a[0] = state_.a[0] * lpf_a + sample.accel[0] * lpf_b;
            state_.a[1] = state_.a[1] * lpf_a + sample.accel[1] * lpf_b;
            state_.a[2] = state_.a[2] * lpf_a + sample.accel[2] * lpf_b;
        } else {
            state_.a[0] = sample.accel[0];
            state_.a[1] = sample.accel[1];
            state_.a[2] = sample.accel[2];
        }

        const float a_inv = FastInverseSqrt(state_.a[0] * state_.a[0] + state_.a[1] * state_.a[1] + state_.a[2] * state_.a[2]);

        EKF::Obs z;
        z << state_.a[0] * a_inv, state_.a[1] * a_inv, state_.a[2] * a_inv;
        ekf_.SetZ(z);
        state_.a_norm = a_inv > 0.0f ? 1.0f / a_inv : 0.0f;
    }

    // 稳定性 + Z 轴零偏跟踪
    {
        const float w_inv = FastInverseSqrt(state_.w[0] * state_.w[0] + state_.w[1] * state_.w[1] + state_.w[2] * state_.w[2]);
        state_.w_norm = w_inv > 0.0f ? 1.0f / w_inv : 0.0f;

        state_.stable = state_.w_norm < config_.w_stable_th && state_.a_norm > config_.a_ref - config_.a_tol && 
                        state_.a_norm < config_.a_ref + config_.a_tol;

        const bool yaw_bias_static = state_.stable && fabsf(state_.w[2]) < kYawBiasStaticThreshold;

        if (yaw_bias_static) {
            state_.yaw_bias_static_cnt = std::min<uint16_t>(static_cast<uint16_t>(state_.yaw_bias_static_cnt + 1U), kYawBiasStaticSamples);
        } else {
            state_.yaw_bias_static_cnt = 0;
        }

        if (state_.yaw_bias_static_cnt >= kYawBiasStaticSamples)
        {
            // 滑动均值：累积 gz，每 500 帧 (~2.5s) 更新一次 bgz
            state_.bgz_sum += sample.gyro[2];
            state_.bgz_cnt++;
            if (state_.bgz_cnt >= 500) {
                state_.bg[2] = state_.bgz_sum / static_cast<float>(state_.bgz_cnt);
                state_.bgz_sum = 0.0f;
                state_.bgz_cnt = 0;
            }
        } else {
            state_.bgz_sum = 0.0f;
            state_.bgz_cnt = 0;
        }

        state_.w[2] = sample.gyro[2] - state_.bg[2];
    }

    // EKF 预测 + 校正（首帧跳过）
    float yaw_pred = 0.0f;
    {
        EKF::Ctrl u;
        u << state_.w[0] * dt, state_.w[1] * dt, state_.w[2] * dt, dt;
        ekf_.SetU(u);

        if (state_.upd_cnt > 0)
        {
            ekf_.Predict();

            {
                const auto& xm = ekf_.GetXMinus();
                const float q0 = xm(0), q1 = xm(1), q2 = xm(2), q3 = xm(3);
                yaw_pred = atan2f(2.0f * (q0 * q3 + q1 * q2),
                                  2.0f * (q0 * q0 + q1 * q1) - 1.0f) * kRadToDeg;
            }

            // 渐消因子：膨胀零偏协方差
            {
                auto& P = ekf_.PMinus();
                P(4,4) = std::min(P(4,4) / state_.lambda, config_.Pb_limit);
                P(5,5) = std::min(P(5,5) / state_.lambda, config_.Pb_limit);
            }

            ekf_.SetGainScale(state_.k_scale);
            ekf_.Update();

            // 观测不修正 yaw（同参考库）：q3 correction 置零
            {
                auto x = ekf_.GetX();
                const auto& xm = ekf_.GetXMinus();
                x(3) = xm(3);
                ekf_.SetState(x);
            }

            state_.chi2 = ekf_.GetChi2();

            if (state_.chi2 < 0.0f || !std::isfinite(state_.chi2)) {
                ekf_.FallbackToPrediction();
                state_.chi2 = 0.0f;
            }

            // 卡方门控
            {
                const float chi2 = state_.chi2;
                const float th   = state_.chi2_th;

                if (chi2 < 0.5f * th) state_.converg = true;

                if (chi2 > th) {
                    ekf_.FallbackToPrediction();
                    state_.chi2 = 0.0f;
                } else {
                    if (state_.converg && chi2 > 0.1f * th)
                        state_.k_scale = std::clamp((th - chi2) / (0.9f * th), 0.0f, 1.0f);
                    else
                        state_.k_scale = 1.0f;
                }
            }
        }
    }

    // 提取状态 → 欧拉角（参考库：不归一化、不重建，直接用后验）
    {
        const auto& x = ekf_.GetX();

        state_.q[0]  = x(0); state_.q[1]  = x(1);
        state_.q[2]  = x(2); state_.q[3]  = x(3);
        state_.bg[0] = x(4); state_.bg[1] = x(5);

        const float q0 = state_.q[0], q1 = state_.q[1];
        const float q2 = state_.q[2], q3 = state_.q[3];

        state_.yaw   = atan2f(2.0f * (q0 * q3 + q1 * q2),
                              2.0f * (q0 * q0 + q1 * q1) - 1.0f) * kRadToDeg;
        state_.roll  = atan2f(2.0f * (q0 * q1 + q2 * q3),
                              2.0f * (q0 * q0 + q3 * q3) - 1.0f) * kRadToDeg;

        const float pitch_input = std::clamp(2.0f * (q0 * q2 - q1 * q3), -1.0f, 1.0f);
        state_.pitch = asinf(pitch_input) * kRadToDeg;

        const float yaw_diff = state_.yaw - state_.yaw_prev;

        if (yaw_diff > 180.0f) {
            state_.yaw_rnd--;
        } else if (yaw_diff < -180.0f) {
            state_.yaw_rnd++;
        }

        state_.yaw_sum  = 360.0f * state_.yaw_rnd + state_.yaw;
        state_.yaw_prev = state_.yaw;
    }

    // 诊断：静态时看 yaw 变化来源
    // if (state_.stable && state_.upd_cnt % 200 == 0) {
    //     printk("st: yaw=%.4f bgz=%.6f gz=%.6f wz=%.6f dt=%.6f\n",
    //         (double)state_.yaw,
    //         (double)state_.bg[2],
    //         (double)sample.gyro[2],
    //         (double)state_.w[2],
    //         (double)dt);
    // }

    state_.upd_cnt++;

}

/**
 * @brief EKF 系统模型回调
 *
 * 四元数 RK1 积分 + bg 随机游走。
 *
 * @param x_in  当前状态 [q0..q3, bgx, bgy]
 * @param u_in  控制输入 [ωx·dt, ωy·dt, ωz·dt, dt]
 * @param x_out 先验状态
 * @param F_out Jacobian 6×6
 */
void QuaternionEkf::SystemFunc(const EkfSt& x_in, const EKF::Ctrl& u_in, EkfSt& x_out, EkfCov& F_out)
{
    const float q0 = x_in(0), q1 = x_in(1), q2 = x_in(2), q3 = x_in(3);

    const float hx  = 0.5f * u_in(0);
    const float hy  = 0.5f * u_in(1);
    const float hz  = 0.5f * u_in(2);
    const float hdt = 0.5f * u_in(3);

    x_out(0) = q0 + (-q1 * hx - q2 * hy - q3 * hz);
    x_out(1) = q1 + ( q0 * hx + q2 * hz - q3 * hy);
    x_out(2) = q2 + ( q0 * hy - q1 * hz + q3 * hx);
    x_out(3) = q3 + ( q0 * hz + q1 * hy - q2 * hx);
    NormalizeQuaternion(x_out);

    x_out(4) = x_in(4);
    x_out(5) = x_in(5);

    F_out.setIdentity();
    F_out(0, 1) = -hx;   F_out(0, 2) = -hy;   F_out(0, 3) = -hz;
    F_out(1, 0) =  hx;   F_out(1, 2) =  hz;   F_out(1, 3) = -hy;
    F_out(2, 0) =  hy;   F_out(2, 1) = -hz;   F_out(2, 3) =  hx;
    F_out(3, 0) =  hz;   F_out(3, 1) =  hy;   F_out(3, 2) = -hx;

    F_out(0, 4) =  q1 * hdt;   F_out(0, 5) =  q2 * hdt;
    F_out(1, 4) = -q0 * hdt;   F_out(1, 5) =  q3 * hdt;
    F_out(2, 4) = -q3 * hdt;   F_out(2, 5) = -q0 * hdt;
    F_out(3, 4) =  q2 * hdt;   F_out(3, 5) = -q1 * hdt;
}

/**
 * @brief EKF 观测模型回调
 *
 * 预测归一化重力方向，计算观测 Jacobian。
 *
 * @param x_in  先验状态 x⁻
 * @param z_out 预测观测值（3×1 归一化重力方向）
 * @param H_out Jacobian H（3×6，零偏列补零）
 */
void QuaternionEkf::ObserveFunc(const EkfSt& x_in, EKF::Obs& z_out, EKF::ObsMat& H_out)
{
    const float q0 = x_in(0), q1 = x_in(1), q2 = x_in(2), q3 = x_in(3);

    z_out(0) = 2.0f * (q1 * q3 - q0 * q2);
    z_out(1) = 2.0f * (q0 * q1 + q2 * q3);
    z_out(2) = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    const float dq0 = 2.0f * q0, dq1 = 2.0f * q1;
    const float dq2 = 2.0f * q2, dq3 = 2.0f * q3;

    H_out.setZero();
    H_out(0, 0) = -dq2;   H_out(0, 1) =  dq3;   H_out(0, 2) = -dq0;   H_out(0, 3) =  dq1;
    H_out(1, 0) =  dq1;   H_out(1, 1) =  dq0;   H_out(1, 2) =  dq3;   H_out(1, 3) =  dq2;
    H_out(2, 0) =  dq0;   H_out(2, 1) = -dq1;   H_out(2, 2) = -dq2;   H_out(2, 3) =  dq3;
}

/**
 * @brief 单位化四元数
 */
void QuaternionEkf::NormalizeQuaternion(EkfSt& x)
{
    const float q_inv = FastInverseSqrt(x(0) * x(0) + x(1) * x(1) + x(2) * x(2) + x(3) * x(3));
    if (q_inv <= 0.0f) {
        x(0) = 1.0f; x(1) = 0.0f; x(2) = 0.0f; x(3) = 0.0f;
        return;
    }
    x(0) *= q_inv; x(1) *= q_inv; x(2) *= q_inv; x(3) *= q_inv;
}

/**
 * @brief 快速倒数平方根
 */
float QuaternionEkf::FastInverseSqrt(float x)
{
    if (x <= 0.0f) return 0.0f;
    const float half_x = 0.5f * x;
    union { float f; uint32_t i; } value = { x };
    value.i = 0x5f375a86u - (value.i >> 1);
    value.f = value.f * (1.5f - (half_x * value.f * value.f));
    return value.f;
}

} // namespace alg::attitude

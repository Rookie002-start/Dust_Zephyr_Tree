/**
 * @file quaternion.hpp
 * @author qingyu
 * @brief 基于四元数的 IMU 姿态扩展 Kalman 滤波器
 * @version 0.1
 * @date 2026-06-28
 */

#pragma once

#include "kalman_ekf.hpp"
#include <stdint.h>

struct Sample;

namespace alg::attitude {

/**
 * @brief 基于四元数的 IMU 姿态扩展 Kalman 滤波器。
 *
 * @par 状态空间 (n = 6)
 *   x = [q0, q1, q2, q3, bg_x, bg_y]ᵀ
 *     - q: 单位四元数 (姿态)
 *     - bg: 陀螺零偏 (仅 x/y 可观)
 *
 * @par 输入空间 (p = 3)
 *   u = [ωx·dt, ωy·dt, ωz·dt, dt]ᵀ
 *   — 去偏角速度积分与采样周期
 *
 * @par 观测空间 (m = 3)
 *   z = a / |a|
 *   — 归一化加速度方向（重力方向在机体坐标系下的投影）
 *
 * @par 系统模型 (RK1 四元数积分)
 *   q'  = q + ½ · Ω(ω) · q · dt
 *   bg' = bg
 *   其中 Ω(ω) 为角速度的斜对称矩阵：
 *     Ω(ω) = [ 0,  -ωx, -ωy, -ωz;
 *              ωx,  0,   ωz, -ωy;
 *              ωy, -ωz,  0,   ωx;
 *              ωz,  ωy, -ωx,  0  ]
 *
 * @par 观测模型 (重力方向预测)
 *   h(q) = [ 2·(q1·q3 - q0·q2);
 *            2·(q0·q1 + q2·q3);
 *            q0² - q1² - q2² + q3² ]
 *   — 将重力向量从导航系旋转到机体系
 *
 * @par 滤波引擎
 *   内部使用 ExtendedKalman<6, 3, 3>：
 *   - SystemFunc:  四元数运动学积分 + F Jacobian
 *   - ObserveFunc: 重力方向预测 + H Jacobian
 */
class QuaternionEkf final
{
public:
    /**
     * @brief 滤波器配置参数
     */
    struct Config {
        // 核心调参
        float Qq            = 10.0f;        // 四元数过程噪声，越大越信任陀螺
        float Qb            = 0.001f;       // 零偏过程噪声，越大零偏跟踪越快
        float R             = 1000000.0f;   // 四元数过程噪声，越大越不信任陀螺预测，越依赖观测修正

        // 高级调参
        float lambda        = 1.0f;         // 渐消因子（≤1），<1 防零偏协方差收敛死

        // 前处理
        float alpha         = 0.0f;         // 加速度 LPF 系数

        // 卡方检验门控
        float chi2_th       = 1e-8f;        // 卡方检验阈值，χ² 超此值认为观测异常
        uint32_t div_limit  = 50;           // 发散判定次数阈值

        // 稳定检测
        float w_stable_th   = 0.3f;         // 角速度稳定阈值 (rad/s)
        float a_ref         = 9.8f;         // 加速度稳定参考值，重力加速度常量
        float a_tol         = 0.5f;         // 加速度稳定容差

        // 安全限幅
        float bias_limit    = 1e-2f;        // 零偏单步修正上限，防单帧跳变过大
        float Pb_limit      = 10000.0f;     // 零偏协方差上限，防发散失控
    };

    /**
     * @brief 滤波器运行时状态
     */
    struct State {
        bool init    = false;               // 滤波器已初始化
        bool converg = false;               // 已收敛标志
        bool stable  = false;               // IMU 稳定标志

        uint64_t err_cnt  = 0;              // 连续发散计数
        uint64_t upd_cnt  = 0;              // 更新次数
        uint16_t yaw_bias_static_cnt = 0;   // 连续静止样本数
        float   bgz_sum = 0.0f;             // 静止期 gz 累积和（滑动均值用）
        uint32_t bgz_cnt = 0;               // 静止期 gz 样本计数

        float q [4]   {1.0f, 0.0f, 0.0f, 0.0f};    // 姿态四元数
        float bg[3]   {0.0f, 0.0f, 0.0f};              // 陀螺零偏
        float w [3]   {0.0f, 0.0f, 0.0f};              // 去偏角速度 ω = ω_raw - bg
        float a [3]   {0.0f, 0.0f, 0.0f};              // 加速度（LPF 后）
        float oc[3]   {0.0f, 0.0f, 0.0f};              // 方向余弦

        float alpha   = 0.0f;               // 加速度 LPF 系数
        float w_norm  = 0.0f;               // 角速度幅值 |ω|
        float a_norm  = 0.0f;               // 加速度幅值 |a|
        float k_scale = 1.0f;               // 自适应增益系数

        float roll    = 0.0f;               // 滚转角 (deg)
        float pitch   = 0.0f;               // 俯仰角 (deg)
        float yaw     = 0.0f;               // 偏航角 (deg)
        float yaw_sum = 0.0f;               // 偏航累积角 (deg)

        float Qq      = 10.0f;              // 四元数过程噪声
        float Qb      = 0.001f;             // 零偏过程噪声
        float R       = 1000000.0f;         // 观测噪声
        float dt      = 0.0f;               // 采样周期 (s)
        float chi2    = 0.0f;               // 卡方检验值 χ²
        float chi2_th = 1e-8f;              // 卡方检验阈值
        float lambda  = 1.0f;               // 渐消因子

        int16_t yaw_rnd  = 0;               // 偏航绕圈计数
        float   yaw_prev = 0.0f;            // 上一帧偏航角
    };

    QuaternionEkf() = default;

    void Init(const Config& config);
    void InitFromAccel(const Sample& sample);
    void Update(const Sample& sample);

    /**
     * @brief 获取滤波器状态
     */
    const State& GetState() const { return state_; }
    State&       GetState()       { return state_; }

private:
    static constexpr uint8_t kStateSize       = 6;  // 状态维数 (4 四元数 + 2 零偏)
    static constexpr uint8_t kMeasurementSize = 3;  // 观测维数 (归一化加速度)
    static constexpr uint8_t kControlSize     = 4;  // 控制输入 [ω·dt, dt]

    // 内部 EKF 类型别名
    using EKF    = alg::filter::ExtendedKalman<kStateSize, kMeasurementSize, kControlSize>;
    using EkfSt  = EKF::State;                      // EKF 状态向量类型  6×1
    using EkfCov = EKF::Cov;                        // EKF 协方差类型    6×6

    Config config_;                                 // 配置参数
    State  state_ {};                               // 运行时状态
    EKF    ekf_ {};                                 // EKF 滤波引擎

    static void  SystemFunc(const EkfSt& x_in, const EKF::Ctrl& u_in, EkfSt& x_out, EkfCov& F_out);
    static void  ObserveFunc(const EkfSt& x_in, EKF::Obs& z_out, EKF::ObsMat& H_out);
    static void  NormalizeQuaternion(EkfSt& x);
    static float FastInverseSqrt(float x);
};

} // namespace alg::attitude

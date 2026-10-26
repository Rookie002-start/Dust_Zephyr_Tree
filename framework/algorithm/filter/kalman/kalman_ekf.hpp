/**
 * @file kalman_ekf.hpp
 * @author qingyu
 * @brief 模板化扩展卡尔曼滤波器（EKF）
 * @version 0.1
 * @date 2026-06-28
 *
 * @par 与 Kalman（标准 KF）的区别
 *       Kalman 使用恒定的 A/H 矩阵，EKF 每周期通过用户回调
 *       计算非线性系统/观测函数的 Jacobian F/H，其余算法一致。
 *
 * @par 使用示例（弹簧阻尼系统）
 * @code
 *   // 系统函数: xₖ = f(xₖ₋₁, uₖ₋₁) + w
 *   //   x[0] = pos    x[1] = vel
 *   //   f[0] = x[0] + dt·x[1]
 *   //   f[1] = x[1] + dt·(-k·x[0] - c·x[1])
 *   //
 *   //   F = ∂f/∂x
 *   //     = [1       dt    ]
 *   //       [-k·dt  1-c·dt ]
 *
 *   static void sys_fn(
 *       const ExtendedKalman<2,1>::State& x,
 *       const ExtendedKalman<2,1>::Ctrl&  u,
 *       ExtendedKalman<2,1>::State& x_out,
 *       ExtendedKalman<2,1>::Cov&   F_out)
 *   {
 *       constexpr float k  = 10.0f;
 *       constexpr float c  = 0.5f;
 *       constexpr float dt = 0.01f;
 *       x_out(0) = x(0) + dt * x(1);
 *       x_out(1) = x(1) + dt * (-k * x(0) - c * x(1));
 *       F_out << 1.0f, dt,
 *                -k * dt, 1.0f - c * dt;
 *   }
 *
 *   ExtendedKalman<2, 1> ekf;
 *   ekf.SetSystemFunc(sys_fn);
 *   ekf.Init(Q, R, P0);
 *
 *   // 每控制周期
 *   ekf.SetZ(z);
 *   ekf.Predict();
 *   ekf.Update();
 *
 *   float pos = ekf.GetX()(0);
 * @endcode
 */

#pragma once

#pragma message "Compiling Algorithm/Filter/ExtendedKalman"

#include <Eigen/Dense>

namespace alg::filter {

/**
 * @brief 扩展卡尔曼滤波器模板
 *
 * 使用于系统模型 xₖ = f(xₖ₋₁, uₖ₋₁) + w 和观测模型 zₖ = h(xₖ) + v
 * 均为非线性的场合（如四元数姿态估计）。
 *
 * 用户通过 SetSystemFunc / SetObserveFunc 注册回调：
 *   - 回调输出 x⁻ = f(x, u) 和 Jacobian F = ∂f/∂x
 *   - 回调输出 z_pred = h(x⁻) 和 Jacobian H = ∂h/∂x
 *
 * 滤波流程（同标准 KF，仅 F/H 每周期更新）:
 *   - 预测: x⁻ = f(x, u)        P⁻ = F·P·Fᵀ + Q
 *   - 更新: K = P⁻·Hᵀ·(S)⁻¹    x = x⁻ + K·(z - z_pred)    P = (I - K·H)·P⁻
 *
 * @tparam nx  状态维数
 * @tparam nz  观测维数
 * @tparam nu  控制输入维数（默认 0 = 无控制输入）
 */
template <uint8_t nx, uint8_t nz, uint8_t nu = 0>
class ExtendedKalman
{
public:
    // 类型别名
    using State   = Eigen::Matrix<float, nx, 1>;    // n×1  后验状态 x
    using Cov     = Eigen::Matrix<float, nx, nx>;   // n×n  协方差矩阵 P
    using Obs     = Eigen::Matrix<float, nz, 1>;    // m×1  观测值 z
    using ObsCov  = Eigen::Matrix<float, nz, nz>;   // m×m  观测噪声协方差 R
    using Ctrl    = Eigen::Matrix<float, nu, 1>;    // p×1  控制输入 u
    using CtrlMat = Eigen::Matrix<float, nx, nu>;   // n×p  控制矩阵 B
    using ObsMat  = Eigen::Matrix<float, nz, nx>;   // m×n  观测矩阵 H
    using Gain    = Eigen::Matrix<float, nx, nz>;   // n×m  卡尔曼增益 K

    /**
     * @brief 系统模型回调
     *        f(x, u) → (x⁻, F)
     * @param[in]  x_in   当前状态
     * @param[in]  u_in   控制输入
     * @param[out] x_out  先验状态 x⁻
     * @param[out] F_out  Jacobian F = ∂f/∂x
     */
    using SystemFunc = void (*)(const State& x_in, const Ctrl& u_in, State& x_out, Cov& F_out);

    /**
     * @brief 观测模型回调
     *        h(x⁻) → (z_pred, H)
     * @param[in]  x_in   先验状态 x⁻
     * @param[out] z_out  预测观测值
     * @param[out] H_out  Jacobian H = ∂h/∂x
     */
    using ObserveFunc = void (*)(const State& x_in, Obs& z_out, ObsMat& H_out);

    // 初始化
    /**
     * @brief 初始化滤波器参数
     * @param Q            过程噪声协方差   n×n
     * @param R            观测噪声协方差   m×m
     * @param P0           初始协方差       n×n（默认 Identity）
     * @param P_min_diag   协方差对角线最小值（默认 0，不钳制）
     * @param B            控制矩阵         n×p（nu=0 时不使用）
     */
    void Init(const Cov&     Q,
              const ObsCov&  R,
              const Cov&     P0         = Cov::Identity(),
              const State&   P_min_diag = State::Zero(),
              const CtrlMat& B          = CtrlMat::Zero())
    {
        Q_  = Q;
        R_  = R;
        B_  = B;
        P_  = P0;
        P_min_diag_ = P_min_diag;
        z_.setZero();
        u_.setZero();
        z_pred_.setZero();
        x_.setZero();
        x_minus_.setZero();
        P_minus_ = P_;
        F_.setZero();
        H_.setZero();
        HT_.setZero();
        K_.setZero();
        S_.setZero();
    }

    // 预测 / 校正
    /**
     * @brief 预测步：x⁻ = f(x, u),  P⁻ = F·P·Fᵀ + Q
     *
     * 调系统回调计算 x⁻ 和 Jacobian F，nu > 0 时叠加线性控制项 B·u。
     */
    void Predict()
    {
        system_func_(x_, u_, x_minus_, F_);
        if constexpr (nu > 0) {
            x_minus_.noalias() += B_ * u_;
        }
        P_minus_.noalias() = F_ * P_ * F_.transpose();
        P_minus_ += Q_;
        MakeSymmetric(P_minus_);
    }

    /**
     * @brief 预测步（同时设置控制输入）
     * @param u  控制输入
     */
    void Predict(const Ctrl& u)
    {
        u_ = u;
        Predict();
    }

    /**
     * @brief 更新步：K = P⁻·Hᵀ·(S)⁻¹, x = x⁻ + K·(z - z_pred), P = (I - K·H)·P⁻
     *
     * 调观测回调计算 z_pred 和 Jacobian H。
     * 使用 LDLᵀ 分解求解 S = H·P⁻·Hᵀ + R，数值稳定性优于直接求逆。
     * 更新后自动执行协方差钳制。
     */
    void Update()
    {
        observe_func_(x_minus_, z_pred_, H_);
        HT_.noalias() = H_.transpose();

        S_.noalias() = H_ * P_minus_ * HT_;
        S_ += R_;
        MakeSymmetric(S_);

        auto solver = S_.ldlt();
        if (solver.info() != Eigen::Success) return;
        if (solver.vectorD().minCoeff() <= 0.0f) return;
        K_.noalias() = P_minus_ * HT_;
        K_ = solver.solve(K_.transpose()).transpose();
        if (k_scale_ != 1.0f) K_ *= k_scale_;

        const Obs innov = z_ - z_pred_;
        x_ = x_minus_ + K_ * innov;

        // 卡方值 χ² = innovᵀ · S⁻¹ · innov
        chi2_ = innov.dot(solver.solve(innov));

        // Joseph 形式协方差更新：P = (I-KH)·P⁻·(I-KH)ᵀ + K·R·Kᵀ
        // 保证 P 半正定，避免大 K 时标准公式的数值对消问题
        {
            const auto I_KH = Cov::Identity() - K_ * H_;
            P_.noalias() = I_KH * P_minus_ * I_KH.transpose() + K_ * R_ * K_.transpose();
        }

        ClampCovariance();
    }

    /**
     * @brief 更新步（同时设置观测值）
     * @param z  观测向量
     */
    void Update(const Obs& z)
    {
        z_ = z;
        Update();
    }

    /**
     * @brief 设置系统模型回调
     * @param f  f(x, u) → (x⁻, F)
     */
    inline void SetSystemFunc(SystemFunc f) { system_func_ = f; }

    /**
     * @brief 设置观测模型回调
     * @param h  h(x⁻) → (z_pred, H)
     */
    inline void SetObserveFunc(ObserveFunc h) { observe_func_ = h; }

    /**
     * @brief 设置观测值 z
     * @param z  观测向量
     */
    inline void SetZ(const Obs& z) { z_ = z; }

    /**
     * @brief 设置控制输入 u
     * @param u  控制向量
     */
    inline void SetU(const Ctrl& u) { u_ = u; }

    /**
     * @brief 设置过程噪声 Q（每周期可更新，如 dt 缩放）
     * @param Q  过程噪声协方差  n×n
     */
    inline void SetQ(const Cov& Q) { Q_ = Q; }

    /**
     * @brief 设置卡尔曼增益缩放因子（自适应增益用）
     * @param s  缩放系数，<1 削弱校正幅值
     */
    inline void SetGainScale(float s) { k_scale_ = s; }

    /**
     * @brief 获取先验协方差的引用（用于渐消因子等外部修改）
     */
    inline Cov& PMinus() { return P_minus_; }

    /**
     * @brief 设置后验状态（硬设定，不经过滤波）
     * @param x0  状态初值
     */
    inline void SetState(const State& x0) { x_ = x0; x_minus_ = x0; }

    /**
     * @brief 回退到预测值（卡方门控发散时使用）
     *        将 x⁻ 和 P⁻ 复制到后验，跳过本帧校正
     */
    inline void FallbackToPrediction()
    {
        x_ = x_minus_;
        P_ = P_minus_;
    }

    /**
     * @brief 重置滤波器
     * - 状态归零
     * - 协方差恢复为单位阵
     */
    inline void Reset()
    {
        x_.setZero();
        x_minus_.setZero();
        P_ = Cov::Identity();
        P_minus_ = P_;
    }

    /**
     * @brief 获取后验状态 x
     */
    inline const State& GetX() const { return x_; }

    /**
     * @brief 获取先验状态 x⁻ 的引用（用于发散回退等外部修改）
     */
    inline const State& GetXMinus() const { return x_minus_; }

    /**
     * @brief 获取后验协方差 P
     */
    inline const Cov&   GetP() const { return P_; }

    /**
     * @brief 设置后验协方差 P
     */
    inline void SetCovariance(const Cov& P) { P_ = P; }

    /**
     * @brief 获取卡尔曼增益 K
     */
    inline const Gain&  GetK() const { return K_; }

    /**
     * @brief 获取上一帧的卡方检验值 χ² = innovᵀ · S⁻¹ · innov
     */
    inline float GetChi2() const { return chi2_; }

private:
    // 回调
    SystemFunc  system_func_   = nullptr;  // f(x, u) → (x⁻, F)
    ObserveFunc observe_func_  = nullptr;  // h(x⁻) → (z_pred, H)

    // 参数
    CtrlMat B_;                 // 控制矩阵       n×p
    Cov     Q_;                 // 过程噪声协方差  n×n
    ObsCov  R_;                 // 观测噪声协方差  m×m

    // 输入
    Obs     z_;                 // 当前观测值    m×1
    Ctrl    u_;                 // 当前控制输入  p×1

    // 状态与协方差
    State   x_;                 // 后验状态     n×1
    State   x_minus_;           // 先验状态     n×1
    Cov     P_;                 // 后验协方差   n×n
    Cov     P_minus_;           // 先验协方差   n×n
    Gain    K_;                 // 卡尔曼增益   n×m
    float   k_scale_ = 1.0f;    // 增益缩放因子（自适应增益）

    // EKF 附加：每周期更新的 Jacobian
    Cov     F_;                 // F = ∂f/∂x    n×n（Predict 更新）
    ObsMat  H_;                 // H = ∂h/∂x    m×n（Update 更新）
    Eigen::Matrix<float, nx, nz> HT_;   // H_ 转置

    // EKF 附加：预测观测
    Obs     z_pred_;            // z_pred = h(x⁻)

    // 中间矩阵
    ObsCov  S_;                 // 创新协方差   H·P⁻·Hᵀ + R
    float   chi2_ = 0.0f;       // 卡方值   χ² = innovᵀ · S⁻¹ · innov

    // 协方差钳制
    State P_min_diag_;          // 对角线最小值（零向量时不钳制）

    /**
     * @brief 强制矩阵对称
     * @param M  原地用上三角填充下三角
     */
    template<typename Mat>
    static inline void MakeSymmetric(Mat& M)
    {
        M.template triangularView<Eigen::Upper>() = M.transpose();
    }

    /**
     * @brief 钳制协方差对角线最小值
     *
     * 防止协方差对角线元素因数值误差变为负值导致发散。
     * 仅当 P_min_diag_ 非零时有效。
     */
    void ClampCovariance()
    {
        P_.diagonal() = P_.diagonal().cwiseMax(P_min_diag_);
    }
};

} // namespace alg::filter

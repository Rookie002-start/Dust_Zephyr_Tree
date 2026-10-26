/**
 * @file rls.hpp
 * @author qingyu
 * @brief 递归最小二乘(RLS)自适应滤波器 — 模板版本，编译期分配内存
 * @version 0.3
 * @date 2026-06-27
 *
 * @par 算法原理
 *      标准 SISO-RLS，符号约定:
 *      - x_k: n×1 待估参数（权重）
 *      - α_k: 1×n 输入行向量
 *      - β_k: 1×1 实际输出（标量）
 *      - P_k: n×n 协方差矩阵
 *      - λ:   遗忘因子
 *
 *      递归步骤:
 *      ① β̂_k = α_k · x_{k-1}                           预测输出
 *      ② e_k  = β_k - β̂_k                              先验误差
 *      ③ K_k  = P_{k-1}·α_kᵀ / (λ + α_k·P_{k-1}·α_kᵀ)  增益
 *      ④ x_k  = x_{k-1} + K_k · e_k                    参数更新
 *      ⑤ P_k  = (P_{k-1} - K_k·α_k·P_{k-1}) / λ        协方差更新
 *
 * @tparam n  输入/参数维度（编译期分配）
 *
 * @par 使用示例
 * @code
 *   // 2 输入，遗忘因子 0.99，初始协方差 10.0
 *   alg::rls::RLS<2> filter(0.99f, 10.0f);
 *
 *   float α[2] = {1.0f, 2.0f};
 *   float β    = 5.0f;
 *   filter.Update(α, β);
 *   float k1 = filter.GetWeights()[0];
 *   float k2 = filter.GetWeights()[1];
 * @endcode
 */

#pragma once

#pragma message "Compiling Algorithm/Identify/Rls"

#include <stdint.h>
#include <string.h>
#include <cmath>
#include <Eigen/Dense>

namespace alg::rls {

template <uint8_t n>
class RLS final
{
    /// 固定尺寸 Eigen 类型：n 是编译期常量，运算全在栈上，不产生堆分配
    using MatN = Eigen::Matrix<float, n, n>;
    using VecN = Eigen::Matrix<float, n, 1>;
    using RowN = Eigen::Matrix<float, 1, n>;

public:
    /**
     * @brief 构造 RLS 辨识器
     * @param lambda  遗忘因子 (0 < λ ≤ 1)，越小对过去遗忘越快
     * @param p_init  协方差矩阵 P 的初始对角值
     * @param p_max   协方差对角上界，0 = 默认 (1e4 × p_init)
     * @param p_min   协方差对角下界，0 = 默认 (1e-6 × p_init)
     *
     * @note P 初始值建议:
     *       - SNR 较高 (信号可信): p_init = 1~10
     *       - SNR 较低 (噪声大):   p_init = 100~1000
     *
     * @note p_max / p_min 是"激励不足"时的数值兜底：输入向量长期接近 0
     *       （电机静置）时，协方差按 P ← P/λ 每拍膨胀，之后第一个有激励的
     *       样本会拿到异常大的增益，把权重甩飞。限幅把这种"静置再激励"的
     *       冲击限制在有界范围内。
     */
    RLS(float lambda, float p_init, float p_max = 0.0f, float p_min = 0.0f) : lambda_(lambda)
    {
        pMax_ = (p_max > 0.0f) ? p_max : (p_init * kPMaxRatio);
        pMin_ = (p_min > 0.0f) ? p_min : (p_init * kPMinRatio);

        for (uint16_t i = 0; i < n * n; i += (n + 1)) {
            P_k[i] = p_init;
        }
    }

    /**
     * @brief 执行一次 RLS 迭代
     * @param alpha  输入行向量 α_k，长度 n
     * @param beta   期望输出标量 β_k
     *
     * @par 算法步骤
     *      ① β̂_k  = α_k · x_k              预测输出
     *      ② e_k  = β_k - β̂_k              先验误差
     *      ③ K_k  = P·α_kᵀ / (λ + α·P·αᵀ)  卡尔曼增益
     *      ④ x_k += K_k · e_k              参数更新
     *      ⑤ P_k  = (P - K·α·P) / λ        协方差更新
     */
    /**
     * @brief 执行一次 RLS 迭代
     * @param alpha           输入行向量 α_k，长度 n
     * @param beta            期望输出标量 β_k
     * @param lambda_override  可选：覆盖内部 lambda_，-1 表示使用默认值
     *
     * @note lambda_override 用于异步帧场景：每帧传入 exp(-Δt / T_forget)
     *       约束: 0 < lambda ≤ 1，否则协方差更新可能发散
     */
    void Update(const float (&alpha)[n], float beta, float lambda_override = -1.0f)
    {
        // 输入非法（NaN / Inf）时直接丢弃本轮，保持上一轮权重
        if (!IsFinite(beta)) return;
        for (uint8_t i = 0; i < n; i++) {
            if (!IsFinite(alpha[i])) return;
        }

        // 拷贝输入
        memcpy(alpha_k, alpha, n * sizeof(float));
        beta_k = beta;

        // 确定本次使用的 lambda
        float lambda = (lambda_override > 0.0f) ? lambda_override : lambda_;
        if (lambda > 1.0f) lambda = 1.0f;

        // Eigen::Map 包装内部 float 数组，零拷贝（固定尺寸 → 不碰堆）
        Eigen::Map<VecN>       x(x_k, n);       // n×1 待估参数
        Eigen::Map<const RowN> a(alpha_k, n);   // 1×n 输入行向量
        Eigen::Map<MatN>       P(P_k, n, n);    // n×n 协方差矩阵

        // 预测 beta_hat_k = alpha_k . x_k
        beta_hat_k = a.dot(x);

        // 误差 e_k = beta_k - beta_hat_k
        e_k = beta_k - beta_hat_k;

        // 增益: K = P . alpha_kT / (lambda + alpha_k . P . alpha_kT)
        VecN K = P * a.transpose();
        float denom = lambda + a.dot(K);
        if (denom < 1e-12f) denom = 1e-12f;
        K /= denom;

        // 参数更新 x_k += K . e_k —— 先算到临时区、确认有限后再提交，
        // 避免更新到一半发散时权重被写坏
        float xNew[n] {};
        for (uint8_t i = 0; i < n; i++) {
            xNew[i] = x_k[i] + K(i) * e_k;
            if (!IsFinite(xNew[i])) return;
        }
        memcpy(x_k, xNew, sizeof(x_k));

        // 协方差更新 P = (P - K . alpha . P) / lambda
        MatN Pnew = (P - K * a * P) / lambda;

        // 对称化：浮点误差会让 P 逐拍失去对称性，增益方向会跟着偏
        Pnew = ((Pnew + Pnew.transpose()) * 0.5f).eval();

        // 对角限幅：防协方差风洞（膨胀到无限），也防塌成 0 后彻底停止学习
        for (uint8_t i = 0; i < n; i++) {
            float d = Pnew(i, i);
            if (!IsFinite(d))   d = pMax_;
            else if (d > pMax_) d = pMax_;
            else if (d < pMin_) d = pMin_;
            Pnew(i, i) = d;
        }
        P = Pnew;
    }

    const float* GetWeights() const { return x_k; }
    void  SetWeights( const float (&w)[n] ) { memcpy(x_k, w, n * sizeof(float)); }
    float GetOutput() const { return beta_hat_k; }
    float GetError()  const { return e_k; }

    /**
     * @brief 重置全部状态（权重清零、协方差重新初始化），用于多轮辨识
     */
    void Reset(float p_init)
    {
        memset(x_k,     0, sizeof(x_k));
        memset(alpha_k, 0, sizeof(alpha_k));
        memset(P_k,     0, sizeof(P_k));

        beta_k     = 0.0f;
        beta_hat_k = 0.0f;
        e_k        = 0.0f;

        for (uint16_t i = 0; i < n * n; i += n + 1) {
            P_k[i] = p_init;
        }
    }

private:
    /// 有限性判据（NaN / Inf 一律视为无效样本）
    static bool IsFinite(float v) { return std::isfinite(v); }

    /// 协方差对角上下界的默认倍率（相对 p_init）
    static constexpr float kPMaxRatio = 1.0e4f;
    static constexpr float kPMinRatio = 1.0e-6f;

    const float lambda_ = 0.99999f;

    float x_k[n]     {};
    float alpha_k[n] {};
    float beta_k    = 0;
    float P_k[n * n] {};

    float beta_hat_k = 0;
    float e_k        = 0;

    float pMax_ = 0.0f;
    float pMin_ = 0.0f;
};

}; // namespace alg::rls

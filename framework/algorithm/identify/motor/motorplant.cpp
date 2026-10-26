/**
 * @file motorplant.cpp
 * @author qingyu
 * @brief 电机本体在线辨识实现
 * @version 0.1
 * @date 2026-07-22
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "motorplant.hpp"

namespace alg::identify::motor {

// 工具函数

/**
 * @brief 回绕安全的时间比较
 * @return true  a 早于 b
 */
bool MotorPlant::TimeBefore(uint32_t a, uint32_t b)
{
    return (int32_t)(a - b) < 0;
}

/**
 * @brief 回绕安全的序号判断
 * @return true  seq 是 last 的下一个
 */
bool MotorPlant::SeqIsNext(uint16_t seq, uint16_t last)
{
    return (uint16_t)(seq - last) == 1;
}

/**
 * @brief 零阶保持：找 target 之前最近一次发送的转矩
 * @param target  目标时刻（μs）
 * @param out     输出转矩
 * @return true   找到
 */
bool MotorPlant::FindLastBefore(uint32_t target, float* out) const
{
    uint16_t count = (valid_count_ < kBufSize) ? valid_count_ : kBufSize;
    for (uint16_t i = 0; i < count; i++)
    {
        // 从最新往最老查
        uint8_t idx = (write_idx_ - 1 - i + kBufSize) % kBufSize;
        if (!TimeBefore(target, buf_[idx].t_req_us))
        {
            *out = buf_[idx].torque_nm;
            return true;
        }
    }
    return false;
}

// 构造与重置

MotorPlant::MotorPlant(const Config& cfg)
    : cfg_(cfg)
    , rls_(1.0f, cfg.p_init)
{
    if (cfg_.buf_size > kBufSize)
        cfg_.buf_size = kBufSize;
}

/**
 * @brief 重置全部状态：清空缓冲、参数归零、RLS 重置
 */
void MotorPlant::Reset()
{
    rls_.Reset(cfg_.p_init);
    valid_count_ = 0;
    write_idx_   = 0;

    has_seq_       = false;
    last_seq_      = 0;
    omega_prev_    = 0.0f;
    prev_time_us_  = 0;

    p1_ = 0.0f;
    p2_ = 0.0f;
    p3_ = 0.0f;
    p4_ = 0.0f;
    physical_valid_ = false;
    model_ready_    = false;
    sample_count_   = 0;

    identify_mode_ = false;

    pred_err_          = 0.0f;
    delay_unavailable_ = 0;
}

// 核心接口

/**
 * @brief 记录一次转矩发送
 * @param t_req_us   发送请求时刻（μs）
 * @param torque_nm  量化反解后的实际转矩（Nm）
 *
 * @note 仅在确认发送成功后才调用
 */
void MotorPlant::OnTorqueSend(uint32_t t_req_us, float torque_nm)
{
    buf_[write_idx_] = {t_req_us, torque_nm};
    if (valid_count_ < kBufSize)
        valid_count_++;
    write_idx_ = (write_idx_ + 1) % kBufSize;
}

/**
 * @brief 处理一次 CAN 反馈，RLS 条件更新
 * @param time_us  反馈到达时刻（μs）
 * @param omega    角速度（rad/s）
 * @param seq      反馈序号（去重 + 连续检查）
 *
 * @par 更新条件
 *      首包初始化 / 去重跳过 / 丢帧跳过 / Δt 超限跳过 / 缓冲不足跳过
 *      辨识模式开启时才更新 RLS 参数
 */
void MotorPlant::OnFeedback(uint32_t time_us, float omega, uint16_t seq)
{
    // 首包初始化
    if (!has_seq_)
    {
        last_seq_     = seq;
        omega_prev_   = omega;
        prev_time_us_ = time_us;
        has_seq_      = true;
        return;
    }

    // 去重
    if (seq == last_seq_)
        return;

    // 连续检查（此时 last_seq_ 还是上帧值）
    if (!SeqIsNext(seq, last_seq_))
    {
        last_seq_     = seq;
        omega_prev_   = omega;
        prev_time_us_ = time_us;
        return;
    }
    last_seq_ = seq;    // 连续检查通过后才更新

    // Δt 有效性
    float dt = (time_us - prev_time_us_) * 1e-6f;
    if (dt < 1e-6f || dt > cfg_.dt_max)
    {
        omega_prev_   = omega;
        prev_time_us_ = time_us;
        return;
    }

    // 获取延时转矩（零阶保持）
    uint32_t target = time_us - (uint32_t)(cfg_.delay_s * 1e6f);
    float torque_delayed;
    if (!FindLastBefore(target, &torque_delayed))
    {
        delay_unavailable_++;
        omega_prev_   = omega;
        prev_time_us_ = time_us;
        return;
    }

    // 摩擦方向符号
    float sign_omega = (omega_prev_ > 0.0f) ? 1.0f
                     : (omega_prev_ < 0.0f) ? -1.0f : 0.0f;

    // 预测误差（所有模式下都算）
    float d_omega_pred = dt * (p1_ * omega_prev_
                             + p2_ * torque_delayed
                             + p3_
                             + p4_ * sign_omega);
    pred_err_ = (omega - omega_prev_) - d_omega_pred;

    // 参数更新（仅辨识模式）
    if (identify_mode_)
    {
        float lambda = expf(-dt / cfg_.forget_tau);
        if (lambda <= 0.0f) lambda = 1e-6f;
        if (lambda > 1.0f)  lambda = 1.0f;

        const float alpha[4] = {
            dt * omega_prev_,
            dt * torque_delayed,
            dt,
            dt * sign_omega,
        };
        rls_.Update(alpha, omega - omega_prev_, lambda);

        p1_ = rls_.GetWeights()[0];
        p2_ = rls_.GetWeights()[1];
        p3_ = rls_.GetWeights()[2];
        p4_ = rls_.GetWeights()[3];

        physical_valid_ = (p1_ < -1e-6f && p2_ > 1e-6f);
        sample_count_++;

        if (physical_valid_ && sample_count_ >= 100)
            model_ready_ = true;
    }

    // 状态平移
    omega_prev_   = omega;
    prev_time_us_ = time_us;
}

// 参数读取

/**
 * @brief 开环时间常数 τ = -1/p1
 * @return 仅 model_ready_ 时有效，否则返回 0
 */
float MotorPlant::GetTau() const
{
    return model_ready_ ? -1.0f / p1_ : 0.0f;
}

/**
 * @brief 开环增益 K = -p2/p1
 * @return 仅 model_ready_ 时有效，否则返回 0
 */
float MotorPlant::GetK() const
{
    return model_ready_ ? -p2_ / p1_ : 0.0f;
}

} // namespace alg::identify::motor

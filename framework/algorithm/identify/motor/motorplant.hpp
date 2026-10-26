/**
 * @file motorplant.hpp
 * @author qingyu
 * @brief 电机本体在线辨识 — 连续时间增量 RLS（含摩擦项）
 *        Δω = Δt·(p1·ω + p2·τ(t-L) + p3 + p4·sign(ω))
 *        零阶保持延时对齐，异步 CAN 反馈
 * @version 0.1
 * @date 2026-07-22
 *
 * @copyright Copyright (c) 2026
 *
 * @par 使用流程
 *      MotorPlant plant(cfg);
 *      plant.OnTorqueSend(t_req_us, torque_nm);   // 发送侧
 *      plant.OnFeedback(t_rx_us, omega, seq);     // 接收侧
 *
 * @par 并发说明
 *      OnTorqueSend 和 OnFeedback 在不同上下文调用时，
 *      调用方需保证 OnTorqueSend 写入不被 RX 中断打断。
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include "rls.hpp"

namespace alg::identify::motor {

class MotorPlant final
{
public:
    struct Config
    {
        float    forget_tau  = 1.0f;       // 遗忘时间常数 (s)，> 0
        float    p_init      = 100.0f;     // 协方差初值
        float    delay_s     = 0.001f;     // 固定延时 L (s)
        float    dt_max      = 0.01f;      // 允许最大 Δt (s)
        uint16_t buf_size    = 64;         // 逻辑缓冲容量 (≤ kBufSize)
    };

    static constexpr uint16_t kBufSize = 64;

    explicit MotorPlant(const Config& cfg);

    void OnTorqueSend(uint32_t t_req_us, float torque_nm);
    void OnFeedback(uint32_t t_rx_us, float omega, uint16_t seq);
    void SetIdentifyMode(bool enable) { identify_mode_ = enable; }

    // 读参数
    float GetP1()     const { return p1_; }
    float GetP2()     const { return p2_; }
    float GetP3()     const { return p3_; }
    float GetP4()     const { return p4_; }
    float GetTau()    const;                // -1/p1, 仅 model_ready_ 有效
    float GetK()      const;                // -p2/p1, 仅 model_ready_ 有效
    float GetPredErr() const { return pred_err_; }
    uint32_t GetUpdateCount()    const { return sample_count_; }
    uint32_t GetDelayUnavail()   const { return delay_unavailable_; }
    bool  IsPhysicalValid() const { return physical_valid_; }
    bool  IsModelReady()    const { return model_ready_; }

    void Reset();

private:
    static bool  TimeBefore(uint32_t a, uint32_t b);
    bool  FindLastBefore(uint32_t target, float* out) const;
    static bool  SeqIsNext(uint16_t seq, uint16_t last);

    Config       cfg_;
    rls::RLS<4>  rls_;

    // ── 转矩环形缓冲 ──────────────────────────────────
    struct TorqueSample {
        uint32_t t_req_us;                      // 发送请求时刻 (μs)
        float    torque_nm;                     // 量化反解后的转矩 (Nm)
    };
    TorqueSample buf_[kBufSize];                // 环形缓冲
    uint16_t     valid_count_   = 0;            // 已写入样本数（≤ kBufSize）
    uint8_t      write_idx_     = 0;            // 写指针

    // ── 反馈状态 ──────────────────────────────────────
    bool     has_seq_           = false;        // 是否收到过首包
    uint16_t last_seq_          = 0;            // 上一帧序号
    float    omega_prev_        = 0.0f;         // ω[k-1]
    uint32_t prev_time_us_      = 0;            // 上一帧时间戳 (μs)

    // ── 参数与状态 ────────────────────────────────────
    float    p1_                = 0.0f;         // 阻尼         p1 < 0,  τ = -1/p1
    float    p2_                = 0.0f;         // 转矩增益     p2 > 0, K = -p2/p1
    float    p3_                = 0.0f;         // 偏置/零漂
    float    p4_                = 0.0f;         // 库仑摩擦     p4 < 0, 单位 ω/s
    bool     physical_valid_    = false;        // 参数符号方向合理
    bool     model_ready_       = false;        // 已收敛可用
    uint32_t sample_count_      = 0;            // RLS 更新次数

    // ── 模式 ──────────────────────────────────────────
    bool identify_mode_         = false;        // true=更新 RLS, false=只算误差

    // ── 统计 ──────────────────────────────────────────
    float    pred_err_          = 0.0f;         // 一步预测误差
    uint32_t delay_unavailable_ = 0;            // 缓冲不足跳过次数
};

} // namespace alg::identify::motor

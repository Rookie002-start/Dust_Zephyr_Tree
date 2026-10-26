/**
 * @file stability.hpp
 * @author qingyu
 * @brief 辨识用稳定判据与扰动工具 — 内联实现，仅辨识阶段使用
 * @version 0.1
 * @date 2026-07-19
 */

#pragma once

#pragma message "Compiling Algorithm/Identify/Stability"

#include <cmath>
#include <cstdint>

namespace stability {

/**
 * @brief 滑动窗口稳定判据
 *
 * 积累 kWinSize 帧数据后，判断极差和斜率是否在限内。
 *
 * @tparam kWinSize  滑动窗口大小（帧数）
 */
template <uint32_t kWinSize>
class WinStable final
{
public:
    constexpr WinStable() = default;

    /**
     * @brief 推入一帧温度并判断是否稳定
     *
     * @param temp_c  当前温度 (°C)
     * @param dt_s    帧间隔 (s)，用于斜率计算
     * @param slope_limit  允许的最大斜率 (°C/s)
     * @param noise_limit  允许的最大极差 (°C)
     * @param drift_limit  允许的窗口净漂移 (首帧-末帧, °C)
     * @return true  窗口满且稳定
     */
    bool Check(float temp_c, float dt_s, float slope_limit = 0.01f, float noise_limit = 0.10f,
               float drift_limit = 0.50f)
    {
        if (cnt_ < kWinSize) {
            Push(temp_c);
            return false;
        }

        float mn = buf_[0], mx = buf_[0];
        for (uint32_t i = 1; i < kWinSize; i++) {
            if (buf_[i] < mn) mn = buf_[i];
            if (buf_[i] > mx) mx = buf_[i];
        }

        // 窗口满时 buf_[head_] 是最旧帧，当前帧 - 最旧帧 = 窗口净漂移
        const float net_drift = temp_c - buf_[head_];
        const float slope = dt_s > 0.0f ? (temp_c - prev_) / dt_s : 0.0f;
        Push(temp_c);

        return std::abs(slope) <= slope_limit
            && (mx - mn) <= noise_limit
            && std::abs(net_drift) <= drift_limit;
    }

    void Reset()
    {
        head_ = 0;
        cnt_  = 0;
        prev_ = 0.0f;
    }

private:
    float buf_[kWinSize] {};
    float prev_     = 0.0f;
    uint32_t head_  = 0;
    uint32_t cnt_   = 0;

    void Push(float val)
    {
        buf_[head_] = val;
        head_ = (head_ + 1) % kWinSize;
        if (cnt_ < kWinSize) cnt_++;
        prev_ = val;
    }
};

/**
 * @brief 均值窗口稳定判据 — 误差均值 + 标准差，多轮确认
 *
 * 每帧累积误差绝对值（及其平方，用于标准差）和斜率绝对值。
 * 窗口满后判：误差均值 + 误差标准差 + 最大误差 + 平均斜率 + 最大斜率。
 * 连续 kRounds 个窗口通过 → 判稳定。
 *
 * @tparam kWinSize  一个窗口的帧数
 * @tparam kRounds   稳定所需连续窗口数
 */
template <uint32_t kWinSize, uint32_t kRounds>
class MeanStable final
{
public:
    constexpr MeanStable() = default;

    /**
     * @brief 推入一帧并判断是否稳定
     *
     * 斜率分母为窗口内实际有效斜率数（kWinSize - 1）。
     *
     * @param target         目标值
     * @param measured       当前测量值
     * @param dt_s           帧间隔 (s)
     * @param error_limit    允许的最大平均绝对误差
     * @param var_limit      允许的误差方差上限
     * @param max_err_limit  允许的单帧最大绝对误差
     * @param slope_limit    允许的最大平均绝对斜率
     * @param max_slp_limit  允许的单帧最大绝对斜率
     * @return true  连续 kRounds 个窗口均合格
     */
    bool Check(float target, float measured, float dt_s,
               float error_limit    = 0.50f,
               float var_limit      = 0.25f,
               float max_err_limit  = 1.00f,
               float slope_limit    = 0.01f,
               float max_slp_limit  = 0.05f)
    {
        const float err = std::abs(measured - target);
        sum_err_    += err;
        sum_err_sq_ += err * err;
        if (err > max_err_) max_err_ = err;

        if (cnt_ > 0 && dt_s > 0.0f) {
            const float slp = std::abs((measured - prev_) / dt_s);
            sum_slp_   += slp;
            slope_cnt_++;
            if (slp > max_slp_) max_slp_ = slp;
        }
        prev_ = measured;

        if (++cnt_ < kWinSize)
            return false;

        // 窗口满，逐项判
        const float mean_err = sum_err_ / kWinSize;
        const float var_err  = (sum_err_sq_ / kWinSize) - (mean_err * mean_err);
        const bool  pass     = mean_err <= error_limit
                            && var_err <= var_limit
                            && max_err_ <= max_err_limit
                            && (sum_slp_ / slope_cnt_) <= slope_limit
                            && max_slp_ <= max_slp_limit;

        // 复位
        sum_err_ = sum_err_sq_ = 0.0f;
        max_err_ = 0.0f;
        sum_slp_ = 0.0f;
        max_slp_ = 0.0f;
        cnt_ = slope_cnt_ = 0;

        if (!pass) {
            good_rnd_ = 0;
            return false;
        }

        return (++good_rnd_ >= kRounds);
    }

    void Reset()
    {
        sum_err_ = sum_err_sq_ = 0.0f;
        max_err_ = 0.0f;
        sum_slp_ = 0.0f;
        max_slp_ = 0.0f;
        prev_ = 0.0f;
        cnt_ = slope_cnt_ = good_rnd_ = 0;
    }

private:
    float sum_err_    = 0.0f;       // 窗口内累积 |error|
    float sum_err_sq_ = 0.0f;       // 窗口内累积 error²（方差用）
    float max_err_    = 0.0f;       // 窗口内最大 |error|
    float sum_slp_    = 0.0f;       // 窗口内累积 |slope|
    float max_slp_    = 0.0f;       // 窗口内最大 |slope|
    float prev_       = 0.0f;       // 上一帧测量值（斜率计算用）
    uint32_t cnt_        = 0;       // 当前窗口帧计数
    uint32_t slope_cnt_  = 0;       // 实际有效斜率数（N 帧 = N-1 个）
    uint32_t good_rnd_   = 0;       // 连续合格窗口数
};

/**
 * @brief 方波发生器：±amp
 */
class SquareGen final
{
public:
    float WaveGen(float amp, uint32_t period)
    {
        return (++cnt_ / period) % 2 ? amp : -amp;
    }
    void Reset() { cnt_ = 0; }
private:
    uint32_t cnt_ = 0;
};

/**
 * @brief 三角波发生器：-amp ~ +amp
 */
class TriangleGen final
{
public:
    float WaveGen(float amp, uint32_t period)
    {
        uint32_t t = cnt_++ % period;
        float v = 2.0f * static_cast<float>(t) / static_cast<float>(period);
        return (v < 1.0f) ? (v * amp) : ((2.0f - v) * amp);
    }
    void Reset() { cnt_ = 0; }
private:
    uint32_t cnt_ = 0;
};

/**
 * @brief 正弦波发生器：amp·sin(2π·freq·t)
 */
class SineGen final
{
public:
    float WaveGen(float amp, float freq_hz, float dt_s)
    {
        phase_ += freq_hz * dt_s;
        if (phase_ >= 1.0f) phase_ -= 1.0f;
        return amp * std::sin(2.0f * 3.14159265f * phase_);
    }
    void Reset() { phase_ = 0.0f; }
private:
    float phase_ = 0.0f;
};

} // namespace stability // namespace stability

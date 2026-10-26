/**
 * @file lpf.hpp
 * @author qingyu
 * @brief 一阶低通滤波器（基于截止频率）
 * @version 0.1
 * @date 2026-04-30
 *
 * @par 使用示例
 * @code
 *   alg::filter::LowPassFilter lpf(100.0f, 0.001f);  // 100 Hz, 1 ms
 *   float out = lpf.Update(raw);
 * @endcode
 */

#pragma once

#pragma message "Compiling Algorithm/Filter/Lpf"

namespace alg::filter {

/**
 * @brief 一阶 RC 低通滤波器
 *
 *        out[n] = alpha * in[n] + (1-alpha) * out[n-1]
 *
 *        alpha = dt / (dt + RC),  RC = 1 / (2π · fc)
 */
class LowPassFilter final
{
public:
    LowPassFilter() = default;

    /**
     * @brief 构造并初始化
     * @param cutoffHz  截止频率（Hz），≤ 0 表示直通
     * @param dt        采样周期（秒）
     */
    LowPassFilter(float cutoffHz, float dt) { Init(cutoffHz, dt); }

    /**
     * @brief（重新）配置滤波器
     * @param cutoffHz  截止频率（Hz），≤ 0 表示直通
     * @param dt        采样周期（秒）
     */
    void Init(float cutoffHz, float dt)
    {
        if (cutoffHz <= 0.0f) {
            alpha_ = 1.0f;      // 直通模式
        } else {
            float rc = 1.0f / (k2Pi * cutoffHz);
            alpha_ = dt / (dt + rc);
        }
        output_ = 0.0f;
        initialized_ = true;
    }

    /**
     * @brief 用新采样值更新滤波器
     * @param input  原始输入值
     * @return       滤波后的输出值
     */
    float Update(float input)
    {
        if (!initialized_) return input;
        output_ = alpha_ * input + (1.0f - alpha_) * output_;
        return output_;
    }

    float GetOutput() const { return output_; }
    void  Reset(float val = 0.0f) { output_ = val; }

private:
    static constexpr float k2Pi = 6.283185307179586f;

    float alpha_       = 0.0f;
    float output_      = 0.0f;
    bool  initialized_ = false;
};

} // namespace alg::filter

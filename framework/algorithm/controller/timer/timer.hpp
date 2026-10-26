/**
 * @file Timer.hpp
 * @author qingyu
 * @brief 软件定时器，依赖外部周期性调用驱动计数（Update 必须在 Tick/Clock 前调用）
 * @version 0.1
 * @date 2026-03-12
 * 
 * @copyright Copyright (c) 2026
 * 
 * @example
 * 
 *   // ========== 1. Update + Clock ==========
 *   Timer timer(1000);
 *   while (1) {
 *       timer.Update();
 *       timer.Clock([]{ led.Toggle(); });
 *       k_msleep(1);
 *   }
 * 
 *   // ========== 2. Update + Tick ==========
 *   Timer timer(1000);
 *   while (1) {
 *       timer.Update();
 *       timer.Tick([&]{
 *           if (timer.GetCounter() == 500) { led.Toggle(); }
 *       });
 *       k_msleep(1);
 *   }
 * 
 *   // ========== 3. Update + Tick + Clock 组合 ==========
 *   Timer timer(1000);
 *   while (1) {
 *       timer.Update();
 *       timer.Tick([&]{
 *           if (timer.GetCounter() == 500) { led2.Toggle(); }
 *       });
 *       timer.Clock([]{ led1.Toggle(); });
 *       k_msleep(1);
 *   }
 * 
 *   // ========== 4. Update + GetCounter 裸用 ==========
 *   Timer timer(1000);
 *   while (1) {
 *       timer.Update();
 *       if (timer.GetCounter() == 500) { led1.Toggle(); }
 *       if (timer.IsFinish())           { led2.Toggle(); }
 *       k_msleep(1);
 *   }
 * 
 *   // ========== 5. 运行时修改周期 ==========
 *   timer.SetPeriod(2000);  // 当前周期结束后生效
 * 
 *   // ========== 6. Pause / Resume ==========
 *   timer.Pause();    // 冻结计数，保留当前值
 *   timer.Resume();   // 从断点继续
 * 
 *   // ========== 7. Stop ==========
 *   timer.Stop();     // 停止 + 清零
 *   timer.Resume();   // 从头开始
 * 
 */
#pragma once

#pragma message "Compiling Algorithm/Controller/Timer"

#include <stdint.h>

class Timer final
{
public:
    Timer(uint32_t period) : period_(period) {}
    ~Timer() = default;

    void Update()
    {
        if (!is_running_) return;
        UpdatePeriod();
        count_++;
        if (count_ >= period_) {
            Finish();
        }
    }

    template<typename T>
    void Tick(T && func) 
    {
        if (!is_period_finish_) {
            func();
        }
    }

    template<typename T>
    void Clock(T && func)
    {
        if (is_period_finish_) {
            func();
        }
    }

    void SetPeriod(uint32_t period) 
    {
        if (is_period_finish_) {
            period_ = period;
            is_period_changed_ = false;
        } else {
            period_buffer_ = period;
            is_period_changed_ = true;
        }
    }

    uint32_t GetPeriod() const { return period_; }
    uint32_t GetCounter() const { return count_; }
    
    bool IsFinish() const { return is_period_finish_; }
    bool IsRunning() const { return is_running_; }

    void Reset() { count_ = 0; }

    void Finish() 
    {
        is_period_finish_ = true;
        count_ = 0;
    }

    void Pause() { is_running_ = false; }
    void Resume() { is_running_ = true; }

    void Stop()
    {
        is_running_ = false;
        count_ = 0;
        is_period_finish_ = true;
    }

private:
    bool is_period_changed_ = false;
    bool is_period_finish_ = false;
    bool is_running_ = true;
    
    uint32_t period_buffer_ = 0;
    uint32_t period_ = 0;
    uint32_t count_ = 0;

    void UpdatePeriod()
    {
        if (is_period_finish_) {
            if (is_period_changed_) {
                period_ = period_buffer_;
                is_period_changed_ = false;
            }
            is_period_finish_ = false;
        }
    }
};
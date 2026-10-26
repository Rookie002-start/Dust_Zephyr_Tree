/**
 * @file exectimer.hpp
 * @author qingyu
 * @brief 一个简单的计算代码开销计时器
 * @version 0.1
 * @date 2026-06-29
 *
 * @copyright Copyright (c) 2026
 *
 * @example
 *
 *   ExecTimer t(1000);
 *   t.SetStart();
 *   func();
 *   t.SetEnd();
 *   t.PrintTotal();
 *
 */

#pragma once

#pragma message "Compiling Algorithm/Controller/ExecTimer"

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <stdint.h>

class ExecTimer final
{
public:
    /**
     * @param print_interval 每 print_interval 次 PrintTotal 才输出一次；0 = 每次都输出
     */
    ExecTimer(uint16_t print_interval = 0) : print_interval_(print_interval) {}

    void SetStart() { start_ = k_cycle_get_32(); }
    void SetEnd()   { end_   = k_cycle_get_32(); }

    uint32_t GetTotalUs() const { return k_cyc_to_us_floor32(end_ - start_); }
    float    GetTotalMs() const { return static_cast<float>(GetTotalUs()) * 0.001f; }

    void PrintTotal()
    {
        if (print_interval_ == 0 || ++counter_ >= print_interval_) {
            counter_ = 0;
            printk("%.3f ms\n", (double)GetTotalMs());
        }
    }

private:
    uint32_t start_ = 0;
    uint32_t end_   = 0;
    uint16_t print_interval_;
    uint16_t counter_ = 0;
};

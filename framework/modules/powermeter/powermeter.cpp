/**
 * @file horizon.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "powermeter.hpp"

#pragma message "Compiling Modules/Powermeter"

void PowerMeter::CanCpltRxCallback(uint8_t* buffer)
{
    const uint8_t* data         = buffer;
    const int16_t shunt_raw     = static_cast<int16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
    // 注意必须用 int16_t：这四个量都是**带符号**的。
    // 之前用 uint16_t 接，刹车/回馈时电流 −0.79 A 会被读成 +64.75 A（65536−786），
    // 看起来像"制动狂吸 65A"，直接误导了功率模型的判断。
    const int16_t bus_raw       = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | data[3]);
    const int16_t current_raw   = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | data[5]);
    const int16_t power_raw     = static_cast<int16_t>((static_cast<uint16_t>(data[6]) << 8) | data[7]);

    /* seqlock 写锁：允许其他中断，线程读到冲突时会自旋重试 */
    atomic_inc(&seq_);
    shunt_volt = static_cast<float>(shunt_raw)   * 0.001f;  // V
    bus_volt   = static_cast<float>(bus_raw)     * 0.001f;  // V
    current    = static_cast<float>(current_raw) * 0.001f;  // A
    power      = static_cast<float>(power_raw)   * 0.01f;   // W
    frames_   += 1;
    stamp_ms_  = k_uptime_get_32();
    atomic_inc(&seq_);
}

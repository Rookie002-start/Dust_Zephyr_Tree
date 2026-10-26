/**
 * @file imu_to.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-06-02
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <stdint.h>
#include <zephyr/zbus/zbus.h>

namespace topic::imu_to {

struct Message
{
    /// 每次发布递增（IMU 模块在 zbus_chan_pub 前自增）。
    /// 消费者据此判断"数据是否真的更新过 / 源是否在发布"：从未发布会保持 0。
    uint32_t version = 0;

    float quaternion[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    float gyro[3]       = {0.0f, 0.0f, 0.0f};

    float temperature = 0.0f;
    float roll        = 0.0f;
    float pitch       = 0.0f;
    float yaw         = 0.0f;
    float yaw_total   = 0.0f;
};

} // namespace topic::imu_to

ZBUS_CHAN_DECLARE(pub_imu_to);

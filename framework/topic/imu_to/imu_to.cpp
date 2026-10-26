/**
 * @file imu_to.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "imu_to.hpp"

#pragma message "Compiling Topic/Imu_to"

ZBUS_CHAN_DEFINE(pub_imu_to,
                 topic::imu_to::Message,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS_EMPTY,        // 无订阅者：消费者用 zbus_chan_read 轮询最新值
                 ZBUS_MSG_INIT({}));

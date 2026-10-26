/**
 * @file remote_to.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-06-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <zephyr/zbus/zbus.h>
#include "remote_to.hpp"

#pragma message "Compiling Topic/Remote_to"

ZBUS_CHAN_DEFINE(pub_remote_to,
                 topic::remote_to::Message,
                 NULL,
                 NULL,
                 ZBUS_OBSERVERS_EMPTY,        // 无订阅者：消费者用 zbus_chan_read 轮询最新值
                 ZBUS_MSG_INIT({}));

/**
 * @file Irq_handlers.h
 * @author qingyu
 * @brief CAN 接收分发——注册表
 * @version 0.1
 * @date 2026-05-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <zephyr/drivers/can.h>

// 必须用宏——预处理阶段展开为数字，用于拼接 section 名 ".can_rx1/2/3"
// constexpr / enum 在编译期才求值，__STRING / ## 拼接时无法展开
#define USER_RX_CAN1            1
#define USER_RX_CAN2            2
#define USER_RX_CAN3            3

/**
 * @brief CAN 帧接收回调类型
 *
 */
using CanRxHandler = void (*)(uint8_t *data);

/**
 * @brief CAN 接收条目
 *
 */
struct CanRxEntry {
    uint16_t     id;
    CanRxHandler handler;
};

/**
 * @brief 注册 CAN ID 对应的接收处理器（编译期，链接段收集）
 *
 * @param bus_  CAN 总线（CanBus 枚举）
 * @param id_   CAN ID
 * @param handler_ 回调函数
 *
 * @code
 *   CAN_RX_HANDLER(CanBus::UserCan1, 0x201, [](uint8_t *data) {
 *       my_motor.CanCpltRxCallback(data);
 *   });
 * @endcode
 */
// 两级字符串化：先展开宏参数（TEST_RX→1），再转字符串（1→"1"）
#define _CAN_RX_STR(x)      #x
#define _CAN_RX_STR2(x)     _CAN_RX_STR(x)

#define CAN_RX_HANDLER(bus_, id_, handler_, name_)                                  \
    static const CanRxEntry kCanRxEntry_##bus_##_##name_                            \
    __attribute__((used, __section__(".can_rx"                                      \
        _CAN_RX_STR2(bus_)))) = { id_, handler_ }

        
void user_can1_rx_callback(struct can_frame &frame, void *);
void user_can2_rx_callback(struct can_frame &frame, void *);
void user_can3_rx_callback(struct can_frame &frame, void *);
















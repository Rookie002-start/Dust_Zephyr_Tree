/**
 * @file Irq_handlers.cpp
 * @author qingyu
 * @brief CAN 接收分发——注册表
 * @version 0.1
 * @date 2026-05-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Irq_handlers.h"
#include <zephyr/sys/printk.h>

extern const CanRxEntry __can_rx1_start[], __can_rx1_end[];
extern const CanRxEntry __can_rx2_start[], __can_rx2_end[];
extern const CanRxEntry __can_rx3_start[], __can_rx3_end[];

static void dispatch(struct can_frame &frame, const CanRxEntry *start, const CanRxEntry *end)
{
    for (const CanRxEntry *e = start; e < end; ++e)
    {
        if (e->id == frame.id)
        {
            e->handler(frame.data);
            return;
        }
    }
}

void user_can1_rx_callback(struct can_frame &frame, void *)
{
    dispatch(frame, __can_rx1_start, __can_rx1_end);
}

void user_can2_rx_callback(struct can_frame &frame, void *)
{
    dispatch(frame, __can_rx2_start, __can_rx2_end);
}

void user_can3_rx_callback(struct can_frame &frame, void *)
{
    dispatch(frame, __can_rx3_start, __can_rx3_end);
}

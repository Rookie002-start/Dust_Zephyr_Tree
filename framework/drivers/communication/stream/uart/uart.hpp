/**
 * @file uart.hpp
 * @author qingyu
 * @brief UART DMA 驱动 - 双缓冲 DMA + Stream 接口
 * @version 0.5
 * @date 2026-07-23
 *
 * # UART DMA 使用说明
 *
 * 双缓冲 DMA 接收，适合高速连续接收。
 * ### 设备树
 * 项目 overlay 中定义 alias：
 * ```dts
 * aliases {
 *     uart-remote = &uart4;
 * };
 * ```
 *
 * ### Kconfig
 * ```kconfig
 * config TRD_REMOTE
 *     select DUST_COM_UART_DMA
 * ```
 *
 * ### 初始化
 * ```cpp
 * static UartDma uart;
 *
 * uart.Init(DEVICE_DT_GET(DT_ALIAS(uart_remote)));
 * ```
 *
 * ### 接收回调
 * ```cpp
 * uart.SetRxCallback([](uint8_t* data, uint16_t len) {
 *     // 处理接收数据
 * });
 * ```
 *
 * ### 发送
 * ```cpp
 * uart.Send(data, len);  // DMA 异步发送
 * ```
 *
 * @copyright Copyright (c) 2026
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <stdint.h>
#include "bipbuf.hpp"
#include "../stream.hpp"

/**
 * @brief UART DMA 模式驱动
 *
 * 双缓冲 DMA 接收 + 中断回调，支持高速连续接收。 
 */
class UartDma final : public Stream
{
    friend void uart_dma_callback(const struct device* dev, struct uart_event* evt, void* user_data);

public:
    struct Config 
    {
        uart_config line_cfg {
            921600,
            UART_CFG_PARITY_NONE,
            UART_CFG_STOP_BITS_1,
            UART_CFG_DATA_BITS_8,
            UART_CFG_FLOW_CTRL_NONE,
        };
        BaseConfig  base_cfg {};
    };

    bool     Init(const struct device* dev, const Config& cfg);
    void     SetBaudrate(uint32_t baud);
    void     SetLineConfig(const uart_config& cfg);

    bool     StartRx();
    void     StopRx();

    uint16_t Read(uint8_t* buf, uint16_t max_len)    override;
    bool     Send(const uint8_t* data, uint32_t len) override;

private:
    /// DMA 双缓冲单块 / 发送缓冲大小（也是中断侧单次 Reserve 的上限）
    static constexpr uint16_t kMaxBufSize = 512;

    /// 软件接收环容量：中断侧（生产者）Reserve/Commit 与线程侧（消费者）Read/Decommit 共用。
    /// 82 kB/s（HiPNUC 1 kHz × 82 B）下 2 KB ≈ 25 ms，足够吸收线程调度抖动；
    /// 约束：必须 ≥ kMaxBufSize（单次 Reserve 不会超过一块 DMA buffer）。
    static constexpr uint32_t kRxRingSize = 2048;

    uint8_t  dma_buf_[2][kMaxBufSize] {};           // 硬件 DMA 双缓冲
    uint8_t  cur_buf_ = 0;                          // 当前提交给硬件的 DMA buffer 索引
    bool     buf_free_[2] = {true, true};   // DMA buffer 空闲状态跟踪

    BipBuffer<kRxRingSize> rx_bip_ {};              // 软件缓冲队列

    uint8_t  tx_data_[kMaxBufSize] {};              // 发送缓冲
    uint8_t* tx_buf_ = tx_data_;                    // 指向发送缓冲
    atomic_t tx_busy_ = ATOMIC_INIT(0);

    const device* dev_ = nullptr;

    Config      config_ {};
    atomic_t    ready_ = ATOMIC_INIT(0);            // 初始化完成标志
    atomic_t    rx_stopped_ = ATOMIC_INIT(0);       // 主动停止 RX 标志（区分异常停止）
    k_spinlock  lock_;
    
    bool ApplyLineConfig();

    void Reset()
    {
        cur_buf_ = 0;
        buf_free_[0] = true;
        buf_free_[1] = true;
        atomic_set(&tx_busy_, 0);
        atomic_set(&ready_, 0);
        atomic_set(&rx_stopped_, 0);
    }
};

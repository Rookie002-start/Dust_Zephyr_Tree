/**
 * @file uart.cpp
 * @author qingyu
 * @brief UART DMA 驱动实现
 * @version 0.5
 * @date 2026-07-23
 */

#include "uart.hpp"
#include <string.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart, LOG_LEVEL_INF);

#pragma message "Compiling Drivers/Communication Uart DMA"

/**
 * @brief UART DMA 事件回调
 *
 * 处理 Zephyr 异步 UART 驱动的事件：数据就绪、缓冲交换、接收停用、发送完成。
 * 数据就绪时存入 BipBuffer，通过信号量通知等待线程。
 *
 * @param dev       UART 设备
 * @param evt       异步事件
 * @param user_data UartDma 实例指针
 */
void uart_dma_callback(const struct device* dev, struct uart_event* evt, void* user_data)
{
    auto* self = static_cast<UartDma*>(user_data);

    auto on_rx_rdy = [self](const uint8_t* data, uint16_t len)
    {
        if (len == 0) return;
        if (self->config_.base_cfg.rx_cb) {
            self->config_.base_cfg.rx_cb(const_cast<uint8_t*>(data), len);
        } else {
            unsigned key = irq_lock();
            uint8_t* p = self->rx_bip_.Reserve(len);
            if (p) { memcpy(p, data, len); self->rx_bip_.Commit(len); }
            irq_unlock(key);
            k_sem_give(&self->sem_);
        }
    };

    auto on_buf_request = [self, dev]()
    {
        int next = self->buf_free_[0] ? 0 : (self->buf_free_[1] ? 1 : -1);
        if (next >= 0) {
            self->buf_free_[next] = false;
            self->cur_buf_ = next;
            uart_rx_buf_rsp(dev, self->dma_buf_[next], UartDma::kMaxBufSize);
        }
    };

    auto on_buf_released = [self, evt]()
    {
        if (evt->data.rx_buf.buf == self->dma_buf_[0])
            self->buf_free_[0] = true;
        else if (evt->data.rx_buf.buf == self->dma_buf_[1])
            self->buf_free_[1] = true;
    };

    // UART_RX_DISABLED 有两种来源：主动 StopRx 和异常/超时停止。
    // STM32 异步驱动在 uart_rx_disable() 内部同步回调本事件，因此主动停止时
    // 绝不能在这里重启接收（否则 StartRx 会拿到 -EBUSY）。
    auto on_rx_disabled = [self, dev]()
    {
        // buf0 即将交回硬件，先占位，避免 BUF_REQUEST 把同一块 buffer 又交回去
        self->cur_buf_     = 0;
        self->buf_free_[0] = false;
        self->buf_free_[1] = true;

        atomic_set(&self->ready_, 0);

        if (atomic_get(&self->rx_stopped_)) {
            return;                                     // 主动停止：等 StartRx 再开
        }

        int32_t timeout = self->config_.base_cfg.rx_timeout;
        int ret = uart_rx_enable(dev, self->dma_buf_[0], UartDma::kMaxBufSize, timeout);
        if (ret < 0) {
            LOG_ERR("rx restart fail %d", ret);
            return;
        }
        atomic_set(&self->ready_, 1);
    };

    auto on_tx_done = [self]()
    {
        atomic_set(&self->tx_busy_, 0);
        if (self->config_.base_cfg.tx_cb) { self->config_.base_cfg.tx_cb(); }
    };

    switch (evt->type)
    {
        case UART_RX_RDY:
            on_rx_rdy(evt->data.rx.buf + evt->data.rx.offset, evt->data.rx.len);
            break;
        case UART_RX_BUF_REQUEST:
            on_buf_request();
            break;
        case UART_RX_BUF_RELEASED:
            on_buf_released();
            break;
        case UART_RX_DISABLED:
            on_rx_disabled();
            break;
        case UART_TX_DONE:
            on_tx_done();
            break;
        default:
            break;
    }
}

/**
 * @brief 初始化 UART DMA 驱动
 *
 * 注册异步回调、应用线配置、启动 DMA 接收。
 *
 * @param dev UART 设备
 * @param cfg 配置（线参数、回调、超时）
 * @return true 初始化成功
 */
bool UartDma::Init(const struct device* dev, const Config& cfg)
{
    dev_    = dev;
    config_ = cfg;
    Reset();

    if (!device_is_ready(dev_)) {
        LOG_ERR("device not ready %s", dev->name);
        return false;
    }

    int ret = uart_callback_set(dev_, uart_dma_callback, this);
    if (ret < 0) {
        LOG_ERR("callback_set fail %d", ret);
        return false;
    }

    if (!ApplyLineConfig()) {
        LOG_ERR("uart_configure failed");
        return false;
    }

    // 先占住 buf0 再 rx_enable：驱动的第一个 UART_RX_BUF_REQUEST 是在
    // uart_rx_enable() 内部同步发出的，否则它会把正在用的 buf0 又交回去。
    buf_free_[0] = false;
    buf_free_[1] = true;

    ret = uart_rx_enable(dev_, dma_buf_[0], kMaxBufSize, config_.base_cfg.rx_timeout);
    if (ret < 0) {
        LOG_ERR("rx_enable fail %d", ret);
        return false;
    }

    atomic_set(&ready_, 1);
    LOG_INF("uart dma ready %s", dev->name);
    return true;
}

/**
 * @brief 从接收缓冲读取数据
 *
 * @param buf     输出缓冲区
 * @param max_len 最大读取长度
 * @return 实际读取字节数，无数据返回 0
 */
uint16_t UartDma::Read(uint8_t* buf, uint16_t max_len)
{
    uint16_t cnt = 0;

    /* ⚠️ 必须与中断侧的 Reserve/Commit 处在同一个临界区里。
       中断侧 on_rx_rdy 是 irq_lock + Reserve + memcpy + Commit；线程侧这里原来是
       「取块 + 拷贝 + Decommit」裸跑，于是线程可能在自己 Decommit 走到一半（region
       A/B 状态只改了一半）时被收包中断打断，ISR 拿这个中间态去 Reserve，算出界外
       指针再把数据 memcpy 过去 —— 实测在 1 kHz/921600（82 kB/s，ISR 每次拷 82 B）
       下就会踩到，表现为 Instruction Access fault 崩机。
       关中断期间只做一次 ≤ max_len 的 memcpy，代价可忽略。 */
    const unsigned int key = irq_lock();
    uint32_t size = 0;
    uint8_t* data = rx_bip_.GetContiguousReadBlock(size);
    if (data != nullptr && size != 0) {
        cnt = (max_len < size) ? max_len : static_cast<uint16_t>(size);
        memcpy(buf, data, cnt);
        rx_bip_.Decommit(cnt);
    }
    irq_unlock(key);

    return cnt;
}

/**
 * @brief DMA 异步发送
 *
 * 拷贝数据到发送缓冲后提交 DMA 发送。上一帧未完成时返回 false。
 *
 * @param data 发送数据
 * @param len  数据长度
 * @return true 提交成功
 */
bool UartDma::Send(const uint8_t* data, uint32_t len)
{
    if (!atomic_get(&ready_)) {
        LOG_ERR("send not ready");
        return false;
    }
    if (len == 0 || !tx_buf_) {
        LOG_ERR("send invalid len=%u tx_buf=%p", len, tx_buf_);
        return false;
    }

    if (atomic_get(&tx_busy_)) {
        LOG_ERR("send busy");
        return false;
    }

    memcpy(tx_buf_, data, len);

    atomic_set(&tx_busy_, 1);
    if (uart_tx(dev_, tx_buf_, len, 0) != 0)
    {
        LOG_ERR("uart_tx fail len=%u", len);
        atomic_set(&tx_busy_, 0);
        return false;
    }
    return true;
}

/**
 * @brief 重新启动 DMA 接收（Stop + SetLineConfig 后调用）
 */
bool UartDma::StartRx()
{
    // 先清掉"主动停止"标志，并占住 buf0，再让硬件开始接收
    atomic_set(&rx_stopped_, 0);

    cur_buf_     = 0;
    buf_free_[0] = false;
    buf_free_[1] = true;

    int ret = uart_rx_enable(dev_, dma_buf_[0], kMaxBufSize, config_.base_cfg.rx_timeout);
    if (ret < 0) {
        LOG_ERR("rx_enable fail %d", ret);
        return false;
    }

    atomic_set(&ready_, 1);
    return true;
}

/**
 * @brief 停止 DMA 接收
 */
void UartDma::StopRx()
{
    k_spinlock_key_t key = k_spin_lock(&lock_);

    // 先声明"这是主动停止"：uart_rx_disable() 会同步回调 UART_RX_DISABLED，
    // 回调据此不再自动重启接收，交回给 StartRx()。
    atomic_set(&rx_stopped_, 1);

    int ret = uart_rx_disable(dev_);
    atomic_set(&ready_, 0);
    if (ret < 0) {
        atomic_set(&rx_stopped_, 0);                    // 本来就没在收，撤销标志
    }
    k_spin_unlock(&lock_, key);
}

/**
 * @brief 设置波特率（仅停止状态下可用）
 *
 * @param baud 波特率
 */
void UartDma::SetBaudrate(uint32_t baud)
{
    k_spinlock_key_t key = k_spin_lock(&lock_);

    if (atomic_get(&ready_)) {
        k_spin_unlock(&lock_, key);
        LOG_ERR("cannot set baudrate while uart is running");
        return;
    }

    config_.line_cfg.baudrate = baud;
    k_spin_unlock(&lock_, key);

    if (!ApplyLineConfig()) {
        LOG_ERR("uart_configure failed");
    }
}

/**
 * @brief 设置全部线参数（仅停止状态下可用）
 *
 * @param cfg 完整的 uart_config
 */
void UartDma::SetLineConfig(const uart_config& cfg)
{
    k_spinlock_key_t key = k_spin_lock(&lock_);

    if (atomic_get(&ready_)) {
        k_spin_unlock(&lock_, key);
        LOG_ERR("cannot set line config while uart is running");
        return;
    }

    config_.line_cfg = cfg;
    k_spin_unlock(&lock_, key);

    if (!ApplyLineConfig()) {
        LOG_ERR("uart_configure failed");
    }
}

/**
 * @brief 应用线参数配置到硬件
 * @return true 配置成功
 */
bool UartDma::ApplyLineConfig()
{
    const auto& lc = config_.line_cfg;

    if (lc.baudrate == 0) {
        LOG_ERR("line_cfg: not fully initialized");
        return false;
    }

    return uart_configure(dev_, &config_.line_cfg) == 0;
}

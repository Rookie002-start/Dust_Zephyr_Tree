/**
 * @file ringbuf.hpp
 * @author qingyu
 * @brief 环形缓冲区（Ring Buffer）— 基于 UART 驱动使用的环形缓冲提取
 * @version 0.2
 * @date 2026-07-01
 *
 * @par 使用示例
 * @code
 *   alg::buffer::RingBuf<256> buf;
 *
 *   uint8_t data[] = {1, 2, 3};
 *   buf.Write(data, 3);
 *
 *   uint8_t out[16];
 *   uint32_t n = buf.Read(out, sizeof(out));
 * @endcode
 */

#pragma once

#pragma message "Compiling Algorithm/Buffer/RingBuf"

#include <cstdint>
#include <cstring>

namespace alg::buffer {

/**
 * @brief 环形缓冲区（单字节 FIFO），模板版本
 *
 * 基于 UART 驱动中使用的环形缓冲模式：
 *   - 内部数组内嵌在类中，不动态分配
 *   - 写满时丢弃新数据（保留旧数据）
 *   - 保留一格区分空/满状态
 *   - 非线程安全，调用方自行保证互斥
 *
 * @tparam Capacity  缓冲区总大小（字节），实际可用容量为 Capacity - 1
 */
template <uint32_t Capacity>
class RingBuf final
{
public:
    /**
     * @brief 写入数据（满则丢弃）
     * @param data  源数据
     * @param len   写入长度
     * @return      实际写入字节数
     */
    uint32_t Write(const uint8_t *data, uint32_t len)
    {
        uint32_t used = Available();
        uint32_t free = Capacity - 1 - used;   // 保留一格

        if (len > free) {
            len = free;
        }
        if (len == 0) {
            return 0;
        }

        uint32_t to_end = Capacity - head_;
        if (len <= to_end) {
            memcpy(&buf_[head_], data, len);
        } else {
            memcpy(&buf_[head_], data, to_end);
            memcpy(&buf_[0], data + to_end, len - to_end);
        }

        head_ = (head_ + len) % Capacity;
        return len;
    }

    /**
     * @brief 读取数据
     * @param buf     目标缓冲区
     * @param max_len 期望读取的最大长度
     * @return        实际读取字节数
     */
    uint32_t Read(uint8_t *buf, uint32_t max_len)
    {
        uint32_t avail = Available();
        uint32_t cnt   = (max_len < avail) ? max_len : avail;

        if (cnt == 0) {
            return 0;
        }

        uint32_t to_end = Capacity - tail_;
        if (cnt <= to_end) {
            memcpy(buf, &buf_[tail_], cnt);
        } else {
            memcpy(buf, &buf_[tail_], to_end);
            memcpy(buf + to_end, &buf_[0], cnt - to_end);
        }

        tail_ = (tail_ + cnt) % Capacity;
        return cnt;
    }

    uint32_t Available() const
    {
        return (head_ - tail_ + Capacity) % Capacity;
    }

    static constexpr uint32_t GetCapacity() { return Capacity - 1; }

    bool Empty() const { return head_ == tail_; }

    bool Full() const
    {
        return ((head_ + 1) % Capacity) == tail_;
    }

    void Clear()
    {
        head_ = 0;
        tail_ = 0;
    }

    /**
     * @brief 丢弃指定字节数
     * @param len  期望丢弃的字节数
     * @return     实际丢弃字节数
     */
    uint32_t Discard(uint32_t len)
    {
        uint32_t avail = Available();
        uint32_t n     = (len < avail) ? len : avail;
        tail_ = (tail_ + n) % Capacity;
        return n;
    }

private:
    alignas(64) uint8_t buf_[Capacity] = {};
    uint32_t head_ = 0;    // 写位置
    uint32_t tail_ = 0;    // 读位置
};

} // namespace alg::buffer

/**
 * @file BipBuffer.hpp
 * @author qingyu
 * @brief 双区环形缓冲区 (BipBuffer)，保证连续内存访问，零拷贝 DMA 友好
 *
 * BipBuffer 是一种特殊的环形缓冲区，保证任何可读数据块在物理内存中
 * 连续。维护两个区域 A 和 B，写入回绕时在头部创建 B 区，读取完 A 后
 * B 自动提升为 A。数据永远不会被分段。
 *
 * @par 使用场景
 * - DMA 直接写入的流式数据接收
 * - 数据包解析（连续内存，无需处理回绕）
 * - 生产者-消费者模式，单线程使用
 *
 * @par 典型用法（DMA 写入）
 * @code
 * BipBuffer<4096> buf;
 *
 * // 1. Reserve 获取写入指针
 * uint8_t* p = buf.Reserve(2048);
 * if (!p) {  // buffer 满  }
 *
 * // 2. DMA 写入数据到 p...
 * //    (零拷贝，DMA 直接写 buffer)
 *
 * // 3. Commit 提交已写入字节数
 * buf.Commit(received_bytes);
 *
 * // 4. GetContiguousReadBlock 读取
 * uint32_t size;
 * uint8_t* data = buf.GetContiguousReadBlock(size);
 * parse_packet(data, size);
 *
 * // 5. Decommit 释放已消费数据
 * buf.Decommit(size);
 * @endcode
 *
 * @par 典型用法（memcpy 写入）
 * @code
 * BipBuffer<1024> buf;
 *
 * // 写入
 * uint32_t n = 256;
 * uint8_t* p = buf.Reserve(n);
 * if (p) {
 *     std::memcpy(p, source_data, n);
 *     buf.Commit(n);
 * }
 *
 * // 读取并消费
 * uint32_t rs = 0;
 * uint8_t* r = buf.GetContiguousReadBlock(rs);
 * if (r) {
 *     process(r, rs);
 *     buf.Decommit(rs);
 * }
 * @endcode
 *
 * @par ForceWrap 用法（尾部过小时跳过）
 * @code
 * BipBuffer<1024> buf;
 * buf.Reserve(900); buf.Commit(900); // A = 0~899
 * buf.Decommit(800);                 // A = 800~99, 头部空闲 800
 * // 尾部剩 124, ForceWrap 直接去头部
 * uint8_t* p = buf.ReserveForceWrap(200);
 * if (p) buf.Commit(200);            // B = 0~199
 * @endcode
 *
 * @version 0.1
 * @date 2026-07-01
 * @copyright Copyright (c) 2026
 */

#include <cstdint>
#include <algorithm>
#include <cassert>

template <uint32_t Capacity>
/**
 * @brief 双区环形缓冲 — 保证连续内存访问，零拷贝 DMA 友好
 *
 * 写入回绕时在头部创建 B 区，读取完 A 后 B 自动提升为 A。
 * 数据永远不会被分段，适合 DMA 直接写入的流式数据接收。
 *
 * @tparam Capacity 缓冲总大小（字节）
 */
class BipBuffer
{
public:
    BipBuffer() = default;

    /**
     * @brief 预留一块连续空间用于写入
     * @param size 请求的写入长度
     * @return 写入指针，失败返回 nullptr
     */
    uint8_t* Reserve(uint32_t size)
    {
        if (reserved_start_ || size == 0 || size > Capacity)
            return nullptr;

        if (size > Capacity - GetUsedSize())
            return nullptr;

        if (region_b_size_ > 0)
        {
            // 双区模式：在 B 区之后、A 区之前（gap 空间）分配
            assert(buffer_ + region_b_size_ <= region_a_start_ && "BipBuffer state corrupted");
            uint32_t gap = static_cast<uint32_t>(region_a_start_ - (buffer_ + region_b_size_));
            if (size <= gap) {
                reserved_start_ = buffer_ + region_b_size_;
                reserved_size_ = size;
                reserved_is_b_ = true;
                return reserved_start_;
            }
            return nullptr;
        }

        // 普通模式：先试末尾
        uint32_t write_pos = region_a_start_ ? static_cast<uint32_t>(region_a_start_ - buffer_) + region_a_size_ : 0;
        uint32_t free_at_end = Capacity - write_pos;
        if (size <= free_at_end) {
            reserved_start_ = buffer_ + write_pos;
            reserved_size_ = size;
            reserved_is_b_ = false;
            return reserved_start_;
        }

        // 末尾不够，回绕到头部（创建 B 区）
        return reserve_at_head_(size);
    }

    /**
     * @brief 强制从头部写入（跳过尾部剩余空间）
     *
     * 尾部剩一点不够装整个包时有用，避免 CPU 判断尾部空间后白跑一次。
     *
     * @param size 请求的写入长度
     * @return 写入指针，失败返回 nullptr
     */
    uint8_t* ReserveForceWrap(uint32_t size)
    {
        if (reserved_start_ || size == 0 || size > Capacity)
            return nullptr;

        if (size > Capacity - GetUsedSize())
            return nullptr;

        // 已在双区模式 → 走正常的 Reserve（B 区后已有 gap）
        if (region_b_size_ > 0)
            return Reserve(size);

        // 跳过尾部，直接试回绕头部
        return reserve_at_head_(size);
    }

    /**
     * @brief 提交之前 Reserve 的空间
     *
     * @param size 实际写入的字节数（<= Reserve 请求的大小）
     */
    void Commit(uint32_t size)
    {
        assert(reserved_start_ != nullptr && "Commit without Reserve");

        if (size == 0) {
            reserved_start_ = nullptr;
            reserved_size_ = 0;
            reserved_is_b_ = false;
            return;
        }
        size = std::min(size, reserved_size_);

        if (reserved_is_b_)
        {
            region_b_size_ += size;
        }
        else
        {
            if (!region_a_start_)
                region_a_start_ = reserved_start_;
            region_a_size_ += size;
        }

        reserved_start_ = nullptr;
        reserved_size_ = 0;
    }

    /**
     * @brief 获取第一个连续可读块
     *
     * @param[out] out_size 可读数据长度
     * @return 数据指针，无数据返回 nullptr
     */
    uint8_t* GetContiguousReadBlock(uint32_t& out_size)
    {
        if (region_a_size_ > 0) {
            out_size = region_a_size_;
            return region_a_start_;
        }
        if (region_b_size_ > 0) {
            out_size = region_b_size_;
            return buffer_;
        }
        out_size = 0;
        return nullptr;
    }

    /**
     * @brief 从可读块头部消费 bytes，释放空间
     *
     * @param size 要消费的字节数
     */
    void Decommit(uint32_t size)
    {
        while (size > 0)
        {
            if (region_a_size_ > 0)
            {
                if (size >= region_a_size_)
                {
                    // A 区消费完，把 B 区提成 A 区，循环处理残余
                    size -= region_a_size_;
                    region_a_start_ = buffer_;
                    region_a_size_ = region_b_size_;
                    region_b_size_ = 0;
                }
                else
                {
                    region_a_start_ += size;
                    region_a_size_ -= size;
                    size = 0;
                }
            }
            else if (region_b_size_ > 0)
            {
                if (size >= region_b_size_) {
                    region_b_size_ = 0;
                } else {
                    // B 区从 buffer_ 开始，直接偏移
                    size_t offset = size;
                    region_b_size_ -= offset;
                    // B 起始隐含为 buffer_ + offset，但 B 提升为 A 时处理
                    // 这里直接存储剩余大小，实际数据从 buffer_ + offset 开始
                    // 简化处理：把 B 拷到 A
                    region_a_start_ = buffer_ + offset;
                    region_a_size_ = region_b_size_;
                    region_b_size_ = 0;
                }
                size = 0;
            }
            else
            {
                size = 0;
            }
        }
    }

    /** @brief 缓冲区总容量 */
    static constexpr uint32_t GetCapacity() { return Capacity; }
    /** @brief 当前已占用大小 */
    size_t GetUsedSize() const { return region_a_size_ + region_b_size_; }

    /** @brief 丢弃全部数据和未提交预留 */
    void Reset()
    {
        region_a_start_ = nullptr;
        region_a_size_  = 0;
        region_b_size_  = 0;
        reserved_start_ = nullptr;
        reserved_size_  = 0;
        reserved_is_b_  = false;
    }

private:
    // 回绕到头部分配（普通模式下公共逻辑）
    uint8_t* reserve_at_head_(uint32_t size)
    {
        if (region_a_start_) {
            uint32_t free_at_head = static_cast<uint32_t>(region_a_start_ - buffer_);
            if (size <= free_at_head) {
                reserved_start_ = buffer_;
                reserved_size_ = size;
                reserved_is_b_ = true;
                return reserved_start_;
            }
        }
        return nullptr;
    }

    alignas(64) uint8_t buffer_[Capacity] {};

    uint8_t* region_a_start_ = nullptr;     // A 区起始
    uint32_t region_a_size_  = 0;           // A 区大小
    uint32_t region_b_size_  = 0;           // B 区大小（起始始终为 buffer_[0]）
    uint8_t* reserved_start_ = nullptr;     // 预留区起始
    uint32_t reserved_size_  = 0;           // 预留区大小
    bool reserved_is_b_      = false;       // 预留区将提交为 B 区还是 A 区
};

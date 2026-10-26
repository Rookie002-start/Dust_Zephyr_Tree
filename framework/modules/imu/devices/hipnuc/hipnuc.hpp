/**
 * @file hipnuc.hpp
 * @author qingyu
 * @brief HiPNUC HI 系列（0x5A 0xA5 / 0x91 组合帧）串口 IMU Source
 * @version 0.1
 * @date 2026-09-10
 *
 * HiPNUC 模块内部已完成姿态融合，直接输出工程单位
 * （欧拉角 / 四元数 / 角速度 / 加速度 / 磁力计 / 气压）。
 *
 * 与 BMI088 / ICM42688P 的差异：
 * - 传输：UART 帧协议（模块主动上报），不是 SPI 寄存器读写
 * - 数据：直接是工程量，没有“原始计数 → *Convert*() 换算”这条链路
 * - 姿态：模块自带融合，Read() 出去的样本带 fused 标志，上层旁路 EKF
 * - 标定：出厂已标定，不做驱动侧零偏采集
 * - 加热：成品模块无加热器
 *
 * 因此本类直接继承 Source，而不继承 CalibSource。
 *
 * @par 帧格式（布局 A，与 HiPNUC 手册字段顺序一致）
 * @code
 *   SOF(2, 0x5A 0xA5) | LEN(2, LE) | CRC16(2, LE) | DATA(LEN)
 * @endcode
 * 兼容部分固件使用的布局 B：
 * @code
 *   SOF(2, 0x5A 0xA5) | LEN(2, LE) | DATA(LEN) | CRC16(2, LE)
 * @endcode
 * 两种布局总开销都是 6 字节，靠 payload 首字节 tag 自动判定。
 *
 * @par 0x91 组合数据帧字段（payload 起点，小端，合计 76 字节）
 * @code
 *   0  tag           u8      = 0x91
 *   1  main_status   u16
 *   3  temperature   i8      °C
 *   4  air_pressure  f32
 *   8  system_time   u32
 *   12 acc_b         f32[3]  m/s²
 *   24 gyr_b         f32[3]  °/s
 *   36 mag_b         f32[3]
 *   48 roll          f32     deg
 *   52 pitch         f32     deg
 *   56 yaw           f32     deg
 *   60 quat          f32[4]  WXYZ
 * @endcode
 */

#pragma once

#include "imu_device_layer.hpp"
#include "uart.hpp"

#include <cstdint>
#include <cstring>

namespace hipnuc {

/**
 * @brief HiPNUC HI 系列串口 IMU 数据源
 *
 * UART DMA 收字节 → 跨 chunk 累积 → 状态机找 0x5A 0xA5 → 校验并解析 0x91 帧。
 * Read() 在 ImuManager 线程上下文轮询调用，取出最新一帧并转成统一 Sample。
 */
class Hipnuc final : public Source
{
public:
    bool Init() override;
    bool LateInit() override;

    /**
     * @brief 取出一帧最新样本
     *
     * 先把 UART 中所有可用字节喂进解析器，若期间解析出完整 0x91 帧则返回 true。
     * 输出的 gyro 已换算为 rad/s，欧拉角为 deg，quaternion 为 WXYZ。
     *
     * @param sample 输出样本（fused = true）
     * @return true  本周期拿到新帧
     * @return false 无新帧
     */
    bool Read(Sample& sample) override;

    /** 模块出厂已标定，驱动侧不做零偏采集 */
    bool Calibrate() override { return true; }

    /** 气压（附加数据，调试/扩展用，不进 topic 契约） */
    float GetPressure() const { return pressure_; }

    /** 磁力计（附加数据，调试/扩展用，不进 topic 契约） */
    void GetMag(float* out) const
    {
        if (out == nullptr) {
            return;
        }
        out[0] = mag_[0];
        out[1] = mag_[1];
        out[2] = mag_[2];
    }

private:
    /* ---------------- 协议常量 ---------------- */

    /* 串口波特率不再在这里硬编码：由设备树决定（hipnuc-uart 节点的 current-speed），
       见 hipnuc.cpp 里的 kBaudrate —— 改模块输出波特率时只改 overlay/板级 dts。 */

    static constexpr uint8_t  kSof0           = 0x5A;   // 帧头
    static constexpr uint8_t  kSof1           = 0xA5;
    static constexpr uint8_t  kTag            = 0x91;   // 组合数据帧 tag

    static constexpr uint16_t kPayloadLen     = 76;     // 0x91 payload 长度
    static constexpr uint16_t kFrameOverhead  = 6;      // SOF(2) + LEN(2) + CRC16(2)
    static constexpr uint16_t kPayloadOffsetA = 6;      // SOF LEN CRC DATA
    static constexpr uint16_t kPayloadOffsetB = 4;      // SOF LEN DATA CRC
    static constexpr uint16_t kMaxPayloadLen  = 128;    // LEN 合理性上限
    static constexpr uint16_t kRxBufSize      = 256;    // 跨 chunk 累积缓冲

    static constexpr float kDeg2Rad = 0.01745329251994329577f;

    /* ---------------- 接收与解析 ---------------- */

    UartDma  uart_ {};

    uint8_t  rx_buf_[kRxBufSize] {};
    uint16_t rx_len_ = 0;

    /* ---------------- 最新一帧解析结果 ---------------- */

    float gyro_deg_[3]   {0.0f, 0.0f, 0.0f};              // °/s
    float accel_[3]      {0.0f, 0.0f, 0.0f};              // m/s²
    float mag_[3]        {0.0f, 0.0f, 0.0f};
    float euler_deg_[3]  {0.0f, 0.0f, 0.0f};              // roll / pitch / yaw
    float quat_[4]       {1.0f, 0.0f, 0.0f, 0.0f};        // WXYZ
    float pressure_      = 0.0f;
    float temp_          = 0.0f;
    float yaw_total_deg_ = 0.0f;                          // 解包络后的累计航向

    bool  has_last_yaw_  = false;
    float last_yaw_deg_  = 0.0f;
    bool  has_new_frame_ = false;

    /* ---------------- 内部实现 ---------------- */

    void ResetRx();
    void Drain();
    void ParseChunk(const uint8_t* data, uint16_t len);
    void ProcessBuffer();
    bool ParseFrame(const uint8_t* frame, uint16_t data_len);
    void ParsePayload(const uint8_t* payload);
    void Consume(uint16_t n);

#if defined(CONFIG_DUST_MOD_DEV_IMU_HIPNUC_CRC)
    static uint16_t Crc16(const uint8_t* data, uint16_t len, uint16_t crc);
    static bool     VerifyCrcLayoutA(const uint8_t* frame, uint16_t data_len);
    static bool     VerifyCrcLayoutB(const uint8_t* frame, uint16_t data_len);
#endif

    template <typename T>
    static T ReadLE(const uint8_t* p)
    {
        T v {};
        memcpy(&v, p, sizeof(T));
        return v;
    }
};

} // namespace hipnuc

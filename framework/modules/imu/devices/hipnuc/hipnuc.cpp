/**
 * @file hipnuc.cpp
 * @author qingyu
 * @brief HiPNUC HI 系列（0x5A 0xA5 / 0x91 组合帧）串口 IMU Source 实现
 * @version 0.1
 * @date 2026-09-10
 */

#include "hipnuc.hpp"
#include "imu.hpp"

#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#pragma message "Compiling Modules/Imu/Devices/HiPNUC"

LOG_MODULE_REGISTER(hipnuc, LOG_LEVEL_INF);

namespace hipnuc {

namespace {

/* 0x91 组合帧字段偏移（相对 payload 起点，小端） */
constexpr uint16_t kOffMainStatus = 1;
constexpr uint16_t kOffTemp       = 3;
constexpr uint16_t kOffPressure   = 4;
constexpr uint16_t kOffSystemTime = 8;
constexpr uint16_t kOffAccel      = 12;
constexpr uint16_t kOffGyro       = 24;
constexpr uint16_t kOffMag        = 36;
constexpr uint16_t kOffRoll       = 48;
constexpr uint16_t kOffPitch      = 52;
constexpr uint16_t kOffYaw        = 56;
constexpr uint16_t kOffQuat       = 60;

/* 串口波特率：**由设备树决定**（hipnuc-uart 节点的 current-speed）。
   驱动不再硬编码 —— 改模块输出波特率时只改 overlay/板级 dts，这里自动跟随。
   （注意：UartDma 会把这个值 uart_configure() 到硬件上，会覆盖 DT 的默认值，
     所以必须显式取 DT 的值，否则会退回成别的数。） */
constexpr uint32_t kBaudrate = DT_PROP(DT_ALIAS(hipnuc_uart), current_speed);

#if defined(CONFIG_DUST_MOD_DEV_IMU_HIPNUC_CRC)
/* CRC-16/XMODEM 查表（poly 0x1021，MSB 先出，输入/输出不反转、末尾不异或）：
   与原来那个"每字节再循环 8 次移位"的实现**完全等价**（已逐字节对拍），
   但把 76 字节 × 8 = 608 次迭代换成 76 次查表 —— 1 kHz 下这是每帧最大的一笔
   CPU 开销（实测解析 ~80 µs/帧，占满一圈 1.5 ms 里的 ~0.5 ms）。
   校验值："123456789" -> 0x31C3。 */
constexpr uint16_t kWcrcTable[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0,
};
#endif

} // namespace

/**
 * @brief 初始化 UART DMA 接收
 *
 * 使用蜂鸣器/遥控器之外的独立 UART（设备树 alias: hipnuc-uart）。
 * 采用 UartDma 的软队列模式（不注册 rx_cb），解析放在 Read() 的线程上下文里做，
 * 避免在中断里跑浮点解析。
 */
bool Hipnuc::Init()
{
    ResetRx();

    UartDma::Config cfg {};
    cfg.line_cfg.baudrate   = kBaudrate;          // 来自设备树
    cfg.base_cfg.rx_timeout = 1000;

    if (!uart_.Init(DEVICE_DT_GET(DT_ALIAS(hipnuc_uart)), cfg)) {
        LOG_ERR("hipnuc uart init failed");
        return false;
    }

    LOG_INF("hipnuc uart ready @%u (devicetree current-speed)", static_cast<unsigned>(kBaudrate));
    return true;
}

/**
 * @brief 延迟初始化
 *
 * HiPNUC 出厂已完成标定，没有需要从 flash 恢复的零偏参数，
 * 因此这里不做任何事，保留接口仅为对齐 Source 生命周期。
 */
bool Hipnuc::LateInit()
{
    return true;
}

/**
 * @brief 取出一帧最新样本
 *
 * 先把 UART 中所有可用字节喂进解析器，再判断本周期是否产出了新帧。
 * 帧率通常 100~200Hz，上层 1ms 轮询，多数周期会返回 false。
 */
bool Hipnuc::Read(Sample& sample)
{
    Drain();

    if (!has_new_frame_) {
        return false;
    }
    has_new_frame_ = false;

    for (uint8_t axis = 0; axis < 3; axis++) {
        // 模块输出 °/s，统一到架构约定的 rad/s（EKF 与 topic 都用 rad/s）
        sample.gyro[axis]  = gyro_deg_[axis] * kDeg2Rad;
        sample.accel[axis] = accel_[axis];
    }

    for (uint8_t i = 0; i < 4; i++) {
        sample.quaternion[i] = quat_[i];
    }

    sample.temp      = temp_;
    sample.roll      = euler_deg_[0] * kDeg2Rad;
    sample.pitch     = euler_deg_[1] * kDeg2Rad;
    sample.yaw       = euler_deg_[2] * kDeg2Rad;
    sample.yaw_total = yaw_total_deg_ * kDeg2Rad;
    sample.fused     = true;

    return true;
}

/**
 * @brief 清空接收解析状态
 */
void Hipnuc::ResetRx()
{
    rx_len_       = 0;
    has_new_frame_ = false;
    has_last_yaw_  = false;
    last_yaw_deg_  = 0.0f;
    yaw_total_deg_ = 0.0f;
}

/**
 * @brief 把 UART 软队列里的数据全部取出并解析
 */
void Hipnuc::Drain()
{
    uint8_t chunk[64];

    for (;;)
    {
        const uint16_t n = uart_.Read(chunk, sizeof(chunk));
        if (n == 0) {
            break;
        }
        ParseChunk(chunk, n);
    }
}

/**
 * @brief 追加一段原始字节到累积缓冲并尝试解析
 *
 * 与旧驱动不同，这里保留了跨 chunk 的半帧：一帧 82 字节，DMA 软队列按块返回，
 * 帧被切开是常态，必须在缓冲里攒齐再解析。
 *
 * @param data 本块数据
 * @param len  本块长度
 */
void Hipnuc::ParseChunk(const uint8_t* data, uint16_t len)
{
    while (len > 0)
    {
        const uint16_t space = static_cast<uint16_t>(kRxBufSize - rx_len_);

        if (space == 0) {
            // 缓冲已满仍凑不出完整帧 → 丢最老一个字节重新同步，避免死锁
            Consume(1);
            ProcessBuffer();
            continue;
        }

        const uint16_t n = (len < space) ? len : space;
        memcpy(rx_buf_ + rx_len_, data, n);
        rx_len_ = static_cast<uint16_t>(rx_len_ + n);
        data   += n;
        len    = static_cast<uint16_t>(len - n);

        ProcessBuffer();
    }
}

/**
 * @brief 帧同步状态机
 *
 * 每轮：对齐 SOF → 读 LEN → 攒够整帧 → 校验解析 → 前移。
 * 校验不过只滑动一个字节重新找 SOF，避免误判帧头时整帧丢数据。
 */
void Hipnuc::ProcessBuffer()
{
    for (;;)
    {
        // 1. 对齐帧头
        if (rx_len_ < 2) {
            return;
        }
        if (rx_buf_[0] != kSof0 || rx_buf_[1] != kSof1) {
            Consume(1);
            continue;
        }

        // 2. 读长度
        if (rx_len_ < kFrameOverhead) {
            return;
        }
        const uint16_t data_len = ReadLE<uint16_t>(rx_buf_ + 2);
        if (data_len < 1 || data_len > kMaxPayloadLen) {
            Consume(2);         // 假帧头，跳过 SOF 继续找
            continue;
        }

        // 3. 攒够整帧
        const uint16_t frame_len = static_cast<uint16_t>(data_len + kFrameOverhead);
        if (rx_len_ < frame_len) {
            return;
        }

        // 4. 校验 + 解析
        if (ParseFrame(rx_buf_, data_len)) {
            Consume(frame_len);
            continue;
        }

        Consume(1);
    }
}

/**
 * @brief 校验并解析一帧
 *
 * 布局 A（SOF LEN CRC DATA）与布局 B（SOF LEN DATA CRC）都尝试，
 * 以 payload 首字节 == 0x91 判定布局；开启 CRC 校验时再叠加 CRC 判定。
 *
 * @param frame    完整帧起始地址
 * @param data_len 帧内 LEN 字段
 * @return true    命中 0x91 并完成解析
 */
bool Hipnuc::ParseFrame(const uint8_t* frame, uint16_t data_len)
{
    const uint8_t* payload_a = frame + kPayloadOffsetA;
    const uint8_t* payload_b = frame + kPayloadOffsetB;

    const bool a_ok = (data_len >= kPayloadLen) && (payload_a[0] == kTag);
    const bool b_ok = (data_len >= kPayloadLen) && (payload_b[0] == kTag);

#if defined(CONFIG_DUST_MOD_DEV_IMU_HIPNUC_CRC)
    if (a_ok && VerifyCrcLayoutA(frame, data_len)) {
        ParsePayload(payload_a);
        return true;
    }
    if (b_ok && VerifyCrcLayoutB(frame, data_len)) {
        ParsePayload(payload_b);
        return true;
    }
#else
    if (a_ok) {
        ParsePayload(payload_a);
        return true;
    }
    if (b_ok) {
        ParsePayload(payload_b);
        return true;
    }
#endif

    return false;
}

/**
 * @brief 解析 0x91 payload
 *
 * 轴向补偿沿用旧驱动：gyro Y 取负、pitch 取负（模块在新板子上的安装方向补偿）。
 * 四元数按模块原值透传，不做同样的补偿。
 *
 * @param payload payload 起始地址（长度 >= 76）
 */
void Hipnuc::ParsePayload(const uint8_t* payload)
{
    temp_     = static_cast<float>(static_cast<int8_t>(payload[kOffTemp]));
    pressure_ = ReadLE<float>(payload + kOffPressure);

    for (uint8_t axis = 0; axis < 3; axis++) {
        accel_[axis] = ReadLE<float>(payload + kOffAccel + axis * 4);
        mag_[axis]   = ReadLE<float>(payload + kOffMag   + axis * 4);
    }

    // 角速度：X/Y/Z 对应 roll/pitch/yaw；Y 取负为安装方向补偿
    gyro_deg_[0] =  ReadLE<float>(payload + kOffGyro + 4);
    gyro_deg_[1] = -ReadLE<float>(payload + kOffGyro);
    gyro_deg_[2] =  ReadLE<float>(payload + kOffGyro + 8);

    // 欧拉角：pitch 取负为安装方向补偿
    euler_deg_[0] =  ReadLE<float>(payload + kOffRoll);
    euler_deg_[1] = -ReadLE<float>(payload + kOffPitch);
    euler_deg_[2] =  ReadLE<float>(payload + kOffYaw);

    for (uint8_t i = 0; i < 4; i++) {
        quat_[i] = ReadLE<float>(payload + kOffQuat + i * 4);
    }

    // yaw 解包络：模块 yaw 在 ±180° 跳变，这里累积成连续航向角
    if (!has_last_yaw_)
    {
        has_last_yaw_  = true;
        last_yaw_deg_  = euler_deg_[2];
        yaw_total_deg_ = euler_deg_[2];
    }
    else
    {
        float delta = euler_deg_[2] - last_yaw_deg_;
        if (delta > 180.0f) {
            delta -= 360.0f;
        } else if (delta < -180.0f) {
            delta += 360.0f;
        }
        yaw_total_deg_ += delta;
        last_yaw_deg_   = euler_deg_[2];
    }

    has_new_frame_ = true;
}

/**
 * @brief 从累积缓冲头部丢弃 n 个字节
 */
void Hipnuc::Consume(uint16_t n)
{
    if (n >= rx_len_) {
        rx_len_ = 0;
        return;
    }

    rx_len_ = static_cast<uint16_t>(rx_len_ - n);
    memmove(rx_buf_, rx_buf_ + n, rx_len_);
}

#if defined(CONFIG_DUST_MOD_DEV_IMU_HIPNUC_CRC)

/**
 * @brief CRC-16（poly 0x1021，初值 0x0000，不反转、末尾不异或 → XMODEM）
 *
 * 注意：多项式需与模块固件/旧驱动 CRC.h 一致；不一致时请关闭
 * CONFIG_DUST_MOD_DEV_IMU_HIPNUC_CRC，改用 tag + LEN 结构校验。
 *
 * @param data 数据
 * @param len  长度
 * @param crc  上一次的 CRC 值（首段传 0x0000）
 * @return 累进后的 CRC
 */
uint16_t Hipnuc::Crc16(const uint8_t* data, uint16_t len, uint16_t crc)
{
    /* 查表实现（poly 0x1021，MSB 先出）：与原来的逐位移位版本完全等价，
       每字节只做一次查表 + 一次异或。见 kWcrcTable 的说明。 */
    for (uint16_t i = 0; i < len; i++)
    {
        crc = static_cast<uint16_t>((crc << 8) ^
              kWcrcTable[((crc >> 8) ^ data[i]) & 0x00FFU]);
    }
    return crc;
}

/**
 * @brief 布局 A 校验：CRC 位于 [4..5]，覆盖 SOF+LEN+DATA
 */
bool Hipnuc::VerifyCrcLayoutA(const uint8_t* frame, uint16_t data_len)
{
    const uint16_t expected = ReadLE<uint16_t>(frame + 4);
    uint16_t crc = Crc16(frame, 4, 0x0000);
    crc = Crc16(frame + kPayloadOffsetA, data_len, crc);
    return crc == expected;
}

/**
 * @brief 布局 B 校验：CRC 位于 payload 之后，覆盖 SOF+LEN+DATA
 */
bool Hipnuc::VerifyCrcLayoutB(const uint8_t* frame, uint16_t data_len)
{
    const uint16_t expected = ReadLE<uint16_t>(frame + kPayloadOffsetB + data_len);
    uint16_t crc = Crc16(frame, 4, 0x0000);
    crc = Crc16(frame + kPayloadOffsetB, data_len, crc);
    return crc == expected;
}

#endif // CONFIG_DUST_MOD_DEV_IMU_HIPNUC_CRC

REGISTER_IMU(Hipnuc, hipnuc);

} // namespace hipnuc

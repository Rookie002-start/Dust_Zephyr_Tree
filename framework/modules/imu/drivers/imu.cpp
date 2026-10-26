/**
 * @file imu.cpp
 * @author qingyu
 * @brief IMU 数据源管理、加热控制与姿态发布
 * @version 0.1
 * @date 2026-06-01
 */

#include "imu.hpp"
#if defined(CONFIG_WDT_SUPERVISOR)
#include "health.hpp"
#endif
#include <cstddef>
#include <cstring>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_DUST_MOD_DEV_IMU_HEATER
#include "heater.hpp"
#endif

#pragma message "Compiling Modules/Imu/Drivers/Imu"

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

extern const imu::ImuEntry __imu_start[];
extern const imu::ImuEntry __imu_end[];

namespace imu {

/**
 * @brief 初始化 IMU 管理器
 *
 * 按依赖顺序串联各子模块：
 * 选择数据源 → 初始化驱动 → 初始化加热器（CONFIG_DUST_MOD_DEV_IMU_HEATER 时）
 * → 可选自动校准（AutoCalib）或辨识（AutoIdent）
 * → LateInit：flash 有校准数据则加载，无则保持 reg 默认
 * → 初始化姿态解算器。
 *
 * 校准数据优先级：
 * - AutoCalib 模式：在线采集 → 计算 offset → 写入 flash
 * - Normal 模式：不跑 Calibrate，由 LateInit 决定是否从 flash 加载
 * - 既无 flash 数据也未跑 Calibrate：使用 reg 出厂默认值
 *
 * @param mode ImuStartMode::Normal / AutoCalib / AutoIdent
 */
bool ImuManager::Init(ImuStartMode mode)
{
    ready_ = false;

    if (!InitSource()) {
        LOG_ERR("InitSource failed");
        return false;
    }
    LOG_INF("InitSource done");

#ifdef CONFIG_DUST_MOD_DEV_IMU_HEATER
    if (!heater_.Init()) {
        LOG_ERR("heater init failed");
        return false;
    }

    LOG_INF("start preheat");
    Preheat();
#endif

    if (mode == ImuStartMode::AutoCalib)
    {
        if (!source_->Calibrate()) { 
            LOG_ERR("calibrate failed"); 
            return false; 
        }
        LOG_INF("calibrate done");
    }

    source_->LateInit();

    attitude_.Init();

    ready_ = true;
    LOG_INF("imu ready");
    
    return true;
}

/**
 * @brief 把当前样本转成 topic 消息
 *
 * 融合型数据源（fused == true，如 HiPNUC）已经给出姿态，
 * 直接搬运到消息结构，跳过 EKF，避免二次融合；
 * 寄存器型数据源（BMI088 / ICM42688P）仍走 QuaternionEkf 解算。
 */
void ImuManager::PublishSample()
{
    if (sample_.fused)
    {
        memcpy(pub_.quaternion, sample_.quaternion, sizeof(pub_.quaternion));
        memcpy(pub_.gyro,       sample_.gyro,       sizeof(pub_.gyro));

        pub_.roll        = sample_.roll;
        pub_.pitch       = sample_.pitch;
        pub_.yaw         = sample_.yaw;
        pub_.yaw_total   = sample_.yaw_total;
        pub_.temperature = sample_.temp;
        return;
    }

    attitude_.Process(sample_, pub_);
}

/**
 * @brief 任务函数
 *
 */
void ImuManager::Task()
{
    const auto kMeasureDT = [&]() -> float
    {
        static uint32_t last_cycle = 0;
        const uint32_t now_cycle = k_cycle_get_32();
        if (last_cycle == 0) {
            last_cycle = now_cycle;
            return sample_.dt;
        }
        const uint64_t delta_ns = k_cyc_to_ns_floor64(now_cycle - last_cycle);
        last_cycle = now_cycle;
        return (delta_ns == 0) ? sample_.dt : static_cast<float>(delta_ns) * 1.0e-9f;
    };

    /* Timer 的周期单位是循环次数（本循环 1ms 一圈），1000 次 = 1s 打一行；
     * 原来写 1 是每 1ms 一行（约 1000 行/s），会把 console 刷满、冲掉其他打印。 */
    log_timer_.SetPeriod(1000);

    /* 周期用「绝对截止时刻」维持（与下板 gimbal 线程同一套做法）：
     * 之前是 k_msleep(1) 的相对睡法，系统 tick 是 0.5ms，
     *   used = k_uptime_ticks() - tick_start 会被量化，
     * 循环体不足半拍时 used 读作 0 → 多睡整整一拍，1ms 目标实测跑成 ~1.5ms
     * （发布率只有 ~650Hz，1kHz 的 IMU 数据被抽稀）。
     * 睡到绝对 deadline 后，tick 量化误差与循环体耗时一起被吸收：
     * 只要循环体 < period，平均周期就恒等于 period。
     * 循环体超时（> period）时不硬追，重排基准，不往后欠债。 */
    const int64_t period = k_ms_to_ticks_ceil64(1);
    int64_t deadline = k_uptime_ticks() + period;

    for (;;)
    {
#if defined(CONFIG_WDT_SUPERVISOR)
        wdg_health::Mark(wdg_health::Service::Imu);
#endif
        log_timer_.Update();

        if (source_ != nullptr && source_->Read(sample_))
        {
            // 用真实经过时间修正当前样本积分周期，避免调度抖动直接传进 EKF。
            sample_.dt = kMeasureDT();

#ifdef CONFIG_DUST_MOD_DEV_IMU_HEATER
            heater_.Update(sample_.temp);
#endif

            PublishSample();

            pub_.version++;                 // 发布计数：消费者据此判断源是否在发布
            zbus_chan_pub(&pub_imu_to, &pub_, K_MSEC(1));
        }
#ifdef CONFIG_DUST_MOD_DEV_IMU_HEATER
        else
        {
            /* IMU 读取失败：拿不到可信温度，立刻切断加热，
             * 否则 PWM 会停在上一次的占空比（可能是升温阶段的满功率）一直加热。 */
            heater_.Stop();
        }
#endif

        log_timer_.Clock([&]()  {
            // printk("%f,%f,%f\n", (double)pub_.roll, (double)pub_.pitch, (double)pub_.yaw);
            // printk("%f, %f\n", (double)sample_.temp, (double)heater_.GetDuty());
        });

        // 睡到绝对截止时刻；本拍已经过点就直接重排基准
        if (k_uptime_ticks() < deadline) {
            k_sleep(K_TIMEOUT_ABS_TICKS(deadline));
        } else {
            deadline = k_uptime_ticks();
        }
        deadline += period;
    }
}

/**
 * @brief 根据注册表选择底层 IMU 数据源
 *
 * 遍历 .imu linker section 中的 ImuEntry，要求恰好注册一个设备。
 * 0 个报错，2+ 报错，仅 1 个时调用 Init()。
 *
 * @return 是否成功选定并初始化数据源
 */
bool ImuManager::InitSource()
{
    ptrdiff_t count = __imu_end - __imu_start;

    if (count == 0) {
        LOG_ERR("no imu registered");
        return false;
    }

    if (count > 1) {
        LOG_ERR("multiple imu registered (%td), only one allowed", count);
        return false;
    }

    const ImuEntry *e = __imu_start;
    LOG_INF("init %s ...", e->name);

    source_ = e->source;
    return source_->Init();
}

/**
 * @brief 等待加热器达到目标温度
 *
 * 循环采集温度样本，直到加热器稳定在目标温度附近。
 * 未启用 CONFIG_DUST_MOD_DEV_IMU_HEATER 时（无加热器的数据源，如 HiPNUC）
 * 该函数直接返回 true。
 *
 * @return true  温度稳定
 * @return false 数据源失效
 */
bool ImuManager::Preheat()
{
#ifdef CONFIG_DUST_MOD_DEV_IMU_HEATER
    constexpr uint32_t kWaitUs    = 1000;               // 采样间隔 (µs)
    constexpr uint32_t kTimeoutMs = 10000;              // 超时时间 (ms)

    const uint32_t Start_ms = k_uptime_get();

    // 用真实经过时间算采样间隔，避免 IMU 输出率/日志开销导致 dt 虚高
    const auto kMeasureDT = [&]() -> float
    {
        static uint32_t last_cycle = 0;
        const uint32_t now_cycle = k_cycle_get_32();
        if (last_cycle == 0) {
            last_cycle = now_cycle;
            return 0.001f;
        }
        const uint64_t delta_ns = k_cyc_to_ns_floor64(now_cycle - last_cycle);
        last_cycle = now_cycle;
        return (delta_ns == 0) ? 0.001f : static_cast<float>(delta_ns) * 1.0e-9f;
    };

    log_timer_.SetPeriod(10);

    while (true)
    {
        log_timer_.Update();

        if (source_ == nullptr || !source_->Read(sample_)) {
            k_busy_wait(kWaitUs);
            continue;
        }

        if (heater_.Preheat(sample_.temp, kMeasureDT())) {
            return true;
        }

        // 预热打印，用于判断是否被稳定判据卡住
        log_timer_.Clock([&](){
            LOG_INF("%f,%f", (double)sample_.temp, (double)heater_.GetDuty());
        });

        if (k_uptime_get() - Start_ms >= kTimeoutMs) {
            LOG_ERR("preheat timeout");
            return false;
        }

        k_busy_wait(kWaitUs);
    }
#else
    // 无加热器的数据源（如 HiPNUC 成品模块），直接跳过预热
    return true;
#endif
}

} // namespace imu

/**
 * @file Init_entry.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-07-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "Init_entry.hpp"
#include "System_startup.h"
#include "buzzer.hpp"
#include "zephyr/sys/printk.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(init, LOG_LEVEL_INF);

static constexpr struct
{
    InitStage    stage;
    const char*  name;
}
kInitStageMap[] {
    {InitStage::PreInit,   "PreInit"},
    {InitStage::EarlyInit, "EarlyInit"},
    {InitStage::MidInit,   "MidInit"},
    {InitStage::LateInit,  "LateInit"},
    {InitStage::AppInit,   "AppInit"},
};

static constexpr struct
{
    ThreadStage  stage;
    const char*  name;
}
kThreadStageMap[] {
    {ThreadStage::PreThread,   "PreThread"},
    {ThreadStage::EarlyThread, "EarlyThread"},
    {ThreadStage::MidThread,   "MidThread"},
    {ThreadStage::LateThread,  "LateThread"},
    {ThreadStage::AppThread,   "AppThread"},
};

static const char* StageName(InitStage stage)
{
    for (const auto& m : kInitStageMap)
    {
        if (m.stage == stage) {
            return m.name;
        }
    }
    return "unknown";
}

static const char* StageName(ThreadStage stage)
{
    for (const auto& m : kThreadStageMap)
    {
        if (m.stage == stage) {
            return m.name;
        }
    }
    return "unknown";
}

static constexpr struct FailAction {
    InitLevel  level;
    bool       halt;
}
kFailActions[] {
    {InitLevel::High, true},
    {InitLevel::Mid,  false},
    {InitLevel::Low,  false},
};

static void HandleInitFail(const InitEntry& entry)
{
    bool halt = false;

    for (const auto& a : kFailActions)
    {
        if (a.level == entry.level) {
            halt = a.halt;
            break;
        }
    }

    if (halt) {
        LOG_ERR("init %s failed, system halted", entry.name);
    } else {
        LOG_WRN("init %s failed", entry.name);
    }

    if (halt) {
        EXEC_BUZZER_ERR(nullptr);
        while (1) {}
    }
}

static void RunStage(InitStage stage)
{
    extern const InitEntry __user_init_start[], __user_init_end[];

    LOG_ERR("==== init stage: %s ====", StageName(stage));

    for (const InitEntry* e = __user_init_start; e < __user_init_end; ++e)
    {
        if (e->stage != static_cast<InitStage>(stage)) {
            continue;
        }
        LOG_ERR("%s", e->name);
        if (!e->func()) {
            HandleInitFail(*e);
        }
    }
    printk("\n");
}

static void RunStage(ThreadStage stage)
{
    extern const InitEntry __user_thread_start[], __user_thread_end[];

    LOG_ERR("==== thread stage: %s ====", StageName(stage));

    for (const InitEntry* e = __user_thread_start; e < __user_thread_end; ++e)
    {
        if (static_cast<ThreadStage>(e->stage) != stage) {
            continue;
        }
        LOG_ERR("%s", e->name);
        if (!e->func()) {
            HandleInitFail(*e);
        }
    }
    printk("\n");
}

/**
 * @brief 系统启动入口
 *
 * 顺序：PreInit/PreThread → EarlyInit/EarlyThread → MidInit/MidThread
 *       → LateInit/LateThread → AppInit/AppThread。
 * 每对先执行 Init（初始化对象），再启动对应的 Thread（线程循环）。
 * 由 main() 调用。
 */
void System_Startup(void)
{
    constexpr uint16_t kNormalStageDelayMs = 500;

    RunStage(InitStage::PreInit);
    k_msleep(kNormalStageDelayMs);
    RunStage(ThreadStage::PreThread);
    k_msleep(kNormalStageDelayMs);
    EXEC_BUZZER_SHORT();

    RunStage(InitStage::EarlyInit);
    k_msleep(kNormalStageDelayMs);
    RunStage(ThreadStage::EarlyThread);
    k_msleep(kNormalStageDelayMs);
    EXEC_BUZZER_SHORT();

    RunStage(InitStage::MidInit);
    k_msleep(kNormalStageDelayMs);
    RunStage(ThreadStage::MidThread);
    k_msleep(kNormalStageDelayMs);
    EXEC_BUZZER_SHORT();

    RunStage(InitStage::LateInit);
    k_msleep(kNormalStageDelayMs);
    RunStage(ThreadStage::LateThread);
    k_msleep(kNormalStageDelayMs);
    EXEC_BUZZER_SHORT();

    RunStage(InitStage::AppInit);
    k_msleep(kNormalStageDelayMs);
    RunStage(ThreadStage::AppThread);
}

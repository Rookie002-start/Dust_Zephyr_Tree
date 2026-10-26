/**
 * @file Init_entry.hpp
 * @author qingyu
 * @brief 初始化项描述：等级、优先级、函数指针
 * @version 0.1
 * @date 2026-07-20
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include <cstdint>

/**
 * @brief 初始化阶段
 *
 */
enum class InitStage : uint8_t
{
    PreInit   = 0,
    EarlyInit = 1,
    MidInit   = 2,
    LateInit  = 3,
    AppInit   = 4,
};

/**
 * @brief 线程阶段
 *
 */
enum class ThreadStage : uint8_t
{
    PreThread   = 0,
    EarlyThread = 1,
    MidThread   = 2,
    LateThread  = 3,
    AppThread   = 4,
};

/**
 * @brief 初始化等级
 */
enum class InitLevel : uint8_t
{
    High = 0,
    Mid  = 1,
    Low  = 2,
};

/**
 * @brief 初始化函数原型
 * @return true=成功, false=失败
 */
using InitFunc = bool (*)();

/**
 * @brief 初始化项描述
 *
 * 每个组件声明一个 InitEntry，链接后形成初始化表，启动器遍历执行。
 */
struct InitEntry
{
    InitFunc     func;      // 初始化函数
    InitStage    stage;     // 所属阶段
    InitLevel    level;     // 高级/中级/低级
    const char*  name;      // 组件名，调试用
};

/**
 * @brief 注册初始化项
 *
 * 每个组件用此宏声明一个 InitEntry，
 * 通过链接段收集形成初始化表，启动器遍历执行。
 *
 * @param fn       初始化函数（符合 InitFunc 签名）
 * @param stage_   InitStage 枚举值
 * @param level_   InitLevel 枚举值
 * @param name_    组件标识，打印用
 */
#define REGISTER_INIT(fn, stage_, level_, name_)                                \
    static const InitEntry kInitEntry_##fn                                      \
    __attribute__((used, __section__(".user_init"))) = {                        \
        fn, InitStage::stage_, InitLevel::level_, name_                         \
}

/**
 * @brief 注册线程项
 *
 * 每个线程用此宏声明一个 InitEntry（存放于 .user_thread 段），
 * 启动器遍历执行。
 *
 * @param fn       线程函数（符合 InitFunc 签名）
 * @param stage_   ThreadStage 枚举值
 * @param level_   InitLevel 枚举值
 * @param name_    组件标识，打印用
 */
#define REGISTER_THREAD(fn, stage_, level_, name_)                                  \
    static const InitEntry kThreadEntry_##fn                                        \
    __attribute__((used, __section__(".user_thread"))) = {                          \
        fn, static_cast<InitStage>(ThreadStage::stage_), InitLevel::level_, name_   \
}

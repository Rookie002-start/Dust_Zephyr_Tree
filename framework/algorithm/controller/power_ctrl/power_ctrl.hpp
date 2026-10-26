/**
 * @file power_ctrl.hpp
 * @author qingyu
 * @brief 电机功率控制器 — 模型预测 / RLS 在线辨识 / 隶属度分配 / 力矩限幅
 * @version 0.3
 * @date 2026-10-15
 */

#pragma once

#pragma message "Compiling Algorithm/Controller/PowerCtrl"

#include <stdint.h>
#include <math.h>
#include <cmath>
#include "rls.hpp"

namespace alg::power_ctrl {

/// 二次方程一次项 (A) 绝对值小于该值时按退化的一次方程处理
constexpr float kQuadraticATiny = 1.0e-9f;

/// 一次方程斜率 (B) 绝对值小于该值：方程与 τ 无关，解不存在
constexpr float kQuadraticBTiny = 1.0e-12f;

/// 限幅比例系数的分母下限：目标/实测转矩小于它就不参与"按比例缩放"
/// （≈0.07 A，低于此认为这一路没在出力，没必要削）
constexpr float kTorqueFloor = 0.02f;

/// 有限性判据：NaN / Inf 一律视为无效数据
inline bool IsFinite(float v) { return std::isfinite(v); }

/**
 * @brief  单电机状态
 */
struct MotorState {
    float torque        = 0.0f;         // τ = current × torqueK（实测口径）
    float omega         = 0.0f;         // ω = rpm / 9.55 (rad/s)
    float torque2       = 0.0f;         // τ²
    float omega2        = 0.0f;         // ω²
    float torqueOmega   = 0.0f;         // τ·ω
    float powerPred     = 0.0f;         // 实测口径预测 P_in = K1·τ² + K2·ω² + η·τ·ω + K3
    float powerCmd      = 0.0f;         // 指令口径预测（代入 PID 目标电流，限幅判决用）
    float pidErr        = 0.0f;         // PID 误差（隶属度计算用）
    float powerLimit    = 0.0f;         // 该电机分配的功率预算
    float targetCurrent = 0.0f;         // PID 目标电流（串级输入）
    float currentOut    = 0.0f;         // 限幅后的输出电流
};

/**
 * @brief  单组电机功率控制器（如底盘 4 个电机）
 *
 * @tparam kMotorCount  电机数量（编译期分配）
 *
 * @par 管线（每控制周期调用一次）：
 *       @n 1. SetMotorData(i, torque, omega, pidErr)       — 喂入实测转矩/角速度
 *       @n 2. SetTarget(i, targetCurrent)                  — 喂入 PID 目标电流
 *       @n 3. Predict()                                    — 功率预测 + RLS
 *       @n 4. Allocate(totalBudget)                        — 隶属度分配 + 限幅
 *       @n 5. GetLimitedCurrent(i)                         — 读取结果
 *
 * @par 两套功率口径（关键）：
 *       · powerPred：把**实测**转矩代进模型 → 反映"现在实际用了多少"，
 *         只用于 RLS 拟合与上报；
 *       · powerCmd ：把 **PID 目标电流**代进同一组系数 → 反映"这一拍要下多少"，
 *         限幅判决必须用它。用实测口径判决要等电流真的上去才知道超了，
 *         阶跃 / 堵转时慢一拍，限不住。
 *       由于 powerCmd 在 Predict() 里算，调用顺序必须是 SetTarget() → Predict()。
 *
 * @par RLS：
 *       在线辨识 K1（铜损系数）和 K2（铁损系数）。
 *       K3 与 η·τ·ω 为已知项，从拟合目标中减掉，不参与辨识。
 *       辨识结果被钳在 [k1Min, k1Max] × [k2Min, k2Max] 内：参数一旦为负，
 *       预测功率会变成负值，限幅会以为"没超预算"而对所有电机直通。
 */
template <uint8_t kMotorCount>
class PowerCtrl final
{
public:
    /**
     * @brief 配置参数
     *
     * @note k1Init / k2Init 必须按**同一套单位**标定，且 K1 不能留默认值：
     *       K1·τ² 是铜损项，物理上 K1 ≈ R / Kt²（R = 相电阻 Ω，Kt = 转矩常数
     *       N·m/A）。以 M3508（Kt = 0.3）为例 K1 应在 1~2 量级；若沿用库默认的
     *       1.453e-7，铜损项实际等于 0，模型只剩 η·τ·ω + K3 —— 低速 / 堵转
     *       （ω→0 且电流很大）时预测功率仍远低于预算，限幅会完全不起作用。
     */
    struct Config {
        float   torqueK    = 4.577e-5f;         // 电流 → 转矩（M3508 常数）
        float   k1Init     = 1.453e-7f;         // RLS 初始 K1（铜损系数），见上方 note
        float   k2Init     = 1.453e-7f;         // RLS 初始 K2（铁损系数）
        float   k3         = 3.0f;              // 固定常数损耗
        float   etaInit    = 1.0f;              // τ·ω 系数
        float   errUpper   = 50.0f;             // 隶属度：全需求阈值
        float   errLower   = 0.01f;             // 隶属度：全功率阈值
        float   rlsLambda  = 0.99999f;          // RLS 遗忘因子
        bool    rlsEnable  = false;             // false = 使用固定 K1/K2
        bool    tauOmegaEnable = true;          // 是否包含 τ·ω 项
        /// 机械功率项是否取绝对值（默认 false = 保留符号，允许回馈）。
        ///
        /// 实车实测：刹车时底盘支路电流为**负**（−0.79 A @24.9 V ≈ 回馈 19 W），
        /// 也就是能量回到母线/电容、由裁判系统的缓冲吸收 —— 和 RM 的缓冲模型一致，
        /// 所以默认按有符号处理（＝HKUST 那套 τ·ω 系数为 1 的形式）。
        /// 仅当确认电调把制动能量全烧在桥里（母线电流始终为正）时才设 true。
        bool    mechAbs = false;
        /// 回馈（制动倒灌）功率上限 W，<= 0 表示不限制。
        float   regenMaxW = 0.0f;
        
        float   pInit      = 1.0e-5f;           // RLS 协方差初值
        float   k1Min      = 0.0f;              // 辨识结果下界（负值一律不允许）
        float   k1Max      = 1.0e4f;            // 辨识结果上界
        float   k2Min      = 0.0f;
        float   k2Max      = 1.0e4f;
        float   minBudgetShare = 0.0f;          // 每台电机的最低预算份额（0~1/N）
    };

    PowerCtrl() : PowerCtrl(Config{}) {}

    /**
     * @brief 构造功率控制器，初始化 RLS
     * @param cfg 配置参数
     */
    explicit PowerCtrl(const Config& cfg)
        : cfg_(cfg)
        , k1_(ClampParam(cfg.k1Init, cfg.k1Min, cfg.k1Max))
        , k2_(ClampParam(cfg.k2Init, cfg.k2Min, cfg.k2Max))
        , eta_(cfg.etaInit)
        , rls_(cfg.rlsLambda, cfg.pInit)
    {
        rlsInited_ = cfg.rlsEnable;

        if (rlsInited_) {
            float w[2] = { k1_, k2_ };
            rls_.SetWeights(w);
        }
    }

    /**
     * @brief 重配置功率控制器，清空状态
     * @param cfg 配置参数
     */
    void Init(const Config& cfg)
    {
        cfg_ = cfg;
        k1_  = ClampParam(cfg.k1Init, cfg.k1Min, cfg.k1Max);
        k2_  = ClampParam(cfg.k2Init, cfg.k2Min, cfg.k2Max);
        eta_ = cfg.etaInit;
        rlsInited_ = cfg.rlsEnable;

        // 协方差一并归零重来：上次运行留下的 P 对新的权重初值没有意义
        rls_.Reset(cfg.pInit);
        if (rlsInited_) {
            float w[2] = { k1_, k2_ };
            rls_.SetWeights(w);
        }

        for (auto& m : motors_) m = MotorState{};
        for (auto& m : membership_) m = 0.0f;

        measuredPower_ = 0.0f;
        measuredFresh_ = false;
    }

    /**
     * @brief 喂入单电机原始数据
     * @param idx     电机索引 [0, kMotorCount)
     * @param torque  实测转矩 (N·m)
     * @param omega   角速度 (rad/s)
     * @param pidErr  PID 误差（隶属度计算用）
     */
    void SetMotorData(uint8_t idx, float torque, float omega, float pidErr)
    {
        if (idx >= kMotorCount) return;

        auto& m = motors_[idx];
        m.torque   = torque;
        m.omega    = omega;
        m.torque2  = m.torque * m.torque;
        m.omega2   = m.omega  * m.omega;
        m.torqueOmega = m.torque * m.omega;
        m.pidErr   = pidErr;
    }

    /**
     * @brief 设置 PID 目标电流（串级输入）
     * @param idx      电机索引
     * @param current  目标电流
     */
    void SetTarget(uint8_t idx, float current)
    {
        if (idx >= kMotorCount) return;
        motors_[idx].targetCurrent = current;
    }

    /**
     * @brief 更新实测总功率
     * @param power 母线实测功率 (W)
     * @param fresh true = 该值是功率计**新帧**带来的；false = 只是缓存刷新
     *
     * @note 控制周期通常快于功率计帧率（底盘 1ms，功率计常见 100~1000Hz）。
     *       把同一条测量值重复喂进 RLS 会让拟合被系统性带偏（x 每拍都在变、
     *       y 却是旧的），所以只有 fresh = true 的那一拍才真正迭代。
     */
    void SetMeasuredPower(float power, bool fresh = true)
    {
        measuredPower_ = power;
        if (fresh) measuredFresh_ = true;
    }

    /// @brief 在线辨识开关：关掉后权重保留，只停用实测功率更新
    /// @note 开关状态发生变化时会复位协方差，避免用膨胀过的 P 继续迭代；
    ///       调用方应把"功率计是否可用"做成带滞回的判断，别每拍翻转。
    void EnableRls(bool on)
    {
        if (on == rlsInited_) return;

        rlsInited_ = on;
        float w[2] = { k1_, k2_ };
        rls_.Reset(cfg_.pInit);
        rls_.SetWeights(w);
    }

    /**
     * @brief 清空辨识状态，保留当前 K1/K2 作为新初值
     * @note 停机 / 复位 / 工况被打断时调用，让 RLS 从干净状态重新收敛
     */
    void ResetIdent()
    {
        float w[2] = { k1_, k2_ };
        rls_.Reset(cfg_.pInit);
        rls_.SetWeights(w);
        measuredFresh_ = false;
    }

    /**
     * @brief 功率预测 + RLS 在线更新
     *
     * 同一组系数算两遍：实测口径 powerPred（RLS 拟合 / 上报）与
     * 指令口径 powerCmd（把目标电流代进去，限幅判决用）。
     *
     * 拟合目标 y = P_meas - ΣK3 - η·Στω，更新 K1、K2。
     * 只有 SetMeasuredPower(..., true) 标记过的新样本才迭代 RLS。
     */
    void Predict()
    {
        float sumTorque2  = 0.0f;
        float sumOmega2   = 0.0f;
        float sumMech     = 0.0f;       // Σ(η·τ·ω)，已按 mechNoRegen 钳过

        for (uint8_t i = 0; i < kMotorCount; i++)
        {
            auto& m = motors_[i];
            const float tauCmd = m.targetCurrent * cfg_.torqueK;

            // 机械功率项 η·τ·ω，取绝对值（mechAbs）。实车实测：4 轮 8A 猛刹车时
            // 母线功率冲到 70 W（功率计从不出现负值），按"回馈"算只有 28 W ——
            // 差额 40 W 会让限幅对制动完全放行。取绝对值后制动也按耗电计入。
            float mechMeas = cfg_.tauOmegaEnable ? (eta_ * m.torqueOmega) : 0.0f;
            float mechCmd  = cfg_.tauOmegaEnable ? (eta_ * tauCmd * m.omega) : 0.0f;
            if (cfg_.mechAbs) {
                mechMeas = fabsf(mechMeas);
                mechCmd  = fabsf(mechCmd);
            }

            // 实测口径：P = K1·τ² + K2·ω² + η·τ·ω(可选, 钳过) + K3
            m.powerPred = k1_ * m.torque2 + k2_ * m.omega2 + cfg_.k3 + mechMeas;

            // 指令口径：同样的系数，转矩换成"这一拍打算下发的"目标电流。
            // 限幅必须按它判决 —— 按实测口径判决等于"等超了才知道超了"。
            m.powerCmd = k1_ * tauCmd * tauCmd + k2_ * m.omega2 + cfg_.k3 + mechCmd;

            sumTorque2 += m.torque2;
            sumOmega2  += m.omega2;
            sumMech    += mechMeas;
        }

        // RLS：已知项（ΣK3、Ση·τω，都是钳位后的）先减掉
        if (rlsInited_ && measuredFresh_)
        {
            measuredFresh_ = false;

            float y = measuredPower_ - static_cast<float>(kMotorCount) * cfg_.k3 - sumMech;

            const float x[2] = { sumTorque2, sumOmega2 };
            rls_.Update(x, y);

            // 钳位后写回 RLS：让滤波器内部状态与对外发布的权重保持一致。
            // 参数跑成负值时预测功率为负 → 判决以为没超预算 → 限幅静默失效。
            k1_ = ClampParam(rls_.GetWeights()[0], cfg_.k1Min, cfg_.k1Max, k1_);
            k2_ = ClampParam(rls_.GetWeights()[1], cfg_.k2Min, cfg_.k2Max, k2_);

            float w[2] = { k1_, k2_ };
            rls_.SetWeights(w);
        }
    }

    /**
     * @brief 功率分配：隶属度计算 + 力矩限幅
     *
     * 判决口径是**指令功率**（powerCmd）：要等电流真的上去才发现超预算，
     * 过载那几拍就已经放出去了，所以不能用实测口径。
     *
     * 总指令预测未超预算时直通 PID 目标电流；
     * 超限时按每台的预算份额解二次方程求受限转矩。
     *
     * @param totalBudget 该组总功率预算 (W)
     * @note 调用前需先 SetTarget() → Predict()
     */
    void Allocate(float totalBudget)
    {
        float sumAbsErr   = 0.0f;
        float sumPowerPos = 0.0f;

        for (uint8_t i = 0; i < kMotorCount; i++) {
            sumAbsErr += fabsf(motors_[i].pidErr);

            // 只统计"耗电"的部分：回馈制动（τ·ω < 0）是在往回灌能量，
            // 用 |P| 参与分配会让正在发电的电机反而分到更多预算。
            const float p = motors_[i].powerCmd;
            if (p > 0.0f) sumPowerPos += p;
        }

        // 权重 K（误差水平插值）
        float k;
        if (sumAbsErr >= cfg_.errUpper) {
            k = 1.0f;
        } else if (sumAbsErr <= cfg_.errLower) {
            k = 0.0f;
        } else {
            float range = cfg_.errUpper - cfg_.errLower;
            k = (sumAbsErr - cfg_.errLower) / (range > 0.0f ? range : 1.0f);
        }

        // 保底份额：不允许任何一台拿到 0 预算。份额为 0 的电机在超限时解不出
        // 与自己同号的根，只能被迫退出驱动，表现成"直线跑着突然丢一个轮子"。
        float shareFloor = cfg_.minBudgetShare;
        if (shareFloor < 0.0f) shareFloor = 0.0f;

        const float shareCeil = 1.0f / static_cast<float>(kMotorCount);
        if (shareFloor >= shareCeil) shareFloor = 0.0f;             // 配不出来就别配
        const float shareFree = 1.0f - shareFloor * static_cast<float>(kMotorCount);

        // 原始份额 → 归一化 → 加保底，保证 Σmembership = 1（即 ΣpowerLimit = 预算）
        float share[kMotorCount] {};
        float sumShare = 0.0f;

        for (uint8_t i = 0; i < kMotorCount; i++)
        {
            const float ratioErr = (sumAbsErr > 0.0f)
                                 ? fabsf(motors_[i].pidErr) / sumAbsErr : 0.0f;
            const float p = motors_[i].powerCmd;
            const float ratioPower = (sumPowerPos > 0.0f)
                                   ? ((p > 0.0f) ? p / sumPowerPos : 0.0f) : shareCeil;

            float s = k * ratioErr + (1.0f - k) * ratioPower;
            if (!IsFinite(s) || s < 0.0f) s = 0.0f;
            if (s > 1.0f) s = 1.0f;

            share[i] = s;
            sumShare += s;
        }

        for (uint8_t i = 0; i < kMotorCount; i++)
        {
            const float norm = (sumShare > 0.0f) ? (share[i] / sumShare) : shareCeil;
            membership_[i] = shareFloor + shareFree * norm;
            motors_[i].powerLimit = membership_[i] * totalBudget;
        }

        // 受限力矩求解：两个口径都要看。
        // 指令口径负责"预判"（阶跃给上去时立刻出手，不用等实测电流爬到），
        // 实测口径负责"纠偏"（电调电流环有增益误差：实测 0.62A 对应命令 0.46A，
        // 只看命令口径会低估 10%+ 的功率）。任一超预算就进入限幅。
        if (GetTotalPowerCmd() <= totalBudget && GetTotalPower() <= totalBudget) {
            // 未超限：透传 PID 目标电流
            for (uint8_t i = 0; i < kMotorCount; i++) {
                motors_[i].currentOut = motors_[i].targetCurrent;
            }
        }
        else
        {
        // 超限：逐电机解方程，限制 PID 目标
        for (uint8_t i = 0; i < kMotorCount; i++)
        {
            auto& m = motors_[i];

            const float a = k1_;                                        // K1
            float b = cfg_.tauOmegaEnable ? (eta_ * m.omega) : 0.0f;    // η·ω

            // 制动象限也要用"同一套模型"：mechAbs 打开时制动按 |τ·ω| 计耗电，
            // 功率 = K1·τ² + K2·ω² + K3 - η·ω·τ（τ 与 ω 反号时 |τω| = -τω），
            // 所以一次项系数要取负。若沿用 +η·ω，二次方程会认为"刹车越大越省功率"，
            // 解出来的受限转矩比目标还大，再被"只削不加"退回目标 → 制动完全不限幅。
            if (cfg_.mechAbs && (m.targetCurrent * m.omega) < 0.0f) {
                b = -b;
            }
            const float c = k2_ * m.omega2 + cfg_.k3 - m.powerLimit;    // K2·ω² + K3 - P_limit

            const bool positive = (m.targetCurrent >= 0.0f);
            const float limitedTorque = SolveTorque(a, b, c, positive);

            // 限幅做成"按比例缩放 PID 输出"，比例取两个口径里更狠的那个：
            //   ① 指令口径：允许转矩 / 目标转矩 —— 阶跃时预判用
            //   ② 实测口径：允许转矩 / 实测转矩 —— 电调实际电流比命令大时纠偏用
            // 用比值而不是直接把解出来的电流当命令，好处是电调电流环的增益误差
            // 自动抵消（两边都在同一个"实际"域里比）。
            float scale = 1.0f;

            const float targetTorque = m.targetCurrent * cfg_.torqueK;
            if (targetTorque * limitedTorque <= 0.0f) {
                // 无同号可行解（例如份额为 0、ω>0 时两根本全负）→ 这一路不能出力。
                // 必须归零：按幅值缩放会把反号的根变成"大幅削减的正向电流"，
                // 反而给出一个超出份额的正输出。
                scale = 0.0f;
            } else if (fabsf(targetTorque) > kTorqueFloor) {
                const float s = fabsf(limitedTorque) / fabsf(targetTorque);
                if (s < scale) scale = s;
            }
            // 实测口径：只在实测与目标同号时才用它纠偏（异号说明这一路正在换向，
            // 此刻没在往目标方向耗电，不需要削）
            if ((m.torque * m.targetCurrent) > 0.0f && fabsf(m.torque) > kTorqueFloor) {
                const float s = fabsf(limitedTorque) / fabsf(m.torque);
                if (s < scale) scale = s;
            }

            if (!IsFinite(scale) || scale < 0.0f) scale = 0.0f;
            if (scale > 1.0f) scale = 1.0f;             // 只削不加

            // 按比例缩放：符号天然跟 PID 目标一致（不会出现反向驱动），
            // 目标为 0 时输出 0
            const float out = m.targetCurrent * scale;
            m.currentOut = IsFinite(out) ? out : 0.0f;
        }
        }   // else（超限路径）

        LimitRegen();
    }

    /**
     * @brief 回馈（制动倒灌）功率上限
     *
     * 只统计制动象限（τ·ω < 0）往母线倒灌的那部分，超过 regenMaxW 就把**制动中的**
     * 电机按同一比例削小；驱动中的电机不动（回馈上限不该影响加速）。
     *
     * 判定取"命令口径"和"实测口径"里更狠的那个：电调电流环有增益误差（实测可能是
     * 命令的 1.3~1.4 倍），只看命令口径会低估真实倒灌功率、保护不到位。
     */
    void LimitRegen()
    {
        if (cfg_.regenMaxW <= 0.0f) return;

        float mech[kMotorCount] {};
        float regen = 0.0f;

        for (uint8_t i = 0; i < kMotorCount; i++) {
            const float mechCmd  = motors_[i].currentOut * cfg_.torqueK * motors_[i].omega;
            const float mechMeas = motors_[i].torque * motors_[i].omega;
            mech[i] = fminf(mechCmd, mechMeas);          // 更负的那个 = 倒灌更多
            if (mech[i] < 0.0f) regen += -mech[i];
        }

        if (regen <= cfg_.regenMaxW) return;

        const float scale = cfg_.regenMaxW / regen;
        for (uint8_t i = 0; i < kMotorCount; i++) {
            if (mech[i] < 0.0f) {
                motors_[i].currentOut *= scale;
            }
        }
    }

    /**
     * @brief 取限幅后的输出电流
     * @param idx  电机索引
     * @return 限幅后的电流值，越界返回 0
     */
    float GetLimitedCurrent(uint8_t idx) const
    {
        return (idx < kMotorCount) ? motors_[idx].currentOut : 0.0f;
    }

    /**
     * @brief 所有电机总预测功率（实测口径：现在实际用了多少）
     * @return 总功率 (W)
     */
    float GetTotalPower() const
    {
        float sum = 0.0f;
        for (uint8_t i = 0; i < kMotorCount; i++) {
            sum += motors_[i].powerPred;
        }
        return sum;
    }

    /**
     * @brief 所有电机总指令功率（判决口径：这一拍要下多少）
     * @return 总功率 (W)，与 Allocate() 的判决量一致
     */
    float GetTotalPowerCmd() const
    {
        float sum = 0.0f;
        for (uint8_t i = 0; i < kMotorCount; i++) {
            sum += motors_[i].powerCmd;
        }
        return sum;
    }

    /// @brief 单台电机的功率预算份额（0~1，Σ = 1）
    float GetMembership(uint8_t idx) const
    {
        return (idx < kMotorCount) ? membership_[idx] : 0.0f;
    }

    /// @brief 单台电机分到的功率预算 (W)
    float GetPowerLimit(uint8_t idx) const
    {
        return (idx < kMotorCount) ? motors_[idx].powerLimit : 0.0f;
    }

    /// @brief 单台电机的 PID 目标电流（限幅前），用于观测"削了多少"
    float GetTargetCurrent(uint8_t idx) const
    {
        return (idx < kMotorCount) ? motors_[idx].targetCurrent : 0.0f;
    }

    float GetK1() const { return k1_; }
    float GetK2() const { return k2_; }
    float GetEta() const { return eta_; }

private:
    Config cfg_;

    float k1_ = 0.0f;                           // 铜损系数
    float k2_ = 0.0f;                           // 铁损系数
    float eta_ = 1.0f;                          // τ·ω 系数（固定）

    MotorState motors_[kMotorCount]{};          // 各电机状态（转矩/转速/功率等）
    float      membership_[kMotorCount]{};      // 各电机隶属度（功率分配系数 0~1）

    alg::rls::RLS<2> rls_;                      // RLS 在线辨识器
    bool rlsInited_ = false;

    float measuredPower_ = 0.0f;                // 功率计实测总功率
    bool  measuredFresh_ = false;               // 是否收到新的实测功率样本

    /**
     * @brief 解 A·τ² + B·τ + C = 0，返回与目标同侧的受限转矩
     *
     * A = K1 > 0 时可行域是两根之间，取正目标的上根 / 负目标的下根。
     * A ≈ 0（K1 未标定或为 0）时退化成一次方程 B·τ + C = 0，直接解边界；
     * 此时模型对转矩没有二次约束能力，返回值仍然要经过调用方的同号 / 只削不加
     * 两道钳位，才不会给出反向电流。
     */
    static float SolveTorque(float a, float b, float c, bool positive)
    {
        if (a <= kQuadraticATiny) {
            return (fabsf(b) > kQuadraticBTiny) ? (-c / b) : 0.0f;
        }

        float delta = b * b - 4.0f * a * c;
        if (delta < 0.0f) delta = 0.0f;

        // 数值稳定写法：直接用 (-b ± √Δ) 求根，在 A 很小、B 与 √Δ 接近时
        // 会发生灾难性相消（差值被舍入噪声吃掉），换成 q/a 与 c/q 两个根
        const float sqrtDelta = sqrtf(delta);
        const float q = -0.5f * (b + copysignf(sqrtDelta, b));

        if (q == 0.0f) {
            return -b / (2.0f * a);         // 重根
        }

        const float r1 = q / a;
        const float r2 = c / q;

        return positive ? fmaxf(r1, r2) : fminf(r1, r2);
    }

    /**
     * @brief 参数钳位：NaN 时退回 fallback，越界时钳到 [lo, hi]
     */
    static float ClampParam(float v, float lo, float hi, float fallback = 0.0f)
    {
        if (!IsFinite(v)) return fallback;
        if (hi < lo) { const float t = lo; lo = hi; hi = t; }
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
};

} // namespace alg::power_ctrl

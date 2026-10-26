# PowerCtrl — 电机功率控制器

功率模型预测 → RLS 在线参数辨识 → 隶属度分配 → 受限力矩求解

## 概述

PowerCtrl 管理一组同类型电机，基于功率模型预测各电机功耗，通过 RLS 在线辨识电机损耗参数，在总功率预算内做最优化分配并求解受限电流。主要用于 RoboMaster 等需要严格功率限制的场景。

## 依赖

- `alg::rls::RLS`（[identify/rls](../identify/rls/README.md)）
- `math.h`（sqrtf）

## 功率模型

```
P_in = K1 · τ² + K2 · ω² + η · τ · ω + K3
```

| 项 | 物理含义 | 说明 |
|----|----------|------|
| K1·τ² | 铜损 | 绕组电阻发热，与转矩平方成正比 |
| K2·ω² | 铁损 + 摩擦 | 磁滞/涡流损耗 + 轴承摩擦，与转速平方成正比 |
| η·τ·ω | 机械功率 | 实际对外做功，P_mech = τ·ω，η 为固定系数（默认 1.0） |
| K3 | 常数损耗 | 控制器待机功耗、传感器供电等固定损耗 |

**辨识目标**：RLS 以 `x = [Στ², Σω²]` 为输入、`y = P_meas - ΣK3 - η·Στω` 为期望输出，在线拟合 K1（铜损系数）和 K2（铁损系数）。`η·τ·ω` 和 K3 是已知项，必须从拟合目标里减掉，不作为辨识参数 —— 否则它们会被 K1/K2 吸收一遍、预测时再加一遍（重复计入，预测偏高）。

**K1 不能留默认值**：K1 的物理值是 `R / Kt²`（R = 相电阻 Ω，Kt = 转矩常数
N·m/A）。以 M3508（Kt = 0.3 N·m/A、R ≈ 0.1~0.2 Ω）为例，K1 应在 **1~2** 量级；
若沿用库默认的 `1.453e-7`，铜损项实际等于 0，模型只剩 `η·τ·ω + K3`，于是
**ω → 0 而电流很大（堵转、顶障碍物）时预测功率仍远低于预算，限幅会完全不起作用**。
标定前请先用 `R/Kt²` 估一个初值，再用实测数据核对。

## 两套功率口径（关键）

模型系数只有一组，但每个周期会算两遍：

| 口径 | 转矩来源 | 用途 |
|------|----------|------|
| `powerPred`（实测口径） | `SetMotorData()` 喂进来的**实测**转矩 | RLS 拟合、上报观测 |
| `powerCmd`（指令口径） | `SetTarget()` 喂进来的 **PID 目标电流** × torqueK | **限幅判决**、分配权重 |

限幅判决必须用指令口径：按实测口径判决等于"等电流真的上去才知道超了"，
过载已经放出去好几拍，阶跃 / 堵转时根本限不住。因此调用顺序是
`SetMotorData()` → `SetTarget()` → `Predict()` → `Allocate()`。

## 管线

```
每个控制周期 (1ms)

① SetMotorData(i, torque, omega, pidErr)
   ├── τ = torque（实测转矩）
   ├── ω = omega (rad/s)
   ├── τ², ω², τ·ω 缓存
   └── pidErr 缓存（隶属度计算用）

② SetTarget(i, targetCurrent)        → PID 目标电流

③ Predict()
   ├── P_pred[i] = K1·τ² + K2·ω² + η·τ·ω + K3              （实测口径）
   ├── P_cmd[i]  = K1·τc² + K2·ω² + η·τc·ω + K3, τc = 目标电流×torqueK（指令口径）
   ├── Στ², Σω², Στω
   └── 仅在 SetMeasuredPower(p, true) 标记过新样本时：
       RLS Update(α=[Στ², Σω²], β=P_meas - ΣK3 - η·Στω)
       └── 更新 K1、K2（钳在 [k1Min,k1Max]×[k2Min,k2Max] 内）

④ Allocate(budget)
   ├── 权重 K = 误差水平插值 (errLower~errUpper)
   ├── share[i] = K·|err|/Σ|err| + (1-K)·P_cmd⁺/ΣP_cmd⁺   （只统计耗电部分）
   ├── membership[i] = minShare + (1-ΣminShare)·share[i]/Σshare（Σ = 1）
   ├── powerLimit[i] = membership[i] × budget
   ├── ΣP_cmd ≤ budget **且** ΣP_pred ≤ budget → 电流直通
   └── 任一超过 → 解 K1·τ² + η·ω·τ + (K2·ω² + K3 - P_limit) = 0 得 τ_lim，
       再按比例缩放 PID 输出：scale = min(|τ_lim|/|τ_target|, |τ_lim|/|τ_meas|)
   ├── 两个比例分别对应"阶跃预判（指令口径）"和"电调电流环增益误差纠偏（实测口径）"
   ├── τ_lim 与目标反号 → scale = 0（无同号可行解，不能出力）
   └── scale 钳在 [0,1]（只削不加，且符号天然跟目标一致）
   └── LimitRegen()：制动倒灌总功率 > regenMaxW 时，把所有**制动中**的电机按比例削
 
⑤ GetLimitedCurrent(i) → 功率限制后的电流值
```

> **回馈上限（regenMaxW）**：制动象限的机械功率是往母线倒灌的（实车实测峰值
> 83.6 W、母线电压被抬高 0.61 V）。倒灌不违反"输入功率 ≤ 预算"，但会把母线电压
> 顶高 —— 电池满电或连续急刹时可能超出器件耐压。`regenMaxW > 0` 时对倒灌总功率
> 做统一缩放，驱动中的电机不受影响。判定取命令口径与实测口径里更狠的那个
> （电调电流环增益误差会让实测大于命令）。默认 0 = 不限制。

> **为什么按比例缩放而不是直接把解出来的电流当命令**：电调的电流环有增益误差
> （实车实测：命令 0.46 A → 电调实际 0.62 A，×1.35）。直接把解出来的电流当命令会
> 让实际功率超出份额（实测偏差 +40%）；按"实测转矩→允许转矩"的比值缩放，增益误差
> 在同一域里自动抵消，实际功率能正好落在份额上。

## 使用方式

### 实例化

电机数量通过模板参数 `kMotorCount` 指定（编译期常量）：

```cpp
// 2 个电机的功率控制器
alg::power_ctrl::PowerCtrl<2> ctrl;
alg::power_ctrl::PowerCtrl<2>::Config cfg{};
cfg.rlsEnable = true;      // 开启 RLS 在线辨识
ctrl.Init(cfg);

// 构造时传入 Config
alg::power_ctrl::PowerCtrl<2> ctrl({
    4.577e-5f,      // torqueK
    1.453e-7f,      // k1Init
    1.453e-7f,      // k2Init
    3.0f,           // k3
    1.0f,           // etaInit（τ·ω 系数）
    50.0f,          // errUpper
    0.01f,          // errLower
    0.99999f,       // rlsLambda
    false           // rlsEnable
});
```

### Config 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| torqueK | 4.577e-5 | 电流 → 转矩系数（N·m/A），M3508 常数 |
| k1Init | 1.453e-7 | 铜损系数 K1 初始值 |
| k2Init | 1.453e-7 | 铁损系数 K2 初始值 |
| k3 | 3.0 | 常数损耗（W），从 RLS 目标中减掉 |
| etaInit | 1.0 | τ·ω 系数 η：1 = 理想机械功率，可按传动效率标定 |
| errUpper | 50.0 | 隶属度上限阈值（|err| ≥ 此值时全按需求分配） |
| errLower | 0.01 | 隶属度下限阈值（|err| ≤ 此值时全按功率分配） |
| rlsLambda | 0.99999 | RLS 遗忘因子 |
| rlsEnable | false | RLS 在线辨识开关 |
| tauOmegaEnable | true | 是否包含 η·τ·ω 项（关掉后预测与求解都不含它） |
| pInit | 1e-5 | RLS 协方差初值 |
| k1Min / k1Max | 0 / 1e4 | K1 辨识结果钳位范围（负值一律不允许） |
| k2Min / k2Max | 0 / 1e4 | K2 辨识结果钳位范围 |
| minBudgetShare | 0.0 | 每台电机的最低预算份额（0~1/N），防某台被分到 0 预算后被迫退出驱动 |
| regenMaxW | 0.0 | 回馈（制动倒灌）功率上限 W，<=0 不限制 |

注意：`motorCount` 不由 Config 设置，而是通过模板参数 `PowerCtrl<kMotorCount>` 在编译期确定。

注意：`rlsLambda` 与 `SetMeasuredPower()` 的调用频率耦合 —— RLS 每次**新样本**迭代
一次，所以遗忘时间常数 ≈ `1 / ((1-λ)·功率计帧率)`。底盘 1 kHz 控制、功率计 100 Hz
时，λ = 0.999 对应约 10 s 记忆；想更快跟随就把 λ 调小，但不要小到开始追噪声。

### 控制循环

```cpp
void control_task(void*, void*, void*)
{
    alg::power_ctrl::PowerCtrl<2> ctrl;
    ctrl.Init({...});

    for (;;)
    {
        /* ① 喂数据 */
        ctrl.SetMotorData(0, motor.GetNowCurrent(), motor.GetNowVelocity(), pidErr);

        /* ② 功率预测 + RLS */
        ctrl.SetMeasuredPower(powerMeter.GetPower());
        ctrl.Predict();

        /* ③ 功率分配 */
        ctrl.Allocate(45.0f);

        /* ④ 取限制后的电流 */
        float limited = ctrl.GetLimitedCurrent(0);

        k_msleep(1);
    }
}
```

### 辨识结果观察

开启 RLS 后，Predict() 内部自动更新 K1、K2：

```cpp
printf("K1=%e, K2=%e, Pred=%f, Meas=%f\n",
       ctrl.GetK1(), ctrl.GetK2(),
       ctrl.GetTotalPower(), measuredPower);
```

K1/K2 会从初始值逐渐收敛到真实值。收敛速度取决于 λ 和激励的丰富程度（转速/转矩变化范围）。

## RLS 在 PowerCtrl 中的工作方式

### 数据流

```
           ┌─────────────────┐
           │  SetMotorData   │
           │  SetMeasuredPower│
           └────────┬────────┘
                    ↓
           ┌─────────────────┐
           │   Predict()     │
           │                  │
           │  P_pred = K1·τ² │
           │        + K2·ω²  │
           │        + η·τ·ω  │
           │        + K3     │
           │                  │
           │  x = [Στ², Σω²] │
           │  y = P_meas     │
           │    - ΣK3 - ηΣτω │
           │       ↓         │
           │  RLS.Update(x,y)│
           │  K1=w[0], K2=w[1]│
           └─────────────────┘
```

### 为什么减去 K3 和 η·τ·ω

RLS 建模的是 `K1·τ² + K2·ω² = P_meas - ΣK3 - η·Στω`。

- K3 是每台电机各一份的固定损耗（总线上的实测总功率里含 `kMotorCount` 份），
  只减一份会让 K1/K2 整体偏高一个常数偏置；
- `η·τ·ω` 是真实的机械功率，必须显式减掉。若留在拟合目标里，K1/K2 会把它
  平均吸收进去，而 `Predict()` 的单机预测又会再加一次 `η·τ·ω`，等于重复计入。

两个量都是已知的，留在辨识里只会污染 K1、K2 的物理含义。

### 激励条件

RLS 需要 **持续激励** 才能收敛。具体来说，输入向量 `α = [τ², ω²]` 的两个分量需要独立变化。如果电机始终匀速（ω² 恒定）且转矩极小（τ² ≈ 0），K1 和 K2 无法同时辨识。建议的做法：

1. 在电机起停、变速、加减载过程中收集数据
2. 速度变化范围大 → K2 收敛快
3. 转矩变化范围大 → K1 收敛快
4. 固定 K3 作为已知量，减少辨识维度

## 关键边界条件

- **不超限**：总**指令**功率未超过预算时，电流直通，不做限制
- **超限求解**：对每个电机解 `K1·τ² + η·ω·τ + (K2·ω² + K3 - P_limit) = 0`，
  取目标同侧的根；`K1 ≈ 0`（未标定）时退化成一次方程 `η·ω·τ + (K2·ω² + K3 - P_limit) = 0`
- **同号钳位**：解出的限幅值若与目标电流反号，一律输出 0 —— 预算不够时方程的两个根
  可能全在目标反侧（例如 ω>0 且份额小于 `K2·ω²+K3`），照抄会变成"PID 要正电流、
  限幅给负电流"的反向驱动
- **只削不加**：限幅结果的幅值超过目标时回退到目标，限幅不会主动加大输出
- **预算份额归一**：`Σmembership = 1`，即 `ΣpowerLimit = budget`；`minBudgetShare > 0`
  时每台至少拿到该份额（否则误差恰为 0 的轮子份额为 0，超限时只能退出驱动）
- **回馈制动**：`P_cmd ≤ 0` 的电机不参与"耗电份额"统计，不会因为发电反而分到更多预算
- **index 越界**：SetMotorData 和 GetLimitedCurrent 静默返回
- **delta < 0**：二次方程判别式负数时归零
- **NaN / Inf**：实测功率、回归向量、参数、输出任一环节出现非有限值都会被丢弃或钳位，
  不会扩散成 NaN 电流指令

> 注意：**预算低于空载损耗时无法靠限流守住总功率**。某台电机在转速 ω 下的空载损耗
> 是 `K2·ω² + K3`，只要它还在转就一定会消耗这么多，限流最多把电流压到 0。
> 例如 `K2 = 5.2e-3`、`K3 = 3` 时，ω = 40 rad/s 的单台空载损耗就是 11.3 W，
> 四台 45 W —— 正好等于预算。所以功率超标排查时，先确认 `K2 / K3` 是否被过拟合，
> 以及预算是否大于"目标转速下的空载损耗总和"。

## 典型配置示例

### 单电机辨识

```cpp
alg::power_ctrl::PowerCtrl<1> ctrl;
alg::power_ctrl::PowerCtrl<1>::Config cfg{};
cfg.rlsEnable = true;
ctrl.Init(cfg);

// 循环中:
ctrl.SetMotorData(0, current, rpm, 0.0f);
ctrl.SetMeasuredPower(powerMeter.GetPower());
ctrl.Predict();
// K1, K2 逐步收敛
```

### 四电机底盘功率限制

```cpp
alg::power_ctrl::PowerCtrl<4> chassisCtrl({
    4.577e-5f, 1.453e-7f, 1.453e-7f, 3.5f, 1.0f,
    0.001f, 500.0f, 0.99999f, false   // 不开启 RLS，用固定 K1/K2
});

// 循环中:
for (int i = 0; i < 4; i++)
    chassisCtrl.SetMotorData(i, current[i], rpm[i], pidErr[i]);
chassisCtrl.Predict();
chassisCtrl.Allocate(45.0f);     // 裁判系统 45W
for (int i = 0; i < 4; i++)
    limitedCurrent[i] = chassisCtrl.GetLimitedCurrent(i);
```

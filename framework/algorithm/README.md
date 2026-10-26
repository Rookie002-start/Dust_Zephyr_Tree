# algorithm/ — 算法层

纯计算模块。提供控制器、滤波器、参数辨识、缓冲区与线性代数等算法封装。

**边界**：只做数学计算和状态更新，**不创建线程、不持有硬件外设句柄**（PWM/UART 等），
纯数据输入输出。所有算法零硬件依赖，可在仿真或离线环境独立测试。

## 目录

```
algorithm/
├── buffer/          ← 缓冲区（BipBuffer 双区 / RingBuf FIFO）
├── controller/      ← 控制器（PID / 功率控制 / 软件定时器 / 执行时间测量）
├── filter/          ← 滤波器（低通 / 高通 / Kalman / EKF / 四元数姿态）
├── identify/        ← 参数辨识（RLS / 电机本体 / 稳定判据）
├── math/eigen/      ← 内置 Eigen 线性代数库
├── CMakeLists.txt   ← 按 Kconfig 开关追加 include/sources
├── Kconfig          ← 全部 DUST_* 符号（default n）
└── ARCHITECTURE.md
```

---

## buffer/ — 缓冲区

### BipBuffer — 双区环形缓冲

保证任何可读数据块在物理内存中**连续、永不分段**，天然零拷贝 DMA 友好。适合 DMA 直接写入的
流式数据接收与数据包解析。

**模板**：`template<uint32_t Capacity> class BipBuffer`，Capacity 为总字节数，内嵌
`alignas(64) uint8_t buffer_[Capacity]`（64 字节对齐利于 DMA 缓存一致性）。

**API**：

```cpp
uint8_t* Reserve(uint32_t size);            // 预留连续写空间，失败返回 nullptr
uint8_t* ReserveForceWrap(uint32_t size);   // 尾部装不下时强制回绕头部写
void      Commit(uint32_t size);            // 提交已写入字节
uint8_t*  GetContiguousReadBlock(uint32_t& out_size);  // 取第一个连续可读块
void      Decommit(uint32_t size);          // 从可读块头部消费
static constexpr uint32_t GetCapacity();
size_t    GetUsedSize() const;
void      Reset();
```

**用法（DMA 零拷贝四步协议）**：

```cpp
BipBuffer<4096> buf;
uint8_t* p = buf.Reserve(2048);      // 1. 预留连续空间
// DMA 直接写 p（零拷贝）
buf.Commit(received_bytes);          // 2. 提交已写入字节
uint32_t size;
uint8_t* data = buf.GetContiguousReadBlock(size);  // 3. 读连续块
parse_packet(data, size);
buf.Decommit(size);                  // 4. 释放已消费
```

**设计要点**：
- 维护 A、B 两个物理区域，B 永远从 `buffer_[0]` 开始；写入回绕时在头部建 B 区，读空 A 后 B 自动
  提升为 A。任何可读块连续，无分段。
- 写采用"预留—提交"两步（`Reserve→Commit`），一次只能有一个未提交预留。
- `ReserveForceWrap` 尾部装不下时跳过尾部直接去头部建 B 区，避免 CPU 判断尾部后白跑一次。
- 单生产者单消费者模型，**非线程安全**，调用方自行保证互斥。

### RingBuf — 环形缓冲 FIFO

基于 UART 驱动环形缓冲提取的单字节 FIFO，内嵌数组、不动态分配，写满时丢弃新数据、保留旧数据，
通过保留一格区分空/满。

**模板**：`template<uint32_t Capacity> class alg::buffer::RingBuf final`，实际可用 `Capacity-1`。

**API**：

```cpp
uint32_t Write(const uint8_t* data, uint32_t len);   // 满则丢弃，返回实际写入
uint32_t Read(uint8_t* buf, uint32_t max_len);       // 返回实际读取
uint32_t Available() const;
uint32_t Discard(uint32_t len);                      // 丢弃，返回实际丢弃
bool Empty() const; bool Full() const; void Clear();
```

**Kconfig**：`DUST_BUF_BIPBUF`（BipBuffer，无依赖）、`DUST_BUF_RINGBUF`（RingBuf，无依赖）。

---

## controller/ — 控制器

### PID — 位置式 PID

位置式（输出 = 累计积分 + 当前项），支持**微分先行、变速积分、积分分离、D 项低通、角度劣弧模式、
前馈**。

**API**：

```cpp
enum class DFirst : uint8_t { Disable=0, Enable };   // Enable 时 D 作用于测量值

struct Config {
    float kp=0, ki=0, kd=0, kf=0;      // kf=前馈
    float iOutMax=0;                    // 0=无积分限幅
    float outMax=0;                     // 0=无输出限幅
    float dt=0.001f;                    // 秒
    float deadZone=0;
    float iSpeedThreshLo=0, iSpeedThreshHi=0;  // 变速积分：|err|≤Lo 全速，≥Hi 停+清零
    float iSeparateThresh=0;            // 积分分离，0=关（与 Hi 功能重叠，建议二选一）
    DFirst dFirst=Disable;
    float dLpfCutoffHz=0;               // 0=无滤波
};

class alg::pid::Pid final {
    Pid(); explicit Pid(const Config&); void Init(const Config&);
    float Calc(float target, float now);    // 设目标+测量再算
    float Calc();                           // 用已设的 target/now
    float CalcAngle();                      // 角度模式劣弧环绕
    void SetShadow(const Config&);          // 运行中改参，Calc 内异步生效
    // SetKp/Ki/Kd/Kf/SetOutMax/SetIOutMax/SetTarget/SetNow/SetIntegralError
    // GetConfig/GetKp/Ki/Kd/Kf/GetOutMax/GetDt/GetOut/GetTarget/GetError...
};
```

**用法**：

```cpp
alg::pid::Pid pid({.kp=3.0f, .ki=0.1f, .kd=0.05f, .outMax=60.0f});
float out = pid.Calc(target, now);
// 角度定值跟踪用 CalcAngle()（劣弧环绕）；目标突变推荐 dFirst=DFirst::Enable 消除微分冲击
```

**Kconfig**：`DUST_CTL_PID`（无依赖）。

### PowerCtrl — 功率控制器

单组电机功率控制器：**功率模型预测 + RLS 在线辨识 K1/K2 + 隶属度功率分配 + 受限力矩求解**。

**模板**：`template<uint8_t kMotorCount> class alg::power_ctrl::PowerCtrl final`（编译期静态分配，
零动态内存）。

**管线**：

```cpp
struct Config {
    float torqueK=4.577e-5f;      // M3508 电流→转矩
    float k1Init=1.453e-7f;       // 铜损
    float k2Init=1.453e-7f;       // 铁损
    float k3=3.0f;                // 固定损耗
    float errUpper=50.0f, errLower=0.01f;   // 隶属度阈值
    float rlsLambda=0.99999f;
    bool  rlsEnable=false;        // false=固定 K1/K2
    bool  tauOmegaEnable=true;    // 是否含 τ·ω 项
};

alg::power_ctrl::PowerCtrl<4> ctrl({.rlsEnable=true});
// 每周期：
for (i) ctrl.SetMotorData(i, torque, omega, pidErr);   // 喂转矩/角速度/PID 误差
ctrl.SetMeasuredPower(voltage*current);                // 实测功率
ctrl.Predict();                                        // 预测功率 + 更新 K1/K2
ctrl.Allocate(totalBudgetW);                           // 功率分配（超预算解二次方程）
for (i) float cur = ctrl.GetLimitedCurrent(i);         // 写电机
```

**Kconfig**：`DUST_MOD_CTL_POWER`（select `DUST_CTL_PID` + `DUST_ID_RLS` + `DUST_FLT_LPF`）。

### Timer — 软件定时器

协作式软件定时器，不占硬件 timer，必须由外部周期调用 `Update()` 驱动。

```cpp
Timer timer(1000);                    // 1000 ticks
while (1) {
    timer.Update();                   // 每周期必须先 Update
    timer.Clock([]{ led.Toggle(); }); // 周期满回调
    // 或 timer.Tick([&]{ if(timer.GetCounter()==500){...} });  // 周期内回调
    k_msleep(1);
}
// SetPeriod(2000) 周期结束生效；Pause/Resume 冻结/断点续；Stop() 停止清零
```

**Kconfig**：`DUST_CTL_TIMER`（无依赖）。

### ExecTimer — 执行时间测量

基于 Zephyr cycle 计数器（`k_cycle_get_32()`）测量代码段执行时间。

```cpp
ExecTimer t(1000);                    // 每 1000 次输出一次
t.SetStart();
func();
t.SetEnd();
t.PrintTotal();                       // printk "%.3f ms"
```

**Kconfig**：`DUST_CTL_EXECTIMER`（依赖 Zephyr 内核）。

---

## filter/ — 滤波器

### HPF / LPF — 一阶 RC 高低通

基于截止频率的标量一阶滤波器。LPF：`out = α·in + (1-α)·out`，`α = dt/(dt+RC)`，`RC=1/(2π·fc)`；
HPF = 输入减一阶低通输出。`cutoffHz<=0` 时直通。

```cpp
alg::filter::LowPassFilter lpf(100.0f, 0.001f);   // 100 Hz, 1 ms
float out = lpf.Update(raw);
alg::filter::HighPassFilter hpf(100.0f, 0.001f);
float out2 = hpf.Update(raw);
```

**Kconfig**：`DUST_FLT_LPF`、`DUST_FLT_HPF`（均无依赖）。

### Kalman — 标准线性 Kalman

模板化固定尺寸（编译期优化，无动态分配）：

```cpp
// Kalman<2,1>：弹簧阻尼系统
Eigen::Matrix2f A; A << 1.0f, dt, -k*dt, 1.0f-c*dt;
Eigen::RowVector2f H; H << 1.0f, 0.0f;
Kalman<2,1> kf;
kf.Init(A, H, Matrix2f::Identity()*0.01f, Matrix<float,1,1>::Identity()*0.1f);
kf.SetZ(z); kf.Predict(); kf.Update();
float pos = kf.GetX()(0);
```

**Kconfig**：`DUST_FLT_KALMAN`（select `DUST_MATH_EIGEN`）。

### ExtendedKalman — 扩展 Kalman（EKF）

用 SystemFunc/ObserveFunc 回调替代 Hook 模式，每周期重算 Jacobian F/H。工程化扩展：
`SetGainScale`（自适应增益）、`PMinus()`（渐消因子）、`GetChi2()`（卡方门控）、
`FallbackToPrediction()`（发散回退）、Joseph 形式 P 更新。

**Kconfig**：`DUST_FLT_KALMAN_EKF`（select `DUST_MATH_EIGEN`）。

### QuaternionEkf — 四元数 IMU 姿态 EKF

内部封装 `ExtendedKalman<6,3,4>`，估四元数 + 陀螺零偏（z 轴零偏不可观，用静止期滑动均值补偿），
RK1 四元数积分 + 重力方向观测。输出 roll/pitch/yaw、q[4]、bg[3]。

```cpp
alg::attitude::QuaternionEkf qekf;
qekf.Init(config);
qekf.Update(sample);              // 每帧 IMU 采样驱动
auto& st = qekf.GetState();       // st.roll/pitch/yaw (deg), st.q[4], st.bg[3]
```

**Kconfig**：`DUST_FLT_QUATERNION`（select `DUST_FLT_KALMAN_EKF`）。

---

## identify/ — 参数辨识

### RLS — 递归最小二乘

编译期定维的 RLS 自适应滤波器，估 SISO 线性回归参数。内存全编译期静态，无堆。

```cpp
alg::rls::RLS<2> filter(0.99f, 10.0f);   // lambda, p_init
float alpha[2] = {1.0f, 2.0f};
filter.Update(alpha, 5.0f);
float k1 = filter.GetWeights()[0];
```

**Kconfig**：`DUST_ID_RLS`（select `DUST_MATH_EIGEN`）。

### MotorPlant — 电机本体在线辨识

基于 RLS 的电机本体辨识（torque→omega），估阻尼/增益/摩擦，含零阶保持延时对齐与异步 CAN 反馈处理。

```cpp
MotorPlant plant(cfg);
plant.OnTorqueSend(t_req_us, torque_nm);   // 记录发送的转矩
plant.OnFeedback(t_rx_us, omega, seq);     // 反馈角速度
if (plant.IsModelReady()) {
    float tau = plant.GetTau();            // 时间常数
    float k   = plant.GetK();              // 增益
}
```

**Kconfig**：`DUST_ID_MOTOR_PLANT`（select `DUST_ID_RLS`）。

### WinStable — 稳定判据 + 波形发生器

滑动窗口稳定判据：连续推入直到窗口满才判"稳定"（极差 ≤ noise_limit、斜率 ≤ slope_limit、
净漂移 ≤ drift_limit），另带方波/三角波/正弦波发生器。

```cpp
stability::WinStable<64> stable;
bool ok = stable.Check(temp_c, dt_s);
```

**Kconfig**：`DUST_ID_STABILITY`（header-only 无依赖）。

---

## math/eigen/ — 线性代数

内置 vendored Eigen 5（header-only），被 Kalman/EKF/RLS 等 `select DUST_MATH_EIGEN` 自动启用。

---

## Kconfig / CMake 装配

### 符号清单（全部 default n，opt-in）

| 符号 | 说明 | 依赖 |
| --- | --- | --- |
| `DUST_BUF_BIPBUF` | BipBuffer 双区环形缓冲 | 无 |
| `DUST_BUF_RINGBUF` | 环形缓冲 FIFO | 无 |
| `DUST_CTL_EXECTIMER` | 代码段执行时间测量 | Zephyr 内核 |
| `DUST_CTL_PID` | 位置式 PID | 无 |
| `DUST_CTL_TIMER` | 软件定时器 | 无 |
| `DUST_FLT_HPF` | 一阶高通 | 无 |
| `DUST_FLT_LPF` | 一阶低通 | 无 |
| `DUST_FLT_KALMAN` | 标准 Kalman | `DUST_MATH_EIGEN` |
| `DUST_FLT_KALMAN_EKF` | 扩展 Kalman | `DUST_MATH_EIGEN` |
| `DUST_FLT_QUATERNION` | 四元数姿态 EKF | `DUST_FLT_KALMAN_EKF` |
| `DUST_ID_RLS` | 递归最小二乘 | `DUST_MATH_EIGEN` |
| `DUST_ID_MOTOR_PLANT` | 电机本体辨识 | `DUST_ID_RLS` |
| `DUST_ID_STABILITY` | 稳定判据 + 波形发生器 | 无 |
| `DUST_MATH_EIGEN` | Eigen 线性代数库 | 无 |
| `DUST_MOD_CTL_POWER` | 功率控制器 | `DUST_CTL_PID`+`DUST_ID_RLS`+`DUST_FLT_LPF` |

**依赖链是单向的**：`MATH_EIGEN` 最底层（被 Kalman/EKF/RLS select）；`KALMAN_EKF` 依赖
`KALMAN`（被 `QUATERNION` select）；`RLS` 被 `MOTOR_PLANT` 和 `MOD_CTL_POWER` select。

**使用方式**：业务线程在 `project/Kconfig` 里 `select DUST_CTL_PID=y` 等；带 select 的符号自动
拉起依赖。CMake 按符号把对应 include 目录/源文件加进 app（大部分 header-only，少数有 .cpp：
`pid.cpp`、`kalman_check.cpp`、`quaternion.cpp`、`motorplant.cpp`）。

---

## 设计原则

- **零硬件依赖** — 算法只依赖 C/C++ 标准库（ExecTimer 例外，用 Zephyr 内核计时），可在仿真/离线测试。
- **opt-in 裁剪** — 一个 Kconfig 符号 = 一个可裁剪模块，默认不引入任何代码。
- **编译期静态** — 模板定维（Kalman/RLS/RingBuf/PowerCtrl），无动态分配，适合嵌入式。
- **头文件为主** — 大多数模块 header-only，模板实例化由使用方控制。
- **单向依赖** — 谁用线性代数 select `DUST_MATH_EIGEN`，谁用 RLS select `DUST_ID_RLS`，不反向依赖业务层。

详见 [ARCHITECTURE.md](ARCHITECTURE.md)。

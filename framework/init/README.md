# init/ — 启动层

`init/` 是当前工程的统一启动层，负责把 `main()`、链接段注册表、初始化阶段、线程启动阶段和公共中断分发入口串起来。

它不属于某一个具体机器人项目，也不属于某个设备模块。它回答的问题是：

```text
镜像里已经被编译进来的组件，应该按什么顺序初始化和启动？
```

---

## 职责

`init/` 主要负责：

- 提供 `System_Startup()`，作为 `src/main.c` 调用的系统启动入口；
- 定义 `REGISTER_INIT()` 和 `REGISTER_THREAD()`；
- 遍历 `.user_init` 和 `.user_thread` 链接段；
- 按阶段执行初始化函数和线程启动函数；
- 根据 `InitLevel` 处理初始化失败；
- 提供 CAN RX 链接段分发入口；
- 将 `cmd/linker/tflm_init.ld` 接入 Zephyr 链接流程。

它不负责：

- 决定某个业务线程是否编译；
- 写具体 IMU、Remote、底盘或云台初始化逻辑；
- 直接访问 UART、CAN、SPI、GPIO 等外设；
- 保存模块运行状态；
- 实现算法或 topic 数据契约。

这些内容分别属于 `project/`、`modules/`、`drivers/`、`algorithm/` 和 `topic/`。

---

## 目录

```text
init/
├── CMakeLists.txt
├── Init_entry.hpp
├── Init_entry.cpp
├── System_startup.h
├── Irq_handlers.h
└── Irq_handlers.cpp
```

| 文件 | 作用 |
| --- | --- |
| `System_startup.h` | 对 C 入口暴露 `System_Startup()` |
| `Init_entry.hpp` | 定义启动阶段、失败等级、注册项结构和注册宏 |
| `Init_entry.cpp` | 遍历 `.user_init/.user_thread` 并执行启动阶段 |
| `Irq_handlers.h` | 定义 `CAN_RX_HANDLER()` 和 CAN RX 分发入口 |
| `Irq_handlers.cpp` | 根据 CAN ID 遍历 `.can_rx1/2/3` 并调用回调 |
| `CMakeLists.txt` | 编译启动层源文件并加入链接脚本 |

---

## 当前启动链

根入口位于：

```text
src/main.c
```

当前入口关系是：

```text
main()
    ->
System_Startup()
    ->
RunStage(...)
```

`main()` 不直接初始化 IMU、Remote、CAN、底盘或云台。具体组件是否存在，先由 Kconfig/CMake 决定；具体组件什么时候运行，再由 `init/` 按链接段表遍历。

---

## 阶段顺序

当前启动器按下面的顺序执行：

```text
PreInit
    ->
PreThread
    ->
EarlyInit
    ->
EarlyThread
    ->
MidInit
    ->
MidThread
    ->
LateInit
    ->
LateThread
    ->
AppInit
    ->
AppThread
```

每一组通常表达：

```text
先初始化对象和底层资源
    ->
再启动依赖这些资源的线程循环
```

示例：

| 阶段 | 适合放什么 |
| --- | --- |
| `PreInit` | 早期总线、调试控制台、基础输出 |
| `PreThread` | 早期通信线程或基础 IO 线程 |
| `EarlyInit` | 传感器、输入设备、需要较早准备的数据源 |
| `EarlyThread` | 传感器采样线程、输入线程 |
| `MidInit` | 底盘、云台等控制对象 |
| `MidThread` | 主要控制线程 |
| `LateInit` | 依赖前面能力的通信或上位机接口 |
| `LateThread` | 后期通信、调试或辅助线程 |
| `AppInit` | Demo、测试、推理示例等应用级入口 |
| `AppThread` | Demo、测试、推理线程 |

这张表是组织建议，不是强制依赖声明。真正的依赖关系仍应通过 Kconfig `select`、模块接口和文档写清楚。

---

## 注册宏

### REGISTER_INIT

初始化函数使用 `REGISTER_INIT()` 注册到 `.user_init`：

```cpp
static bool thread_init()
{
    return true;
}

REGISTER_INIT(thread_init, MidInit, Mid, "foo_init");
```

宏展开后会生成一个静态 `InitEntry`，并放入 `.user_init` 段。启动器运行时通过 `__user_init_start` 和 `__user_init_end` 遍历。

### REGISTER_THREAD

线程启动函数使用 `REGISTER_THREAD()` 注册到 `.user_thread`：

```cpp
static bool thread_start()
{
    return true;
}

REGISTER_THREAD(thread_start, MidThread, Mid, "foo_start");
```

这样可以把“对象初始化”和“线程启动”分开，避免线程启动时依赖的资源还没有准备好。

---

## 失败等级

每个启动项都有一个 `InitLevel`：

| 等级 | 当前行为 |
| --- | --- |
| `High` | 打印错误，触发错误蜂鸣并停机 |
| `Mid` | 打印告警，继续启动 |
| `Low` | 打印告警，继续启动 |

`InitLevel` 表示失败处理策略，不是同一阶段内的排序优先级。当前同阶段执行顺序由链接段中的条目顺序决定，因此不要把同阶段顺序当作强依赖机制。

如果组件之间存在硬依赖，优先用不同阶段表达；如果依赖仍然复杂，应在组件的 `thread_init()` 中显式检查依赖对象是否 ready。

---

## 链接段关系

`init/` 运行时会消费这些链接段边界：

| 链接段 | 注册宏 | 边界符号 |
| --- | --- | --- |
| `.user_init` | `REGISTER_INIT()` | `__user_init_start/end` |
| `.user_thread` | `REGISTER_THREAD()` | `__user_thread_start/end` |
| `.can_rx1` | `CAN_RX_HANDLER(USER_RX_CAN1, ...)` | `__can_rx1_start/end` |
| `.can_rx2` | `CAN_RX_HANDLER(USER_RX_CAN2, ...)` | `__can_rx2_start/end` |
| `.can_rx3` | `CAN_RX_HANDLER(USER_RX_CAN3, ...)` | `__can_rx3_start/end` |

这些 section 的实际定义当前集中在：

```text
cmd/linker/tflm_init.ld
```

`init/CMakeLists.txt` 通过：

```cmake
zephyr_linker_sources(ROM_SECTIONS ../cmd/linker/tflm_init.ld)
```

把链接段定义加入 Zephyr 链接流程。

---

## CAN RX 分发

CAN 接收回调不是在每个业务文件里手写总分发器，而是用 `CAN_RX_HANDLER()` 注册：

```cpp
CAN_RX_HANDLER(USER_RX_CAN1, 0x201, motor_rx_handler, motor_201);
```

注册项进入 `.can_rx1` 后，`user_can1_rx_callback()` 会按 CAN ID 遍历并调用对应处理函数。

这和 `REGISTER_INIT()` 的思路一致：

```text
业务文件本地声明
    ->
链接段收集
    ->
统一入口遍历
```

---

## 新增线程的接入方式

一个典型项目线程应该在自己的 `trd_xxx.cpp` 中完成注册：

```cpp
#include "Init_entry.hpp"

namespace thread::foo {

static bool thread_init()
{
    return Foo::Instance().Init();
}

static bool thread_start()
{
    Foo::Instance().Start();
    return true;
}

REGISTER_INIT(thread_init, MidInit, Mid, "foo_init");
REGISTER_THREAD(thread_start, MidThread, Mid, "foo_start");

}  // namespace thread::foo
```

新增线程通常还需要同步修改：

```text
project/thread/Kconfig
project/thread/CMakeLists.txt
project/boards/<vendor>/<board_cfg>/<board>.overlay
```

但不应该修改：

```text
src/main.c
init/Init_entry.cpp
```

也就是说，`init/` 提供启动框架，具体组件通过本地注册进入框架。

---

## 与其他层的边界

| 层 | 和 init 的关系 |
| --- | --- |
| `src/` | 只调用 `System_Startup()` |
| `project/` | 选择板级配置和业务线程，线程本地注册启动项 |
| `drivers/` | 提供外设访问能力，不直接决定启动阶段 |
| `modules/` | 提供设备能力，可被项目线程初始化 |
| `algorithm/` | 提供纯计算能力，通常不直接注册启动项 |
| `topic/` | 提供线程间数据契约，不参与启动调度 |
| `cmd/` | 可以注册调试控制面，也提供当前链接脚本位置 |

---

## 设计原则

- **main 保持干净**：`main()` 只进入系统启动器，不关心具体组件。
- **本地注册**：组件在自己的 `.cpp` 中声明启动项。
- **编译决定存在**：Kconfig/CMake 未选中的源文件不会贡献注册项。
- **运行决定顺序**：`init/` 运行时按阶段遍历链接段。
- **失败策略显式**：每个启动项声明自己的 `InitLevel`。
- **不隐藏强依赖**：阶段只能表达粗粒度顺序，复杂依赖要在接口和 ready 检查中写清楚。

---

## 三次筛选：配置、链接、运行

一个初始化函数能否运行，要经过三个不同时间点的筛选：

```text
Kconfig / CMake
        ↓
目标文件和链接段
        ↓
System_Startup() 的阶段遍历
```

| 时间点 | 关键问题 | 典型证据 |
| --- | --- | --- |
| 配置和编译 | 这个功能是否被选择，源文件是否进入编译 | `CONFIG_*`、`CMakeLists.txt` |
| 链接 | 注册项是否被放入目标 section，section 是否被保留 | `REGISTER_*`、`.user_init`、map 文件 |
| 运行 | 当前阶段是否遍历到该注册项，函数是否返回成功 | 启动日志、失败等级、设备 ready 状态 |

因此，源码中存在 `REGISTER_INIT()` 不等于函数一定会运行；函数进入
`.user_init` 也不等于设备已经初始化成功。排查启动问题时应按这三个
层次逐步确认，不要把链接证据当成硬件证据。

## 阶段、等级、优先级和周期

这四个概念解决不同问题：

| 概念 | 所属位置 | 作用 |
| --- | --- | --- |
| `InitStage` | 初始化注册项 | 决定一次性初始化在哪个阶段执行 |
| `ThreadStage` | 线程注册项 | 决定线程创建或启动请求在哪个阶段执行 |
| `InitLevel` | 注册项 | 决定返回 `false` 后是停止还是继续 |
| RTOS priority | 线程实现或创建参数 | 决定线程被调度时与其他线程的相对优先级 |
| period / sleep | 线程循环 | 决定线程自身的运行周期 |

不能用提高线程优先级来修复初始化阶段错误，也不能用推迟线程周期来
替代设备 ready 检查。阶段只表达系统启动时的粗粒度依赖，实时调度由
线程实现和 RTOS 负责。

## 当前项目注册概览

当前 `project/` 中的注册项可以按启动职责概括为：

| 组件 | 初始化阶段 | 线程阶段 | 等级 | 作用 |
| --- | --- | --- | --- | --- |
| GPIO 输出 | `PreInit` | `PreThread` | `Low` | 准备基础输出 |
| 遥控器 | `PreInit` | `PreThread` | `High` | 初始化接收和协议探测 |
| CAN TX | `PreInit` | `PreThread` | `High` | 建立统一发送线程 |
| GPIO 输入 | `EarlyInit` | `EarlyThread` | `Low` | 准备输入资源 |
| PC 通信 | `PreInit` | `LateThread` | `High` | 接入调试或上位机链路 |
| IMU | `EarlyInit` | `LateThread` | `High` | 启动采样、校准和姿态输出 |
| 底盘 | `MidInit` | `MidThread` | `Mid` | 运行底盘控制 |
| 云台 | `MidInit` | `MidThread` | `Mid` | 运行云台控制 |
| 测试 | `LateInit` | `LateThread` | `High` | 设备验证和实验功能 |
| TFLM | `AppInit` | `AppThread` | `Low` | 运行应用层功能 |

这张表描述当前项目的实际装配，不是所有项目都必须采用同样的线程集合。
新增项目组件时，应根据真实依赖选择阶段和等级，而不是机械复制表格。

## 注册项的生命周期

一个本地注册项从源代码到执行的路径如下：

```text
拥有资源的 .cpp
      ↓
REGISTER_INIT / REGISTER_THREAD
      ↓
InitEntry 常量对象
      ↓
.user_init / .user_thread
      ↓
cmd/linker/tflm_init.ld 的 KEEP
      ↓
__user_*_start / __user_*_end
      ↓
RunStage()
      ↓
InitFunc()
```

`RunStage()` 只根据 stage 过滤当前 section 中的项，然后调用对应函数。
它不重新解析组件依赖，也不根据名字排序。需要严格顺序的资源应通过
阶段、模块 ready 检查或明确的接口契约表达，不能假设链接器会按源文件
顺序给出业务顺序。

## 初始化函数和线程函数契约

初始化函数应完成一次性资源准备，并返回明确结果：

- `true` 表示当前启动项完成了它承诺的准备工作；
- `false` 表示无法完成，`InitLevel` 决定是否停止后续启动；
- 不应在初始化函数中放置无限循环；
- 需要持续执行的功能应拆成线程注册项；
- 初始化函数中的阻塞等待应有明确上限或可诊断状态。

线程函数应负责自己的循环、周期和退出条件：

- 创建或启动动作由统一启动链触发；
- 线程内部处理设备离线、数据超时和降级；
- 不应通过修改全局启动器来表达业务模式；
- 线程间通信使用已有 topic、消息队列或模块快照契约。

## 失败等级的实际含义

`InitLevel` 不是日志级别，而是启动策略：

| 等级 | 初始化返回 `false` 时 | 适用场景 |
| --- | --- | --- |
| `High` | 停止当前启动流程或进入故障停机 | 系统不可运行的基础资源 |
| `Mid` | 记录失败并继续后续阶段 | 某个功能不可用但系统可降级 |
| `Low` | 记录警告并继续 | 可选功能、实验功能或非关键输出 |

选择等级时要按“系统没有这个功能时还能否安全运行”判断，而不是按组件
大小判断。一个体积很小的通信入口也可能是 `High`，一个很复杂的实验
功能也可能是 `Low`。

## 链接段与专用消费者

`cmd/linker/tflm_init.ld` 当前保留的注册段包括：

| section | 注册内容 | 消费者 |
| --- | --- | --- |
| `.user_init` | 一次性初始化项 | `RunStage()` |
| `.user_thread` | 线程注册项 | `RunThreadStage()` |
| `.can_rx1` | CAN1 ID 回调 | CAN1 RX callback |
| `.can_rx2` | CAN2 ID 回调 | CAN2 RX callback |
| `.can_rx3` | CAN3 ID 回调 | CAN3 RX callback |
| `.remote` | 遥控器协议实现 | `Remote` 模块 |
| `.imu` | IMU 芯片或实现项 | IMU 模块 |
| `.shell_var` | shell 变量或控制项 | shell/命令侧代码 |

只有 `.user_init` 和 `.user_thread` 由通用阶段遍历直接消费。CAN、remote、
imu 和 shell section 有各自的消费者，不能把它们都当成启动表。

## CAN RX 分发检查

CAN RX 注册的完整路径是：

```text
CAN_RX_HANDLER(bus, id, handler)
        ↓
.can_rx1 / .can_rx2 / .can_rx3
        ↓
__can_rxN_start / __can_rxN_end
        ↓
user_canN_rx_callback()
        ↓
线性匹配 CAN ID
        ↓
handler(data)
```

排查“回调不进”时，应分别检查：

1. 注册宏使用的 bus 是否对应实际控制器。
2. 目标 ID 是否与驱动收到的 ID、扩展帧标志一致。
3. 链接脚本是否保留对应 `.can_rxN`。
4. 设备是否完成过滤器配置和 `can_start()`。
5. CAN 控制器 IRQ 和设备树中断是否已使能。
6. 回调是否只做快速解析，是否把耗时工作移到线程。

注册表存在只能证明软件回调入口被收集，不能证明总线已经启动或物理层
已经收到有效帧。

## 新增组件清单

- [ ] 确认组件拥有的资源和初始化承诺。
- [ ] 判断它需要一次性初始化、线程，还是现有模块的被动回调。
- [ ] 选择合适的 stage、level、线程优先级和周期。
- [ ] 在本地源文件中使用 `REGISTER_INIT()` / `REGISTER_THREAD()`。
- [ ] 确认源文件受 Kconfig 和 CMake 控制。
- [ ] 确认注册 section 被 `tflm_init.ld` 保留。
- [ ] 如果使用专用 section，确认存在对应消费者。
- [ ] 明确初始化失败后的系统行为。
- [ ] 明确设备未 ready、数据超时和线程退出时的行为。
- [ ] 用静态搜索检查宏、section 符号和调用路径。

`init/` 的变更应优先扩展注册能力或公共分发能力，不应把具体设备协议
和业务策略搬进启动层。

## 启动问题的证据顺序

面对一个启动或回调问题，建议按以下顺序收集证据：

```text
配置是否打开
    ↓
源文件是否编译
    ↓
注册宏是否生成
    ↓
链接段是否保留
    ↓
阶段是否遍历到
    ↓
初始化函数返回值
    ↓
驱动设备状态和真实硬件数据
```

这个顺序可以把“没有编译进来”“没有链接进来”“没有被阶段调用”和
“硬件初始化失败”区分开，避免只看到一条启动日志就扩大修改范围。

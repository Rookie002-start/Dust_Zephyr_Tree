# Zephyr HPM5E3Y 开发环境搭建指南（从零开始）

> 适用对象：从未接触过先楫 HPM 系列的开发者。本文把"让 `hpm5e3y`（HPM5E00 系列 / HPM5E3Y 芯片）
> 能在 Zephyr 里编译"这件事，从**原理 → 目录结构 → 源码获取 → 逐文件移植 → 编译验证 → 踩坑记录**
> 完整讲一遍。照着做就能从零复现出可编译的 hpm5e3y 开发环境。
>
> 本文配套的已落地实现位于（本机示例路径）：
> - 用户小树（自研层）：`~/Zephyr_HPMicro/sdk_glue_user`
> - 官方 zephyr 适配层：`~/Zephyr_HPMicro/sdk_glue`
> - 先楫 SDK：`~/Zephyr_HPMicro/sdk_env`（内含 `hpm_sdk`）
>
> 已实现并验证：
> ```
> source ~/zephyrproject/setup_env.sh
> cd ~/zephyrproject/projects/temp
> west build -b hpm5e3y -d build_hpm5e3y .
> # 产物：build_hpm5e3y/zephyr/zephyr.elf / .hex / .bin
> ```

---

## 目录

- [0. 背景与结论](#0-背景与结论)
- [1. 概念：三个仓库分别是什么](#1-概念三个仓库分别是什么)
- [2. 前置条件](#2-前置条件)
- [3. 第一步：获取源码与建立目录](#3-第一步获取源码与建立目录)
- [4. 第二步：应用既有补丁（apply-patches.sh）](#4-第二步应用既有补丁apply-patchessh)
- [5. 第三步：为什么需要"移植"HPM5E3Y](#5-第三步为什么需要移植hpm5e3y)
- [6. 第四步：新增 HPM5E00 SoC 支持（用户小树）](#6-第四步新增-hpm5e00-soc-支持用户小树)
- [7. 第五步：新增 HPM5E3Y 时钟宏头文件](#7-第五步新增-hpm5e3y-时钟宏头文件)
- [8. 第六步：新增 HPM5E00/HPM5E3Y 设备树](#8-第六步新增-hpm5e00hpm5e3y-设备树)
- [9. 第七步：新增 hpm5e3y 板卡](#9-第七步新增-hpm5e3y-板卡)
- [10. 第八步：挂载进官方 sdk_glue（5 处修改）](#10-第八步挂载进官方-sdk_glue5-处修改)
- [11. 第九步：编译验证](#11-第九步编译验证)
- [12. 踩坑记录与解决方法](#12-踩坑记录与解决方法)
- [13. 烧录与调试](#13-烧录与调试)
- [14. 后续：把板卡做成你们自己的硬件](#14-后续把板卡做成你们自己的硬件)
- [15. 参考资料](#15-参考资料)

---

## 0. 背景与结论

**背景**：先楫 zephyr 适配层（`sdk_glue`）官方只带了
`hpm6200evk / hpm6750evk2 / hpm6800evk / hpm6e00evk`；
用户层（`sdk_glue_user`）此前只带了 `hpm5361icb`（HPM5300 系列）。
**没有 `hpm5e3y`，所以直接 `west build -b hpm5e3y` 会找不到板卡。**

**结论**：HPM5E3Y 属于 **HPM5E00 系列**，单核 Andes D45 RISC-V、480 MHz、
带 1MB 片内 Flash / 128K ILM / 128K DLM / 256K AXI-SRAM，主打 EtherCAT/工业以太。
`hpm_sdk v1.11.0` 里已经有该芯片的底层库（`soc/HPM5E00/HPM5E31`），
因此**只需要补齐"Zephyr 侧"的三件事**：
SoC 适配、芯片设备树 + 时钟宏、板卡定义。做完后即可编译出可烧录固件。

> **⚠️ HPM5E3Y 与 HPM5E31 的关系（务必先读）**
>
> - 经逐文件比对，两者在 `hpm_soc_ip.h / hpm_soc_irq.h / hpm_iomux.h / SVD` 等软件可见层面**完全一致**
>   （同一颗料的寄存器视图）；差别主要在物理/市场定位：HPM5E3Y 是机器人关节专用版
>   （内置 EtherCAT PHY、小封装、105℃），与 HPM6E8Y 互补。
> - **hpm_sdk v1.11 的可编译型号目录是 HPM5E31**（HPM5E3Y 目录只有寄存器头文件、没有驱动/CMake），
>   因此本工程 Zephyr 侧 SoC 命名用 **HPM5E3Y**，并在 `sdk_glue/modules/hpm_sdk/CMakeLists.txt`
>   加了 “HPM5E3Y → SDK HPM5E31” 的一行别名映射（见补丁 0008）。
> - 官方 `hpm5e00evk` 出厂芯片是 HPM5E31；本文 `hpm5e3y` 板卡的引脚/代码可直接参考它。


---

## 1. 概念：三个仓库分别是什么

以本机工作区 `~/Zephyr_HPMicro` 为例：

```text
Zephyr_HPMicro/                    # HPM 侧根目录（独立 workspace）
├── sdk_glue/                      # 官方 hpmicro/zephyr_sdk_glue —— “Zephyr 适配层”
│     ├── soc/hpmicro/             #   SoC 层（官方支持 HPM6200/6700/6800/6E00）
│     ├── boards/hpmicro/          #   官方板卡
│     ├── drivers/ dts/ include/   #   Zephyr 驱动 / 设备树 / 头文件
│     └── zephyr/module.yml
├── sdk_env/                       # 官方 hpmicro/sdk_env（含 hpm_sdk v1.11.0 + 烧录工具）
│     └── hpm_sdk/
│           ├── soc/HPM5E00/HPM5E31/   # ← 芯片寄存器/中断/时钟/IP 定义（我们照它写 dtsi/soc.c）
│           └── boards/hpm5e00evk/     # ← 官方评估板参考（引脚/初始化参考）
└── sdk_glue_user/                 # 用户自研小树（独立 git 仓库）
      ├── soc/hpmicro/HPM5300/     #   HPM5300 系列 SoC（此前已由团队自研）
      ├── soc/hpmicro/HPM5E00/     #   ← 本次新增：HPM5E00 系列 SoC
      ├── boards/hpmicro/hpm5361icb/   #   用户自制板
      ├── boards/hpmicro/hpm5e3y/      #   ← 本次新增：hpm5e3y 板卡
      ├── dts/riscv/hpmicro/hpm5exx.dtsi / hpm5e3y.dtsi   # ← 本次新增设备树
      ├── include/dt-bindings/clock/hpm5e3y-clocks.h      # ← 本次新增时钟宏
      └── zephyr-patches/sdk_glue/     # 对官方 sdk_glue 的补丁（0001~0008）
```

另外还有一个"纯 zephyr 侧"（STM32 时代就有的）：
`~/zephyrproject`（zephyr 源码 + SDK + `zephyr_user` + `projects/<app>`）。
HPM 工程通过应用 `CMakeLists.txt` 把 `sdk_glue` / `sdk_glue_user` 以
`BOARD_ROOT / SOC_ROOT / DTS_ROOT / ZEPHYR_EXTRA_MODULES` 挂进 Zephyr 构建。

**记忆口诀**：
- 芯片底层寄存器/驱动库 → `sdk_env/hpm_sdk`
- Zephyr 官方适配（驱动/soc/板卡）→ `sdk_glue`
- 你们自己新写的板卡/SoC/设备树 → `sdk_glue_user`（独立新增文件，不污染官方）
- 必须改官方文件的地方 → 走 `sdk_glue_user/zephyr-patches/*.diff` 打补丁

---

## 2. 前置条件

| 项 | 要求 |
| --- | --- |
| 硬件 | HPM5E3Y 板卡（本次按官方 `hpm5e00evk` 引脚作参考）；调试器 cmsis-dap / FT2232 |
| 系统 | Linux（本指南命令均为 Linux）；Windows 用 PowerShell 等价命令 |
| Zephyr 环境 | 已按《zephyr_子树架构搭建指南.md》搭好的 `~/zephyrproject`（Zephyr v4.4.1 + zephyr-sdk） |
| 工具 | git、west、cmake、ninja、dtc、gperf、python3（west 的 .venv） |
| HPM 工具 | `sdk_env`（含 hpm_sdk v1.11.0 与 openocd） |

> Zephyr SDK 需含 riscv 工具链；HPM 侧所有板卡都用 **riscv32 / riscv64-zephyr-elf** 工具链编译。

---

## 3. 第一步：获取源码与建立目录

```bash
mkdir -p ~/Zephyr_HPMicro && cd ~/Zephyr_HPMicro

# 1) 官方 zephyr 适配层（本实现基于 1756cb8）
git clone https://github.com/hpmicro/zephyr_sdk_glue sdk_glue
git -C sdk_glue checkout 1756cb8

# 2) 官方 sdk_env（hpm_sdk v1.11.0 已含 HPM5E00）
git clone --depth 1 --branch v1.11.0 https://github.com/hpmicro/sdk_env sdk_env

# 3) 用户自研小树（本实现与 hpm5361icb/HPM5300 都在这里）
git clone git@github-qingyu0310:qingyu0310/Dust_Zephyr_HPMicro_Tree.git sdk_glue_user
```

若从零开始**没有**这个小树，可只保留本次新增内容的最小目录结构（见第 6~9 章）。

### 3.1 设置环境变量（重要）

应用工程 `CMakeLists.txt` 用到的 `${ZEPHYR_SDK_GLUE_MODULE_DIR}` 就来自这里：

```bash
export ZEPHYR_BASE=~/zephyrproject/zephyr
export ZEPHYR_SDK_INSTALL_DIR=~/zephyrproject/zephyr-sdk-1.0.1
export ZEPHYR_TOOLCHAIN_VARIANT=zephyr
export SDK_GLUE_DIR=~/Zephyr_HPMicro/sdk_glue
export ZEPHYR_SDK_GLUE_MODULE_DIR=~/Zephyr_HPMicro/sdk_glue
```

---

## 4. 第二步：应用既有补丁（apply-patches.sh）

`sdk_glue_user` 把"必须修改官方仓库文件"的部分做成了补丁，脚本会逐个 `git apply --check` 后应用：

```bash
cd ~/Zephyr_HPMicro/sdk_glue_user
./apply-patches.sh \
  --zephyr    ~/zephyrproject/zephyr \
  --sdk-env   ~/Zephyr_HPMicro/sdk_env \
  --sdk-glue  ~/Zephyr_HPMicro/sdk_glue \
  --cherryusb <CherryUSB 仓库路径，没有就随便给一个存在的目录>
```

> 本任务新增了 `zephyr-patches/sdk_glue/0008-soc-hpm5e00.diff`（把 HPM5E00 挂进官方 sdk_glue），
> `apply-patches.sh` 已更新为依次应用 0001~0008。

---

## 5. 第三步：为什么需要"移植"HPM5E3Y

HPM 在 Zephyr 里的支持 = **sdk_glue(官方 soc/驱动) + sdk_glue_user(自研 soc/板卡) + sdk_env(hpm_sdk)**。
要让一颗新芯片能编译，一般缺三样：

1. **SoC 层**（start.S 启动、soc.c 时钟初始化、Kconfig 让 Zephyr 认识这颗 SoC、linker）；
2. **芯片设备树**（`hpm5exx.dtsi`：CPU、中断控制器、定时器、内存、外设地址与中断号）+
   **时钟宏头文件**（`hpm5e3y-clocks.h`，供 dtsi 里写 `clocks = <&clk CLOCK_UART0 ...>`）；
3. **板卡**（`hpm5e3y.dts` + pinctrl + defconfig + Kconfig + board.cmake）。

数据来源全部在 `hpm_sdk/soc/HPM5E00/HPM5E31/` 里查得到：
- 外设基地址 → `hpm_soc_ip.h`
- 中断号 → `hpm_soc_irq.h`
- 时钟枚举/时钟源 → `hpm_clock_drv.h`、`hpm_sysctl_drv.h`
- 引脚复用编号 → `hpm_iomux.h`（如 `IOC_PA00_FUNC_CTL_UART0_TXD = ALT2`）

> ⚠️ **千万别照抄其它芯片的 dtsi**：HPM5E3Y 的外设基地址、中断号与 HPM5300/HPM6E00
> 都不同（例如 DLM 在 `0x00200000` 而非 HPM5300 的 `0x00080000`；UART0 中断是 19 而非 13）。

---

## 6. 第四步：新增 HPM5E00 SoC 支持（用户小树）

新建目录：`~/Zephyr_HPMicro/sdk_glue_user/soc/hpmicro/HPM5E00/`，共 9 个文件。
**最快做法**：把已有的 `soc/hpmicro/HPM5300/` 整个复制过来，然后按下面逐文件修改。

| 文件 | 作用 / 修改要点 |
| --- | --- |
| `Kconfig.soc` | 声明 `SOC_SERIES_HPM5E00`（select `SOC_FAMILY_HPM`/`HAS_HPMSDK_PLLCTLV2`/`HAS_HPMSDK_PCFG`）、`SOC_SERIES="HPM5E00"`、`SOC_HPM5E3Y`、`SOC="HPM5E3Y"` |
| `Kconfig` | `SOC_SERIES_HPM5E00` 选择 RISC-V/PLIC/动态中断；`SOC_HPM5E3Y` 选择 `RISCV_ISA_EXT_M/A/C/ZICSR/ZIFENCEI`、cache、`RISCV_PMP`；CPU ISA 与 FPU 选择项（HPM5E3Y 支持双精度 FPU → `HPM5E_DOUBLE_PRECISION_FPU`） |
| `Kconfig.defconfig.series` | `SYS_CLOCK_HW_CYCLES_PER_SEC=24000000`、`KERNEL_ENTRY="_start"`、ICACHE/DCACHE、中断聚合参数、**`NUM_IRQS=128`**；最后 `source` 同目录 `Kconfig.defconfig.HPM5E3Y` |
| `Kconfig.defconfig.HPM5E3Y` | 栈大小等默认值（ISR/MAIN/IDLE 等） |
| `soc.h` | 与 HPM5300 相同：`RISCV_MTIME_BASE/MTIMECMP_BASE` 取自设备树 mtimer |
| `start.S` | 与 HPM5300 相同：mstatus 清零 → 按 `CONFIG_ICACHE/DCACHE` 开关 L1 cache → `c_startup` → `__start` |
| `soc.c` | 时钟/功耗初始化：preset2 → 把 cpu0/总线/外设时钟加入 group0 → DCDC 1.275V+DCM → **PLL0 到 480MHz**（`pllctlv2_init_pll_with_freq(…, 480000000)`）→ cpu0 用 `clk_src_pll0_clk0` → mchtmr0 用 24M 晶振；`SYS_INIT` 挂 `PRE_KERNEL_1` |
| `linker.ld` | 与 HPM5300 相同（通用脚本，含 `<../common/linker.ld>`） |
| `CMakeLists.txt` | `zephyr_sources(start.S soc.c)`；`CONFIG_SOC_ANDES_V5_HWDSP` 时加 `-mext-dsp`；`CONFIG_SOC_SERIES_HPM5E00` 时把 `SOC_LINKER_SCRIPT` 指到本目录 `linker.ld` |

> soc.c 里用的时钟名（`clock_ahb0/clock_axif/clock_xpi0/clock_rom0/clock_ram0`…）必须是
> HPM5E3Y `hpm_clock_drv.h` 里真实存在的名字，照抄 HPM5300 的 `clock_ahb/clock_rom` 会编译不过。
> HPM5E3Y 的时钟域结构和 HPM6E00 更像（有 `clock_ahb0/axif/axis/axic`），没有 `clock_ahb`。

---

## 7. 第五步：新增 HPM5E3Y 时钟宏头文件

`sdk_glue` 每个芯片都有 `include/dt-bindings/clock/<chip>-clocks.h`（如 `hpm5361-clocks.h`），
供设备树写 `clocks = <&clk CLOCK_UART0 CLK_SRC_PLL1_CLK0 6>`。
官方 sdk_glue **没有 HPM5E3Y** 的版本，用官方脚本从 hpm_sdk 自动生成：

```bash
# 生成脚本位置：sdk_glue/scripts/gen_clock_dts_header.py
# 它默认输出到 sdk_glue/include/dt-bindings/clock，这里把输出改到用户小树 include
sed 's#output_dir = os.path.join(utils.SDK_GLUE_BASE, "include", "dt-bindings", "clock")#output_dir = "~/Zephyr_HPMicro/sdk_glue_user/include/dt-bindings/clock"#' \
  ~/Zephyr_HPMicro/sdk_glue/scripts/gen_clock_dts_header.py > /tmp/gen_hpm5e31.py

cd ~/Zephyr_HPMicro/sdk_glue/scripts
PYTHONPATH=$PWD python3 /tmp/gen_hpm5e31.py --soc HPM5E31 --org HPMicro --license APACHE
```

得到：`sdk_glue_user/include/dt-bindings/clock/hpm5e31-clocks.h`（生成器输出名）。
> 生成器以 SDK 编译目标 HPM5E31 命名输出；请复制/重命名为 **`hpm5e3y-clocks.h`**（内容逐字节相同，
> 只是文件名与 include guard 不同），dtsi 统一 include `hpm5e3y-clocks.h`。
（内含 `CLOCK_UART0/CLOCK_CAN0/CLOCK_MOT0/CLOCK_AHB0/CLOCK_XPI0/CLOCK_USB0…` 与 `CLK_SRC_*`）。

> HPM5E3Y 没有 `CLOCK_AHB`，只有 `CLOCK_AHB0`；`CLOCK_MOT0` 才是 PWM/MOT 域时钟。

---

## 8. 第六步：新增 HPM5E00/HPM5E3Y 设备树

新建（用户小树）：
```text
sdk_glue_user/dts/riscv/hpmicro/hpm5e3y.dtsi   # 只 include 下面这个
sdk_glue_user/dts/riscv/hpmicro/hpm5exx.dtsi   # 芯片主设备树
```

`hpm5exx.dtsi` 骨架（照着 `hpm53xx.dtsi` 的写法，但要按 HPM5E3Y 改）：

```dts
/dts-v1/;
#include <zephyr/dt-bindings/gpio/gpio.h>
#include <dt-bindings/clock/hpm5e3y-clocks.h>
#include <mem.h>
#include <freq.h>

/ {
    cpus { cpu0: cpu@0 {
        compatible = "andestech,andescore-d45", "riscv";  /* HPM5E3Y 是 D45 内核 */
        riscv,isa-base = "rv32i";
        riscv,isa-extensions = "i","m","a","f","d","c","zicsr","zifencei","xandes";
        ...
    }; };

    soc {
        compatible = "hpmicro,hpm5e00";
        ranges;
        clk: clock-controller@f40c0000 {
            compatible = "hpmicro,hpm-clock";
            reg = <0xf4000000 DT_SIZE_K(256)   /* SYSCTL */
                   0xf40c0000 DT_SIZE_K(16)>;  /* PLLCTLV2 */
            #clock-cells = <3>;                 /* cells: name, src, div */
        };
        ilm: memory@0        { reg = <0x0 DT_SIZE_K(128)>; };   /* ILM */
        dlm: memory@200000   { reg = <0x200000 DT_SIZE_K(128)>; }; /* DLM，注意不是 0x80000 */
        plic: interrupt-controller@e4000000 { ... };
        mtimer: machine-timer@e6000000 { ... };
        xpi0: xpi@f3000000 { compatible = "hpmicro,xpi"; ... };  /* 片内 XPI Flash 控制器 */
        gpio0: peripheral@f00d0000 { ... };
        uart0: serial@f0040000 { /* 中断 19(中断号来自 hpm_soc_irq.h) */ };
        ...
    };
};

&gpio0 { gpioa: gpio@0 {...}; gpiob: gpio@1 {...}; };
```

HPM5E3Y 关键寄存器地址（摘自 `hpm_soc_ip.h`）：
```
SYSCTL 0xF4000000  PLLCTLV2 0xF40C0000  PLIC 0xE4000000  MCHTMR 0xE6000000
UART0..7 0xF0040000..(+0x4000)   SPI0..3 0xF0070000..(+0x4000)
GPIO0 0xF00D0000  HDMA 0xF00C8000  DMAMUX 0xF00C4000
MCAN0..3 0xF0300000..(+0x4000)    PWM0/1 0xF0420000/0xF0424000
XPI0 0xF3000000   USB0 0xF1420000  ENET0 0xF1400000
```

关键中断号（摘自 `hpm_soc_irq.h`）：
```
GPIO0_A/B=1/2   GPTMR0..3=11..14   UART0..7=19..26
I2C0..3=27..30  SPI0..3=31..34     HDMA=40
MCAN0..3=46..49 PWM0=53 PWM1=54    ENET0=58 USB0=60 XPI0=65
```

注意事项：
- **PWM 用 `compatible = "hpmicro,hpm-pwmv2"`**（HPM5E3Y 是 PWMv2），时钟写
  `clocks = <&clk CLOCK_MOT0 CLOCK_AHB0 1>`；不要照抄 hpm5361 的 `hpmicro,hpm-pwm`。
- UART 节点的 binding 要求 `interrupts` 至少 1 个、`clocks`、`reg` 必填；RX 空闲用的
  `idletimer` 中断、`uart-idle-*` 属性在用到 UART+异步 DMA 时才需要。
- **GPIO 端口**：HPM5E3Y 的 IOMUX pad 只有 `PA..PF` 与 `PV..PX`（见 `hpm_iomux.h`），
  **没有 PY/PZ**。设备树里别声明不存在的端口（HPM5300 能用的 `gpioy` 在 HPM5E3Y 上没有）。
- UART 设备树 `interrupts = <19 1>, <11 1>` 的第二个中断（idletimer）在 HPM5E3Y 上对应
  GPTMR0..3 的中断号，启用方式参考 hpm5e00evk 官方 `board.c/h` 的 UART 空闲定时器用法。

---

## 9. 第七步：新增 hpm5e3y 板卡

新建：`sdk_glue_user/boards/hpmicro/hpm5e3y/`

| 文件 | 说明 |
| --- | --- |
| `hpm5e3y.dts` | include `<hpmicro/hpm5e3y.dtsi>` + pinctrl；`chosen`：console=uart0、sram=dlm、flash=flash0、itcm=ilm、flash-controller=xpi0；使能 uart0/gpioa；`&xpi0` 下挂 1MB `flash@80000000` 与分区 |
| `hpm5e3y-pinctrl.dtsi` | 引脚复用：默认 `pinmux_uart0 = PA0/PA1 ALT2`（115200 控制台，官方 EVK 即此） |
| `hpm5e3y_defconfig` | `CONFIG_XIP=y`、`CONFIG_PINCTRL/GPIO/SERIAL/CONSOLE/UART_CONSOLE/FLASH=y`、`CONFIG_DCACHE=n`、`CONFIG_HPM5E_DOUBLE_PRECISION_FPU=y`、**`CONFIG_ROM_START_OFFSET=0x3100`** |
| `board.yml` / `hpm5e3y.yaml` | 板卡元数据（soc: hpm5e3y） |
| `Kconfig` / `Kconfig.defconfig` / `Kconfig.hpm5e3y` | `BOARD_HPM5E3Y` → `select SOC_HPM5E3Y`（+`ZEPHYR_HPM_SDK_MODULE`）等 |
| `board.c` | 空 `SYS_INIT` |
| `CMakeLists.txt` / `pre_dt_board.cmake` | 常规板卡构建文件 |
| `board.cmake` | 烧录（openocd）：参考 hpm5361icb 写法，soc cfg 用官方 `soc/hpm5e00.cfg` + `boards/hpm5e00evk.cfg`，探针用 cmsis-dap |

`hpm5e3y.dts` 的 `chosen` 要点：

```dts
chosen {
    zephyr,console = &uart0;
    zephyr,shell-uart = &uart0;
    zephyr,sram = &dlm;            /* RAM 区域：DLM 128K @0x200000 */
    zephyr,flash = &flash0;        /* 1MB @0x80000000 */
    zephyr,itcm = &ilm;
    zephyr,code-partition = &slot0_partition;
    zephyr,flash-controller = &xpi0;
};
```

> `zephyr,sram/flash` 选好后，`CONFIG_SRAM_BASE_ADDRESS/SRAM_SIZE` 等会自动由设备树推导，
> 不需要手写。Flash 分区请按你们实际板卡 Flash 布局改。

---

## 10. 第八步：挂载进官方 sdk_glue（5 处修改）

官方 sdk_glue 的 `soc/hpmicro/` 用通配 + 显式 `rsource` 把各系列 SoC 拉进来；
HPM5E00 在 `sdk_glue_user` 小树里，必须像 HPM5300 一样**显式挂载**。共 5 处：

### 10.1 `sdk_glue/soc/hpmicro/Kconfig`
在 HPM5300 挂载行后追加：
```kconfig
# SELF-MAINTAINED — 用户自研 HPM5E00 SoC 支持（sdk_glue_user 小树）
rsource "${ZEPHYR_SDK_GLUE_MODULE_DIR}/../sdk_glue_user/soc/hpmicro/HPM5E00/Kconfig"
```

### 10.2 `sdk_glue/soc/hpmicro/Kconfig.soc`
```kconfig
# SELF-MAINTAINED — 用户自研 HPM5E00 SoC 支持（sdk_glue_user 小树）
source "${ZEPHYR_SDK_GLUE_MODULE_DIR}/../sdk_glue_user/soc/hpmicro/HPM5E00/Kconfig.soc"
```

### 10.3 `sdk_glue/soc/hpmicro/Kconfig.defconfig`
```kconfig
# SELF-MAINTAINED — 用户自研 HPM5E00 SoC 支持（sdk_glue_user 小树）
rsource "${ZEPHYR_SDK_GLUE_MODULE_DIR}/../sdk_glue_user/soc/hpmicro/HPM5E00/Kconfig.defconfig.series"
```

### 10.4 `sdk_glue/soc/hpmicro/CMakeLists.txt`
把 HPM5300 的条件分支扩成二选一：
```cmake
if(SOC_SERIES STREQUAL "HPM5300")
    add_subdirectory(${ZEPHYR_SDK_GLUE_MODULE_DIR}/../sdk_glue_user/soc/hpmicro/HPM5300 ${CMAKE_CURRENT_BINARY_DIR}/HPM5300)
elseif(SOC_SERIES STREQUAL "HPM5E00")
    # SELF-MAINTAINED — 用户自研 HPM5E00 SoC 支持（sdk_glue_user 小树）
    add_subdirectory(${ZEPHYR_SDK_GLUE_MODULE_DIR}/../sdk_glue_user/soc/hpmicro/HPM5E00 ${CMAKE_CURRENT_BINARY_DIR}/HPM5E00)
else()
    add_subdirectory(${SOC_SERIES})
endif()
```

### 10.5 `sdk_glue/soc/hpmicro/soc.yml`
在 `hpm5300` 系列后新增：
```yaml
  - name: hpm5e00
    socs:
    - name: hpm5e11
    - name: hpm5e3y
```

> 以上 5 处已固化成补丁：
> `sdk_glue_user/zephyr-patches/sdk_glue/0008-soc-hpm5e00.diff`
> 干净环境直接 `./apply-patches.sh` 即可，无需手改。

---

## 11. 第九步：编译验证

### 11.1 用官方 hello_world 验证（最快）

建一个最小应用目录（或用 `zephyr/samples/hello_world`），CMakeLists 需要把用户小树挂进来：

```cmake
cmake_minimum_required(VERSION 3.20.0)
set(SDK_GLUE_USER_DIR "/home/<you>/Zephyr_HPMicro/sdk_glue_user")
list(APPEND BOARD_ROOT "${SDK_GLUE_USER_DIR}")
list(APPEND SOC_ROOT   "${SDK_GLUE_USER_DIR}")
list(APPEND DTS_ROOT   "${SDK_GLUE_USER_DIR}")
list(APPEND DTS_ROOT   "${SDK_GLUE_USER_DIR}/dts")
set(ZEPHYR_EXTRA_MODULES "/home/<you>/Zephyr_HPMicro/sdk_glue")
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(hello_hpm5e3y)
target_sources(app PRIVATE src/main.c)
```

```bash
west build -b hpm5e3y -d build hello_hpm5e3y/
```

预期：编译链接通过，末尾出现
```
ROM:   ~3%   1 MB
RAM:   ~17%  128 KB
```
产物 `build/zephyr/zephyr.elf / .hex / .bin`。

### 11.2 用实际工程（temp）验证

```bash
cd ~/zephyrproject/projects/temp
west build -b hpm5e3y -d build_hpm5e3y .
```

预期：整包 C/C++（含 framework 与线程）链接通过：
```
ROM:  58484 B / 1 MB  (~5.6%)
RAM:  26052 B / 128 KB (~19.9%)
```

> 若只是验证芯片/板卡，11.1 足够；如果应用代码里用到了具体外设/引脚，需要按第 14 章
> 在板卡 overlay 里补引脚。

---

## 12. 踩坑记录与解决方法

按"报错现象 → 原因 → 解决"整理如下（本次实际遇到）：

### 12.1 soc.c 编译报错：`irq_lock / irq_unlock` implicit declaration
- 原因：soc.c 里用了 `irq_lock()/irq_unlock()` 但没包含 `<zephyr/irq.h>`。
- 解决：在 soc.c 顶部加 `#include <zephyr/irq.h>`。

### 12.2 链接阶段报错：`cannot move location counter backwards (from 8000302c to 80000000)`
- 原因：XIP + 启动头模式下，`rom_start` 片段里 `.start` 之后要把位置推进到
  `CONFIG_ROM_START_OFFSET` 才开始放向量表/正文；不设该值时推进量为 0，位置会“倒退”，链接失败。
- 解决：板卡 `hpm5e3y_defconfig` 加
  `CONFIG_ROM_START_OFFSET=0x3100`（与 hpm5361icb 一致：0x1000 启动头之后留出 0x3100 起始）。

### 12.3 设备树报错：`CLOCK_AHB` / `CLOCK_PWM0` 未定义
- 原因：照抄了 hpm5361 的 PWM 写法；HPM5E3Y 的时钟宏里**没有 `CLOCK_AHB`**（只有 `CLOCK_AHB0`），
  PWM 是 PWMv2，时钟域是 `CLOCK_MOT0`。
- 解决：PWM 节点改为
  `compatible = "hpmicro,hpm-pwmv2"; clocks = <&clk CLOCK_MOT0 CLOCK_AHB0 1>;`
  （与 sdk_glue 官方 hpm6exx.dtsi 的写法一致）。

### 12.4 设备树引用了 HPM5E3Y 不存在的 GPIO 端口（gpioy / gpioc）
- 原因：HPM5361 有 `gpioy`，HPM6E00 甚至有 A~Z 很多口；HPM5E3Y 的 pad 只有 `PA..PF + PV..PX`。
- 解决：只保留实际存在的端口（gpioa/gpiob…），以 `hpm_soc_ip.h` / `hpm_iomux.h` 为准核对。

### 12.5 缺 `hpm5e3y-clocks.h`
- 原因：官方 sdk_glue 没生成 HPM5E3Y 的时钟宏。
- 解决：用官方 `scripts/gen_clock_dts_header.py --soc HPM5E31` 自动生成（见第 7 章），
  输出到用户小树 include；不要手写数值，避免和 hpm_sdk 枚举不一致。

### 12.6 中断号/基地址照抄 HPM5300 导致"看着对但运行时错"
- 原因：HPM5E3Y 与 HPM5361 中断布局、部分外设基址不同。
- 解决：**以 HPM5E3Y 自己的 `hpm_soc_irq.h` / `hpm_soc_ip.h` 为准**逐项填写 dtsi。
  例如 DLM=0x00200000、UART0 IRQ=19、MCAN0=0xF0300000、PWM0=0xF0420000、USB0=0xF1420000。

### 12.7 缺少 `ZEPHYR_SDK_GLUE_MODULE_DIR`
- 原因：所有 `rsource/source "${ZEPHYR_SDK_GLUE_MODULE_DIR}/../sdk_glue_user/..."` 依赖该变量。
- 解决：`export ZEPHYR_SDK_GLUE_MODULE_DIR=~/Zephyr_HPMicro/sdk_glue`（见 3.1）。

### 12.8 找不到板卡 `hpm5e3y`
- 原因：应用 CMakeLists 没把 `sdk_glue_user` 挂到 `BOARD_ROOT/SOC_ROOT/DTS_ROOT`，
  或 sdk_glue 的 5 处挂载没做。
- 解决：按第 10 章补挂载 + 应用 CMakeLists 加 `BOARD_ROOT`（见 11.1）。

---

## 13. 烧录与调试

板卡 `board.cmake` 已按官方 HPMicro 评估板写好 openocd runner（cmsis-dap 探针）：

```bash
west flash            # 编译目录内执行，或 west flash -d build_hpm5e3y
west debug
west debugserver
```

- 若用官方 `hpm5e00evk`（出厂芯片为 HPM5E31，与 HPM5E3Y 寄存器级一致）：控制台 = UART0（PA0/PA1）115200，插上板载 FT2232 即可。
- 若是你们自制的 HPM5E3Y 板：改 `board.cmake` 里的 openocd 配置与探针类型即可。

---

## 14. 后续：把板卡做成你们自己的硬件

本次 `hpm5e3y` 板卡按官方 EVK 引脚做了"能编译、能起串口"的最小配置。自制板请改：

1. **引脚**：`sdk_glue_user/boards/hpmicro/hpm5e3y/hpm5e3y-pinctrl.dtsi`
   - 每个外设先查 `soc/HPM5E00/HPM5E31/hpm_iomux.h` 确认该引脚该功能的 ALT 值；
   - 在 `hpm5exx.dtsi` 里确认外设节点存在并打开；
   - 板卡 dts 里 `&uartX { pinctrl-0 = <&pinmux_uartX>; …; status = "okay"; }`。
2. **外设使能**：在板卡 dts/overlay 里打开需要的 `mcan0/spi1/pwm0/usb0…`，并补
   `clocks`/DMA/中断相关属性。
3. **Flash 分区**：按实际 Flash 容量改 `flash@80000000` 的 `reg` 和 partitions。
4. **烧录**：按实际调试器改 `board.cmake`。

> 应用工程级 overlay（对应你 temp 工程 `boards/hpm/hpm5361icb/` 那种用法）：
> 在应用里新建 `boards/<vendor>/<board_cfg>/hpm5e3y.overlay` 与 `.conf`，即可在不改用户小树的前提下
> 为 hpm5e3y 定制外设/引脚/业务 Kconfig。

---

## 15. 参考资料

- HPMicro 官方 zephyr 适配层：https://github.com/hpmicro/zephyr_sdk_glue
- HPM SDK / sdk_env：https://github.com/hpmicro/sdk_env （本机 `~/Zephyr_HPMicro/sdk_env`）
- HPM5E00 系列产品页：https://www.hpmicro.com/en/product-center/microcontroller/hpm5e00-series
- HPM5E00EVK 评估板说明：`~/Zephyr_HPMicro/sdk_env/hpm_sdk/boards/hpm5e00evk/README_zh.rst`
- 芯片数据（地址/中断/时钟/引脚）：
  `~/Zephyr_HPMicro/sdk_env/hpm_sdk/soc/HPM5E00/HPM5E31/{hpm_soc_ip.h,hpm_soc_irq.h,hpm_clock_drv.h,hpm_iomux.h}`
- 团队既有文档：`zephyr_user/doc/zephyr_HPM_搭建指南.md`（HPM5300 移植方法，HPM5E00 同套路）
- 已落地实现与补丁：
  - `sdk_glue_user/soc/hpmicro/HPM5E00/`
  - `sdk_glue_user/dts/riscv/hpmicro/hpm5exx.dtsi`、`hpm5e3y.dtsi`
  - `sdk_glue_user/include/dt-bindings/clock/hpm5e3y-clocks.h`
  - `sdk_glue_user/boards/hpmicro/hpm5e3y/`
  - `sdk_glue_user/zephyr-patches/sdk_glue/0008-soc-hpm5e00.diff`

# SVPWM_HALL — GD32 FOC Motor Drive Firmware

基于 **GD32F30x (Cortex-M4)** 的磁场定向控制（FOC）电机驱动固件，支持 **MT6816 磁编码器**，开环 SVPWM 及速度/位置闭环控制。
此版本为初始版本，还有很多不足，支持二次开发。
配套技术参考资料详见 [NTW的小站](https://ntw48.icu)

---

## 硬件要求

| 项目 | 说明 |
|------|------|
| MCU | GD32F303CB / GD32F30x 系列 (Cortex-M4 with FPU) |
| 编码器 | MT6816 14-bit 磁编码器 (SPI) |
| 驱动板 | 三相逆变桥 + 差分电流采样 (R010, gain≈36.5) |
| 调试接口 | USART0 (PB6 TX, PB7 RX) @115200 |

硬件开源：[全国产12-24V FOC驱动控制一体板](https://oshwhub.com/ntw48/project_rozcjswj)

## 特性

- **SVPWM** 空间矢量调制，中心对齐 PWM @20kHz
- **Clarke/Park** 坐标变换 + 增量式 PI 电流环
- **双 ADC 同步注入** 采样三相电流
- **MT6816** 14-bit 磁编码器驱动 + LUT 非线性补偿
- **PLL 速度估算** + 直接差分转速反馈
- **速度环 / 位置环** PI 控制器
- **S 曲线** 7 段轨迹规划（加加速度连续）
- **V/f 开环控制** 频率斜坡启动
- **非阻塞校准** 零点 → R/L → 编码器 LUT
- **Flash 存储** 校准结果掉电保存
- **USART DMA** 收发（FIFO 缓冲 + 命令解析）

## 目录结构

```
SVPWM_HALL/
├── app/                 应用入口
│   ├── main.c
│   ├── sys_malloc.c/h   内存分配初始化
│   └── usr_config.c/h   用户配置（修改这里适配新电机/板）
├── bsp/                 板级支持包（硬件抽象）
│   ├── bsp_adc.c/h      ADC + 电流采样
│   ├── bsp_gpio.c/h     GPIO / LED
│   ├── bsp_mt6816.c/h   编码器 SPI 驱动
│   ├── bsp_systick.c/h  系统时钟
│   ├── bsp_timer.c/h    PWM 定时器
│   └── bsp_usart.c/h    UART DMA 驱动
├── motor/               电机控制算法
│   ├── calib.c/h        校准状态机
│   ├── control.c/h      控制环（电流/速度/位置）
│   ├── flag_storage.c/h Flash 存储
│   ├── motor.c/h        电机数据结构 + PI 控制器
│   ├── svpwm.c/h        SVPWM + 坐标变换
│   └── trajectory.c/h   S 曲线轨迹规划
├── components/          通用组件
│   └── fifo.c/h         环形缓冲区
├── GD32F30x_LIB/        标准外设库（不动）
├── DEMO/                参考代码
└── doc/                 文档
```

## 快速开始

### 1. 克隆

```bash
git clone https://github.com/yourname/SVPWM_HALL.git
```

### 2. 修改配置

打开 `app/usr_config.h`，根据你的电机和驱动板调整参数：

```c
#define PWM_FREQ_HZ             20000       /* PWM 载波频率 */
#define CURRENT_SHUNT_R_OHM     0.010f      /* 采样电阻 */
#define ENCODER_POLE_PAIRS      7           /* 电机极对数 */
#define OL_START_FREQ_HZ        5.0f        /* 开环启动频率 */
/*注意电感L已经写死0.86mH，在代码里，用RL自测的话，需要改一下*/
```

### 3. 编译

使用 **Keil MDK-ARM V5** 打开 `AnchorFOC.uvprojx`，选择目标 `AnchorFOC`，编译下载。

### 4. 串口调试

连接 USART0 (PB6 TX, PB7 RX)，115200 8N1，发送命令：

| 命令 | 说明 |
|------|------|
| `0xAA 0x55 0x55 0xAA 0x01 0x01` | 电机启动 |
| `0xAA 0x55 0x55 0xAA 0x01 0x00` | 电机停止 |
| `0xAA 0x55 0x55 0xAA 0x09 0x02` | 速度闭环模式 |
| `0xAA 0x55 0x55 0xAA 0x09 0x04` | 位置闭环模式 |
| `0xAA 0x55 0x55 0xAA 0x08 [lo] [hi]` | 设目标转速 (RPM, little-endian) |

### 5. 校准流程

上电后自动执行校准：
1. **ADC 零点校准** — 测量零电流偏移
2. **相电阻测量** — PI 闭环注入电流
3. **相电感测量** — 交替方波法
4. **编码器校准** — 方向检测 → 极对数 → 128 点 LUT

校准完成后结果自动保存到 Flash，下次启动跳过。

## 数据流

```
main() loop [~1kHz]
  ├── FocSystemRun()
  │     ├── CALIB → 校准状态机
  │     └── RUN → 打印 Id/Iq/转速/角度
  └── UartPollDmaTx() → TX FIFO → DMA 发送

TIMER0 TRGO → ADC 触发 [20kHz]
  └── ADC0_1_IRQHandler
        ├── 读 ADC → 三相电流
        ├── 读 MT6816 → 编码器角度
        ├── PLL 速度估算
        ├── 位置环 → 速度环 → 电流环
        └── Clarke → Park → PI → InvPark → SVPWM
```

## 许可

GNU General Public License v3.0

Copyright (c) 2026 ntw48

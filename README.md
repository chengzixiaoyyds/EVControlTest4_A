# 水下机器人推进器控制系统

基于 STM32F103C8T6 的 ROV 推进器控制与通信系统，实现推力曲线拟合、多路电机 PWM 驱动、电流监测及上位机串口通信。

---

## 项目简介

本系统用于小型水下机器人（ROV）的推进器控制，通过 USART3 接收地面站下发的目标推力指令，经推力曲线模型转换为 PWM 信号驱动 4 路 DRV8871 直流电机推进器及 1 路舵机，同时通过 INA226 采集电流数据并回传。

---

## 硬件平台

| 组件 | 型号 | 说明 |
|------|------|------|
| 主控芯片 | STM32F103C8T6 | ARM Cortex-M3, 72MHz |
| 电机驱动 ×4 | DRV8871 | 3.6A 有刷直流电机驱动 |
| 电流传感器 | INA226 | I²C 接口, 12A 量程, 5mΩ 采样电阻 |
| 舵机 | 通用舵机 | 50Hz PWM, TIM1 CH3 |
| 通信接口 | USART3 | 115200bps, 8N1 |
| 开发环境 | Keil MDK-ARM + STM32CubeMX | — |

---

## 功能说明

### 1. 推力曲线拟合

基于 `thruster.xlsx` 实测数据，采用**二次拟合 + 死区**模型：

$$F =
\begin{cases}
a_{fwd} \cdot (PWM - N)^2, & PWM > N + \Delta \\
0, & N - \Delta \leq PWM \leq N + \Delta \\
-a_{rev} \cdot (N - PWM)^2, & PWM < N - \Delta
\end{cases}$$

| 参数 | 值 | 说明 |
|------|:---:|------|
| 中性点 N | 7.48% | 水阻力平衡点 |
| 死区半宽 Δ | 0.12% | 死区 [7.36%, 7.60%] |
| 正向系数 a_fwd | 639.88 | 拟合 RMS 误差 18.2N |
| 反向系数 a_rev | 362.95 | 拟合 RMS 误差 18.2N |

推进器通过 PWM 占空比调节净推力方向：
- PWM > 7.60%：推力 > 阻力 → 净正向力
- PWM = 7.48%：推力 = 阻力 → 净力为零
- PWM < 7.36%：推力 < 阻力 → 净反向力

### 2. 推力分配与 PWM 输出

四个推进器的空间布局与推力分配逻辑：

| 电机 | 位置 | 功能 | 分配公式 |
|------|:---:|------|------|
| **Motor A** | 左 | Y 轴前进 + Yaw 轴偏航差动（正方向向前） | $F_A = \frac{F_y}{2} + F_{yaw}$ |
| **Motor B** | 右 | Y 轴前进 + Yaw 轴偏航差动（正方向向前） | $F_B = \frac{F_y}{2} - F_{yaw}$ |
| **Motor C** | 后 | Z 轴升降（正方向向下） | $F_C = \frac{F_z}{2}$ |
| **Motor D** | 前 | Z 轴升降（正方向向下） | $F_D = \frac{F_z}{2}$ |

> **偏航力臂**：100mm（0.1m）。偏航通道接收的是**扭矩**（N·m），需先换算为力：
> $$F_{yaw} = \frac{\tau_{yaw}}{0.1\ \text{m}}$$
>
> X 轴推力预留，从串口接收但不参与电机分配。

DRV8871 每个电机使用 2 路 PWM 互补控制：

| 电机 | 目标量 | DRV8871 IN1 | DRV8871 IN2 | 物理引脚 | 正转时 | 反转时 |
|------|------|:---:|:---:|------|:---:|:---:|
| Motor A | Y + Yaw | TIM2 **CH1** | TIM2 **CH2** | PA0 / PA1 | IN1=HIGH, IN2=PWM | IN1=PWM, IN2=HIGH |
| Motor B | Y + Yaw | TIM2 **CH3** | TIM2 **CH4** | PA2 / PA3 | IN1=HIGH, IN2=PWM | IN1=PWM, IN2=HIGH |
| Motor C | Z | TIM3 **CH4** | TIM3 **CH3** | PB1 / PB0 | IN1=HIGH, IN2=PWM | IN1=PWM, IN2=HIGH |
| Motor D | Z | TIM3 **CH2** | TIM3 **CH1** | PA7 / PA6 | IN1=HIGH, IN2=PWM | IN1=PWM, IN2=HIGH |
| 舵机 | 机械臂角度 | — | TIM1 **CH3** | PA10 | — | — |

> DRV8871 控制逻辑：`DRV8871_SetSpeed(speed)` 中
> - `speed > 0`：IN1 恒高（CCR=PERIOD），IN2 输出 PWM（CCR = PERIOD - speed），电机**正转**
> - `speed < 0`：IN1 输出 PWM（CCR = PERIOD - |speed|），IN2 恒高（CCR=PERIOD），电机**反转**
> - `speed = 0`：IN1/IN2 均恒高（CCR=PERIOD），电机**刹车**

- PWM 频率：10kHz（TIM2/TIM3, PSC=9-1, ARR=800-1）
- 舵机频率：50Hz（TIM1, PSC=720-1, ARR=2000-1）
- 角度映射：`CCR = 50 + angle/256 × 200`（0.5ms~2.5ms 脉冲）

### 3. 通信协议

通信接口：USART3, 115200bps, 8N1

#### 下行请求帧（上位机 → 下位机）

| 偏移 | 字段 | 类型 | 值 |
|:---:|------|------|------|
| 0 | 帧头 | uint8×2 | `FA AF` |
| 2 | 帧类型 | uint8 | `52` (ASCII 'R') |
| 3 | 帧尾 | uint8×2 | `FB BF` |

收到后触发电流采集并回传上行数据帧。

#### 下行数据帧（上位机 → 下位机）

| 偏移 | 字段 | 类型 | 说明 |
|:---:|------|------|------|
| 0 | 帧头 | uint8×2 | `FA AF` |
| 2 | 帧类型 | uint8 | `49` (ASCII 'I') |
| 3~6 | Y 推力 | float32 LE | 单位 N |
| 7~10 | X 推力 | float32 LE | 单位 N（预留） |
| 11~14 | Z 推力 | float32 LE | 单位 N |
| 15~18 | Yaw 扭矩 | float32 LE | 单位 N·m |
| 19 | 机械臂角度 | uint8 | 0x00~0xFF |
| 20 | 异或校验 | uint8 | XOR(帧类型~角度) |
| 21 | 帧尾 | uint8×2 | `FB BF` |

帧总长：23 字节

#### 上行数据帧（下位机 → 上位机）

| 偏移 | 字段 | 类型 | 说明 |
|:---:|------|------|------|
| 0 | 帧头 | uint8×2 | `FA AF` |
| 2 | 帧类型 | uint8 | `53` (ASCII 'S') |
| 3~6 | 温度 | float32 LE | 单位 °C（预留，填 0） |
| 7 | 进水检测 | uint8 | 0=正常（预留，填 0） |
| 8~11 | 电流 | float32 LE | 单位 A |
| 12 | 异或校验 | uint8 | XOR(帧类型~电流) |
| 13 | 帧尾 | uint8×2 | `FB BF` |

帧总长：15 字节。**请求-响应机制**：下位机仅在收到请求帧（0x52）后发送，不主动推送。

---

## 目录结构

```
Test4_A/
├── Core/
│   ├── Inc/                  # HAL 外设头文件
│   │   ├── main.h
│   │   ├── gpio.h / i2c.h / tim.h / usart.h
│   │   ├── stm32f1xx_hal_conf.h
│   │   └── stm32f1xx_it.h
│   └── Src/                  # HAL 外设源文件
│       ├── main.c            # 主程序、串口协议状态机
│       ├── gpio.c / i2c.c / tim.c / usart.c
│       ├── stm32f1xx_hal_msp.c
│       ├── stm32f1xx_it.c
│       └── system_stm32f1xx.c
├── UserCode/
│   ├── Inc/                  # 用户模块头文件
│   │   ├── ThrustCurve.h     # 推力曲线参数与接口
│   │   ├── DRV8871.h         # 电机驱动接口
│   │   └── INA226.h          # 电流传感器接口
│   └── Src/                  # 用户模块源文件
│       ├── ThrustCurve.c     # 推力曲线正反函数
│       ├── DRV8871.c         # DRV8871 PWM 驱动
│       └── INA226.c          # INA226 I²C 读写
├── Drivers/
│   ├── CMSIS/
│   └── STM32F1xx_HAL_Driver/
├── MDK-ARM/                  # Keil 工程
│   ├── startup_stm32f103xb.s # 启动文件
│   └── Test4_A.uvprojx       # Keil 工程文件
├── Test4_A.ioc               # CubeMX 工程
├── thruster.xlsx             # 推力标定数据
├── LICENSE
└── README.md
```

---

## 快速开始

### 1. 环境准备

- Keil MDK-ARM 5.x
- STM32CubeMX 6.16.0+
- STM32F1xx HAL 库

### 2. 编译烧录

1. 打开 `MDK-ARM/Test4_A.uvprojx`
2. 编译
3. 通过 ST-Link / J-Link / 串口烧录

### 3. 硬件连接

**通信 & 传感器**

| 外设 | 引脚 | 说明 |
|------|------|------|
| USART3 TX | PB10 | 连接上位机 RX |
| USART3 RX | PB11 | 连接上位机 TX |
| I²C1 SCL | PB8 | INA226 |
| I²C1 SDA | PB9 | INA226 |

**推进器电机**

| 外设 | IN1 引脚 | IN2 引脚 | 定时器通道 | 说明 |
|------|:---:|:---:|------|------|
| DRV8871 Motor A | PA0 | PA1 | TIM2 CH1 / CH2 | Y 轴推进 + Yaw 轴偏航差动（左侧） |
| DRV8871 Motor B | PA2 | PA3 | TIM2 CH3 / CH4 | Y 轴推进 + Yaw 轴偏航差动（右侧） |
| DRV8871 Motor C | PB1 | PB0 | TIM3 CH4 / CH3 | Z 轴推进器（后，正向下） |
| DRV8871 Motor D | PA7 | PA6 | TIM3 CH2 / CH1 | Z 轴推进器（前，正向下） |

**舵机**

| 外设 | 信号引脚 | 定时器通道 | 说明 |
|------|:---:|------|------|
| 机械臂舵机 | PA10 | TIM1 CH3 | 50Hz, 0.5~2.5ms 脉冲 |

### 4. 通信测试

发送请求帧获取电流数据：
```
FA AF 52 FB BF
```

发送数据帧控制推进器（Y=10N, 舵机=0x80）：
```
FA AF 49  00 00 20 41  00 00 00 00  00 00 00 00  00 00 00 00  80  0B  FB BF
```

---

## 关键参数配置

如需修改运行参数，请同步调整以下位置：

| 参数 | 文件位置 | 说明 |
|------|------|------|
| 推力曲线系数 | `UserCode/Inc/ThrustCurve.h` | 中性点、死区、正/反向系数 |
| PWM 周期 | `UserCode/Src/DRV8871.c` → `PWM_PERIOD` | 当前 800 |
| TIM 周期 | `Core/Src/tim.c` → `htim2/htim3.Init.Period` | 需与 PWM_PERIOD 一致 |
| TIM 预分频 | `Core/Src/tim.c` → `htim2/htim3.Init.Prescaler` | PWM 频率 = 72MHz / (PSC+1) / (Period+1) |
| 串口波特率 | `Core/Src/usart.c` → `huart3.Init.BaudRate` | 当前 115200 |
| I²C 地址 | `UserCode/Inc/INA226.h` → `INA226_ADDR` | 当前 0x80 |

> **注意**：修改 `PWM_PERIOD` 后必须在 CubeMX 中同步修改 TIM2/TIM3 的 Period，并重新生成代码。

---

## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.1 | 2026-05 | 重新分配推力：A(左)/B(右) Y轴前进+Yaw轴偏航差动(力臂0.1m)，C(后)/D(前) Z轴升降(正向下)，X轴预留 |
| v1.0 | 2026-05 | 初始版本，完成推力曲线拟合、4 路推进器控制、通信协议、电流监测 |

# 四足轮腿底层MCU

四足轮腿机器人 - 腿部伺服电机底层控制（STM32F407），适配 Linux 上位机（ROS2 `hardware_driver`）。

> ⚠️ **开发中：** 主函数尚未完成，当前主要工作在 `HARDWARE/` 目录下的硬件驱动层。

## 开发环境

- **MCU:** STM32F407（Cortex-M4，168MHz）
- **HAL 库:** STM32F4xx HAL Driver
- **IDE:** Keil MDK-ARM
- **配置工具:** STM32CubeMX
- **调试器:** J-Link / ST-Link

## 硬件接口总览

| 外设 | 引脚 / 参数 | 用途 |
|------|------------|------|
| USART1 | PA9=TX / PA10=RX，115200 8N1 | 与 Linux 上位机通信（协议层） |
| USART5 | PC12=TX / PD2=RX，1Mbps 半双工 | STS/SCS 舵机总线 |
| I2C（位带） | — | AS5600 磁编码器（角度读取） |
| TIM2~TIM7 | — | FOC SVPWM、控制定时等 |
| LED1（红） | PC0，低电平点亮 | 收到 `CMD_CHASSIS_CMD` 闪烁 |
| LED2（绿） | PC1，低电平点亮 | 收到 `CMD_LEG_STATE` 闪烁 |
| LED3（蓝） | PC2，低电平点亮 | 收到 `CMD_WHEEL_STATE` 闪烁 |

## 串口协议（与 Linux 上位机对接）

### 帧格式

```
[0xAA][0x55][cmd][len][payload...][crc8]
```

| 字段 | 长度 | 说明 |
|------|------|------|
| `0xAA 0x55` | 2 字节 | 帧头 |
| `cmd` | 1 字节 | 命令号 |
| `len` | 1 字节 | payload 字节数（**不含帧头和 CRC**） |
| `payload` | `len` 字节 | 数据区，浮点数为 IEEE754 小端（little-endian） |
| `crc8` | 1 字节 | CRC-8，多项式 0x07，初值 0x00，不反射 |

**CRC8 说明：**

- 多项式 `0x07`，初值 `0x00`，无输入/输出反射
- 覆盖范围：`cmd + len + payload`（**不含** 2 字节帧头和最后的 CRC 字节）
- 校验向量：`"123456789"` → `0xF4`
- 代码入口：`Protocol_Crc8()`

### 命令定义

| 命令 | 值 | len | 方向 | 内容 |
|------|----|-----|------|------|
| `CMD_CHASSIS_CMD` | 0x01 | 40 | 下行（Linux → MCU） | `foot_x[4]` `foot_z[4]` `left_wheel` `right_wheel` |
| `CMD_LEG_STATE` | 0x02 | 17 | 上行（MCU → Linux） | `leg_id` `hip` `knee` `foot_x` `foot_z` |
| `CMD_WHEEL_STATE` | 0x03 | 13 | 上行（MCU → Linux） | `wheel_id` `velocity` `position` `torque` |
| `CMD_IMU_DATA` | 0x04 | 12 | 上行（MCU → Linux） | `roll` `pitch` `yaw` |

### 各命令 payload 字节排布

所有多字节字段均为 **小端（little-endian）**，浮点为 IEEE754 `float`（4 字节）。

**CMD_CHASSIS_CMD（下行，40 字节）：**

| 偏移 | 字段 | 类型 |
|------|------|------|
| 0~3 | `foot_x[0]` | float |
| 4~7 | `foot_x[1]` | float |
| 8~11 | `foot_x[2]` | float |
| 12~15 | `foot_x[3]` | float |
| 16~19 | `foot_z[0]` | float |
| 20~23 | `foot_z[1]` | float |
| 24~27 | `foot_z[2]` | float |
| 28~31 | `foot_z[3]` | float |
| 32~35 | `left_wheel` | float（rad/s） |
| 36~39 | `right_wheel` | float（rad/s） |

**CMD_LEG_STATE（上行，17 字节）：**

| 偏移 | 字段 | 类型 |
|------|------|------|
| 0 | `leg_id` | uint8 |
| 1~4 | `hip_angle` | float |
| 5~8 | `knee_angle` | float |
| 9~12 | `foot_x` | float |
| 13~16 | `foot_z` | float |

**CMD_WHEEL_STATE（上行，13 字节）：**

| 偏移 | 字段 | 类型 |
|------|------|------|
| 0 | `wheel_id` | uint8 |
| 1~4 | `velocity` | float |
| 5~8 | `position` | float |
| 9~12 | `torque` | float |

**CMD_IMU_DATA（上行，12 字节）：**

| 偏移 | 字段 | 类型 |
|------|------|------|
| 0~3 | `roll` | float |
| 4~7 | `pitch` | float |
| 8~11 | `yaw` | float |

### 收发方式（均非阻塞）

| 方向 | 机制 | 说明 |
|------|------|------|
| 接收 | 中断，逐字节 | `HAL_UART_Receive_IT` 每次收 1 字节，`HAL_UART_RxCpltCallback` 里喂入 RX 状态机后重新启动接收 |
| 发送 | 中断，非阻塞 | 帧组进静态缓冲 `tx_frame`，`HAL_UART_Transmit_IT` 后台发送 |

**发送的非阻塞实现（protocol.c）：**

1. 静态缓冲 `tx_frame[45]` + 忙标志 `tx_busy`
2. 组帧时若 `tx_busy` 已置位，则**丢帧返回**（10Hz 上行速率下不会发生）
3. 否则置 `tx_busy = 1`，调用 `HAL_UART_Transmit_IT` 开始后台发送
4. 发送完成触发 `HAL_UART_TxCpltCallback`，清 `tx_busy = 0`

> 主循环不会因发送而阻塞；以前 `HAL_UART_Transmit` 会卡住主循环等发完，现已消除。

### 数据频率

- **上行（MCU → Linux）：** 每 100ms（10 Hz）发一次状态，共 3 帧：

  | 帧 | 字节数（头2 + cmd1 + len1 + payload + crc1） |
  |----|------|
  | `CMD_LEG_STATE` | 22 |
  | `CMD_WHEEL_STATE` | 18 |
  | `CMD_IMU_DATA` | 17 |
  | **合计** | **57 字节/拍** |

  速率 = 57 × 10 = **约 570 字节/秒**。115200 波特率、8N1（10 bit/字节）理论吞吐 ≈ 11520 字节/秒，上行仅占 **~5% 带宽**。

- **下行（Linux → MCU）：** `CMD_CHASSIS_CMD` 每帧固定 **45 字节**，速率 = 45 × Linux 控制循环频率（由 `hardware_driver` 的 write 周期决定）。

- **空中传输时间（单帧）：**

  | 帧 | 字节 | 传输时间 @115200 |
  |----|------|------|
  | 下行 `CMD_CHASSIS_CMD` | 45 | 约 3.9ms |
  | 上行 3 帧合计 | 57 | 约 4.9ms |

## LED 指示灯

收到 **CRC 校验通过** 的帧后，不同数据类型对应不同灯闪烁（低电平点亮，每收到一帧翻转一次）：

| 灯 | 引脚 | 触发命令 |
|----|------|----------|
| 灯 1（红） | PC0 | 收到 `CMD_CHASSIS_CMD`（0x01） |
| 灯 2（绿） | PC1 | 收到 `CMD_LEG_STATE`（0x02） |
| 灯 3（蓝） | PC2 | 收到 `CMD_WHEEL_STATE`（0x03） |
| 灯 2 + 灯 3 | PC1 + PC2 | 收到 `CMD_IMU_DATA`（0x04） |
| 灯 1（红） | PC0 | 其它命令（默认） |

实现：`col_update()` 里检测 `g_rx_valid_count` 变化，按 `g_rx_last_cmd` 用 `LedToggle()` 翻转对应灯。

**回环测试：** 将 USART1 的 RX 与 TX 短接，上位机（或单片机自身）发送任一帧，若协议解析正确（CRC 通过）则对应灯闪烁。

## 控制逻辑

### 主循环调度（`col_update()`）

| 任务 | 周期 | 说明 |
|------|------|------|
| 舵机使能扫描 | 每 50ms 一个舵机 | 逐个 `ping` + `enable_torque`，非阻塞 |
| 轮子速度环（FOC） | 8ms | 应用新底盘命令 + 4 路速度环 |
| 状态上报 | 100ms | 发 3 帧上行状态 |
| LED 刷新 | 事件触发 | 收到新帧时翻转对应灯 |

### 底盘命令应用（`ApplyChassisCmd()`）

1. 收到 `CMD_CHASSIS_CMD` 且 CRC 通过后，置 `g_chassis_cmd_new = 1`
2. 速度环 tick 内检测到该标志，对每条腿做逆运动学（IK）解算
3. `foot_x/foot_z` → `hip/knee` 角度，用 `SyncWritePosEx()` 广播写入舵机（速度 2400，加速度 50）
4. `left_wheel` → 电机 1、2；`right_wheel` → 电机 3、4

### 舵机 / 电机 ID 映射

**腿部舵机（每条腿 2 个，共 8 个）：**

| 腿 | knee | hip |
|----|------|-----|
| 0 | 1 | 2 |
| 1 | 3 | 4 |
| 2 | 5 | 6 |
| 3 | 7 | 8 |

> ⚠️ 请对照实际接线核对。

**轮毂电机（4 个）：**

| 方向 | 电机 |
|------|------|
| 左 | 1、2 |
| 右 | 3、4 |

> ⚠️ 请对照实际接线核对。

## 启动流程（非阻塞）

1. `Protocol_Init()` —— 先开串口，立即启动 USART1 中断接收（不阻塞）
2. `sts_init()` —— 舵机总线初始化（不阻塞）
3. `FOC_Init(1..4)` —— 电机 FOC 对齐（阻塞，约 1.3s/电机，共约 5s）
4. 舵机的 `ping` + 上电使能（torque enable）推迟到 `col_update()` 里逐个扫描，不阻塞启动

> 舵机总线超时已由 100ms 缩短为 5ms，无舵机在线时也不会长时间卡死主循环。

## 舵机/电机 ID 映射测试

在 `col.c` 中提供了两个一次性测试函数（在 `main.c` 的 `col_init()` 之后调用）：

```c
col_test_servo_map();   /* ping 舵机 1~8，串口打印哪些在线 */
col_test_motor_map();   /* 电机 1~4 逐个开环转动约 1.5s，观察是哪个轮子转 */
```

结果通过 USART1（`HC05_Printf`）打印，串口助手可看到：

```
--- servo id map test ---
servo 1: OK (reply 1)
servo 2: no reply
...
--- motor id map test ---
motor 1: running...
motor 1: stopped
...
```

用法：在 `main.c` 的 `USER CODE BEGIN 2` 中取消注释对应调用即可。测试时请先断开与上位机的正常通信（或确认 USART1 未被占用）。

## 目录结构

```
├── Core/           # STM32CubeMX 生成的 HAL 内核代码
├── Drivers/        # CMSIS + STM32F4 HAL 驱动库
├── HARDWARE/       # 硬件驱动层（主要开发内容 👈）
├── MDK-ARM/        # Keil MDK 工程文件
├── GPIO.ioc        # STM32CubeMX 项目配置
└── README.md
```

## HARDWARE/ — 硬件驱动

| 模块 | 文件 | 功能 |
|------|------|------|
| **AS5600** | `AS5600.c/h` | 磁编码器驱动（I2C，角度读取） |
| **FOC** | `foc.c/h` | FOC 磁场定向控制算法（SVPWM、Clarke/Park、PI） |
| **Servo** | `servo.c/h` | STS/SCS 舵机控制（USART5） |
| **IK** | `IK.c/h` | 逆运动学解算（2 连杆） |
| **Col** | `col.c/h` | 主要控制逻辑、LED 反馈、ID 映射测试 |
| **Protocol** | `protocol.c/h` | 串口帧协议（USART1，非阻塞收发） |
| **PID** | `pid.c/h` | PID 控制器 |
| **HC05** | `hc05.c/h` | 蓝牙透传模块驱动（USART1 调试打印） |
| **Key** | `key.c/h` | 按键输入 |
| **LED** | `led.c/h` | LED 指示灯 |

## 待完成

- [ ] 主函数 `main.c` 逻辑整合
- [ ] 多传感器数据融合
- [ ] 步态控制算法
- [ ] IMU 接入（当前上行 IMU 数据为 0）
- [ ] 整体联调测试

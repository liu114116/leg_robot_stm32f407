# 四足轮腿底层MCU

四足轮腿机器人 - 腿部伺服电机底层控制（STM32F407）

> ⚠️ **开发中：** 主函数尚未完成，当前主要工作在 `HARDWARE/` 目录下的硬件驱动层。

## 开发环境

- **MCU:** STM32F407（Cortex-M4）
- **HAL 库:** STM32F4xx HAL Driver
- **IDE:** Keil MDK-ARM
- **配置工具:** STM32CubeMX
- **调试器:** J-Link / ST-Link

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
| **FOC** | `foc.c/h` | FOC 磁场定向控制算法 |
| **Servo** | `servo.c/h` | 舵机控制 |
| **IK** | `IK.c/h` | 逆运动学解算 |
| **Col** | `col.c/h` | 碰撞检测 |
| **PID** | `pid.c/h` | PID 控制器 |
| **HC05** | `hc05.c/h` | 蓝牙透传模块驱动（USART） |
| **Key** | `key.c/h` | 按键输入 |
| **LED** | `led.c/h` | LED 指示灯 |

## 待完成

- [ ] 主函数 `main.c` 逻辑整合
- [ ] 多传感器数据融合
- [ ] 步态控制算法
- [ ] 整体联调测试

# 四足轮腿底层MCU

四足轮腿机器人 - 腿部伺服电机底层控制（STM32F407）

## 开发环境

- MCU: STM32F407
- HAL 库
- IDE: Keil MDK-ARM
- 配置工具: STM32CubeMX

## 目录结构

```
├── Core/           # STM32CubeMX 生成的 HAL 内核代码
├── Drivers/        # HAL 驱动库
├── HARDWARE/       # 硬件抽象层
├── MDK-ARM/        # Keil MDK 工程文件
└── GPIO.ioc        # STM32CubeMX 项目配置
```

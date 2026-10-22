# VehMotionHub

车载 **IMU（SCHA63X）** 与 **底盘 CAN** 运动数据汇聚节点。MCU 采集六轴 IMU、解析选定 CAN 帧（轮速、车速、挡位、方向盘角等），经 USART1 以类 NMEA 文本协议输出，供上位机或惯导组合使用。

| 项目名 | MCU | RTOS |
|--------|-----|------|
| **VehMotionHub** | STM32H743IITx | FreeRTOS (CMSIS-RTOS v1) |

---

## 功能概览

- **IMU**：SPI 驱动 Murata **SCHA63X**，十字轴补偿，约 **100 Hz** 输出 `$GRIMU` 句子（TIM16 触发）。
- **CAN**：FDCAN1 接收多路底盘报文，凑齐更新后输出 `$GRODO` 句子。
- **授时**：USART3 接收含日期/时间的 NMEA 类字符串，同步 **RTC**（北京时间存 RTC，时间戳按 UTC 输出）；原样转发到 **USART2**。
- **指示**：PB1 GPIO 约 1 Hz 翻转，表示程序运行。
- **日志**：USART1 **460800** 波特率，线程安全 `App_LogPrintf`。

---

## 硬件接口（摘要）

| 外设 | 用途 | 说明 |
|------|------|------|
| SPI2 + PG10/PG12 | SCHA63X UNO/DUE | IMU |
| FDCAN1 | 车辆 CAN | 标准帧 ID 见下 |
| USART1 PA9/10 | 主输出 | `$GRIMU` / `$GRODO` |
| USART2 PA2/3 | 透传 | USART3 整帧转发 |
| USART3 PB10/11 | 输入 | 时间/NMEA，中断接收 |
| RTC + LSE | 时钟 | 可选 GNSS/主机授时 |
| PB1 | LED/心跳 | 运行指示 |

具体引脚以 `VehMotionHub.ioc` / CubeMX 为准。

---

## 串口协议

校验为 `$` 与 `*` 之间字节的 XOR，以 **2 位十六进制** 输出（无 `0x` 前缀）。

### `$GRIMU`（IMU，约 100 Hz）

字段顺序：

`时间戳, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, temp_due`

示例形态：`$GRIMU,1739123456.010,0.001,...*XX`

### `$GRODO`（底盘，CAN 集齐后输出）

字段顺序：

`时间戳, 左前轮速, 右前轮速, 左后轮速, 右后轮速, 纵向车速, 挡位`

`时间戳` 与最近一次 `$GRIMU` 使用同一时间基准（`time_ms`）。

---

## CAN 标准 ID（当前固件）

| ID | 信号（DBC 名参考） | 数据说明 |
|----|-------------------|----------|
| 0x0E0 | VehSpdLgtSafe | `payload[5]` × 0.00391 → 车速 |
| 0x1A0 | 前轮轮速 | byte1 / byte5 → 左前 / 右前 |
| 0x1C0 | 后轮轮速 | byte3 / byte7 → 左后 / 右后 |
| 0x040 | SteerWhlAgSafe | `payload[1]` → 方向盘角（未写入 `$GRODO`） |
| 0x120 | GearLvrIndcn | `payload[0]` → 挡位 |

字节与比例需与实车 DBC 一致；修改见 `Core/Src/main.c` 中 `vehicle_can_handle_frame()`。

---

## 软件架构

```
┌─────────────┐     信号量      ┌──────────────┐
│ TIM16 ISR   │ ──────────────► │ data_output  │──► SPI IMU → $GRIMU
│ USART3 ISR  │ ──────────────► │ (Normal)     │──► RTC / USART2 转发
└─────────────┘                 └──────────────┘
┌─────────────┐     信号量      ┌──────────────┐
│ FDCAN ISR   │ ──────────────► │ can_com      │──► $GRODO
└─────────────┘                 │ (AboveNormal)│
                                └──────────────┘
                                ┌──────────────┐
                                │ work_state   │──► PB1 心跳
                                │ (BelowNormal)│
                                └──────────────┘
```

- **互斥锁**：`g_telemetryMutex`（时间/IMU 时间轴）、`g_vehicleDataMutex`（CAN 解析结果）、`g_logMutex`（USART1 日志）。
- **初始化**：`MX_FREERTOS_Init()`（`freertos.c`）创建对象与任务；`main()` 在传感器初始化后调用 `osKernelStart()`。
- **HAL 时基**：TIM1（1 ms）；**RTOS 时基**：SysTick。

主要源码：

| 路径 | 说明 |
|------|------|
| `Core/Src/main.c` | 协议组帧、CAN 解析、任务实现 |
| `Core/Src/app_rtos.c` | RTOS 对象与日志 |
| `Core/Src/freertos.c` | 任务创建 |
| `Core/Src/scha63x.c` | IMU 驱动 |
| `Core/Inc/FreeRTOSConfig.h` | 内核配置 |

---

## 构建与烧录

1. 使用 **STM32CubeIDE** 打开工程（Eclipse 工程名：**VehMotionHub**）。
2. 确认 `Core/Src/app_rtos.c` 已加入编译（若 Rebuild 报缺符号，在 Project → Properties → C/C++ Build 中添加该文件）。
3. **Project → Build Project**（Debug 配置）。
4. 生成固件：`Debug/VehMotionHub.elf`（工程名变更后）。
5. 调试配置：**VehMotionHub Debug.launch**（ST-LINK / OpenOCD）。

命令行（在 `Debug` 目录且工具链在 PATH 中）：

```bash
make -j16 all
```

---

## 从旧名 `test2sn` 迁移

工程已重命名为 **VehMotionHub**。若本地目录仍叫 `test2sn`，不影响编译；在 CubeIDE 中导入时请以 **`.project` 中的工程名** 为准。重新用 CubeMX 生成代码时，请打开 **`VehMotionHub.ioc`**。

---

## 许可证

STM32 HAL / FreeRTOS / Cube 生成代码遵循 ST 与 Amazon 相应许可证；应用层代码请参考仓库说明。

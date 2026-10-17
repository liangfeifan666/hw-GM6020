# STM32F407CB — GM6020 正弦跟随（电控组第二次作业）

GM6020 电机分别跟随**正弦速度曲线**与**正弦位置曲线**，CAN 总线控制，带独立看门狗（IWDG）保护与反馈掉线保护。

- 主控：RoboMaster 开发板 C 型（STM32F407IGH6）
- 执行器：DJI GM6020（ID = 1，CAN 1Mbps）
- 控制周期：1 kHz（TIM6 中断）
- 工具链：CMake + Ninja + arm-none-eabi-gcc，VS Code / Ozone 调试

---

## 目录结构

```
STM32F407CB.ioc        CubeMX 工程
CMakeLists.txt         队内模板（CMake 4.x）
startup_stm32f407xx.s  启动文件
STM32F407xx_FLASH.ld   链接脚本
Core/  Drivers/        CubeMX 生成，未改动
Tasks/                 ★ 业务代码全部在这里
  inc/  src/             ControlTask（控制主任务）、Gm6020（电机协议）、Pid（PID 控制器）
```

## 构建与烧录

```bash
cmake --preset debug          # 配置（Tasks/ 改动后如加文件需重新执行）
cmake --build --preset debug  # 构建，产物在 build/
```

调试：Ozone 打开 `STM32F407CB.jdebug`（J-Link）；或 VS Code + Cortex-Debug（OpenOCD）。

> ⚠️ **上电顺序**：先给电机上 24V（绿灯正常闪烁），再给 C 板上电/复位。
> 当前配置 `AutoBusOff = DISABLE`：若 C 板先开始发帧时总线上没人应答，
> 控制器会在数毫秒内进入 Bus-Off 并停止发送，需复位恢复。

---

## 两种模式怎么切换

[Tasks/src/ControlTask.cpp](Tasks/src/ControlTask.cpp) 顶部：

```c
#define CONTROL_MODE  1    /* 1 = 速度单环（任务一）
                              2 = 位置双环（任务二） */
```

改宏 → 重新编译 → 烧录即可。两种模式共用同一套速度内环（含滤波与前馈）。

目标曲线参数（同文件内）：

| 宏 | 含义 | 当前值 |
| --- | --- | --- |
| `VEL_AMP` / `VEL_FREQ` | 速度目标：幅值 / 频率 | ±6.0 rad/s，0.2 Hz（周期 5 s） |
| `POS_AMP` / `POS_FREQ` | 位置目标：幅值 / 频率 | ±π/2 rad（±90°），0.1 Hz（周期 10 s） |

位置目标以**电机上线时停住的位置为原点**（`g_angle0`），上电不会猛甩到绝对零点。

---

## 控制结构

### 任务一：速度单环

```
refVel(t) = 6·sin(2π·0.2·t)
                │
                ▼
        ┌─── PID(KP,KI,KD) ◄─── g_velFilt（反馈速度低通滤波后）
        │       +
        │    摩擦前馈 FF·sign(refVel)   ← 静/动摩擦分级补偿
        ▼
    电流给定 (±16384 ≈ ±3A) ──► GM6020
```

### 任务二：位置双环

```
refPos(t) = angle0 + (π/2)·sin(2π·0.1·t)
                │
                ▼
        位置外环 PID ──► 目标角速度 (±12 rad/s 限幅)
                            │
                            ▼
                    速度内环（同上，含滤波+前馈）──► GM6020
```

### 两个关键的工程处理（调参过程中实测踩出来的）

1. **分级摩擦前馈**：实测挣脱静摩擦需约 1000 指令（≈0.18A），而维持转动只需约 425。
   单靠 PI：增益小了推不动静摩擦（死区），增益大了被噪声/延迟激发振荡（极限环）。
   做法：启动瞬间补 `VEL_FF_STICTION`，转起来后只补 `VEL_FF_RUNNING`
   （按滤波后速度切换，带 0.5/1.5 rad/s 迟滞），PI 只收拾残差。
2. **反馈速度低通**：电机转速由驱动器每 1 ms 数编码器脉冲得出，低速时量化抖动
   可达 ±1.5 rad/s。一阶低通（α=0.1，约 16 Hz 截止）后进 PID 与微分——
   先滤波后微分，避免 KD 放大噪声。

---

## 最终参数

### 速度环（任务一，`CONTROL_MODE 1`）

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `VEL_KP` / `VEL_KI` / `VEL_KD` | 25 / 2 / 1.5 | KD 作阻尼消喘振 |
| `VEL_FF_STICTION` / `VEL_FF_RUNNING` | 800 / 380 | 静摩擦 / 动摩擦前馈（指令单位） |
| `VEL_OUT_MAX/MIN` | ±8000 | 输出限幅 ≈ ±1.5 A |
| `VEL_FILT_ALPHA` | 0.1 | 反馈速度低通系数 |

### 位置环（任务二，`CONTROL_MODE 2`）

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `POS_KP` / `POS_KI` / `POS_KD` | 5 / 0.5 / 0 | 外环，输出目标速度 |
| `POS_OUT_MAX/MIN` | ±12 | 目标速度限幅 (rad/s) |
| 速度内环 | 同上表 | 位置证据采集时内环为 KP=25, KI=0, KD=0, FF=850/450 |

位置环"一遍过"的原因：位置是速度的积分（天然低通），角度反馈直接读编码器
（8192 分辨率，无量化噪声问题），且目标峰值速度仅约 1 rad/s——
脏活全部由已整定的内环完成。

---

## 调试观测变量（Ozone Watched Data / Timeline）

| 变量 | 含义 |
| --- | --- |
| `g_ref` / `g_fdb` | 本拍目标 / 本拍反馈（同量纲，画在同一 Timeline 即"跟随曲线"） |
| `g_innerRef` | 位置模式专用：外环输出的目标速度，定位"哪一环没跟上" |
| `g_voltage` | 本拍输出的电流给定，看是否长期贴限幅 |
| `g_lostCnt` | 连续多少拍（ms）没收到反馈；稳定小数字 = 通信健康 |
| `hcan1.Instance->ESR` | CAN 错误状态寄存器（TEC/REC/LEC/BOFF 位） |

Timeline 采样建议：5 s/Div，录 ≥3 个完整周期。

---

## 保护机制

- **IWDG**：超时约 0.8 s，由 1 kHz 控制中断喂狗；调试时 `__HAL_DBGMCU_FREEZE_IWDG()`
  冻结，断点暂停不会触发复位，正常运行时保护不削弱。
- **反馈掉线保护**：连续 50 ms 收不到反馈 → 判掉线 → 输出归零 + 清 PID 积分，
  防止总线恢复瞬间顶着满幅指令冲出。
- **输出限幅 + 积分回退抗饱和**：见 `Pid::calc()`。

---

## 已知事项

1. **电机当前处于电流控制模式**：本工程发送的是电流指令帧 `0x1FE`（给定范围
   ±16384 ≈ ±3A）。若电机经 RoboMaster Assistant 关闭电流环（恢复电压模式），
   需把 `Gm6020::txId()` 改回 `0x1FF`、限幅改回 ±25000，并重新整定增益
   （代码注释中有还原点标记）。
2. GM6020 反馈帧 `0x204+ID`，1 kHz 无条件广播；CAN 滤波器放行 `0x200~0x2FF`。
3. 拨码开关：1~3 位 = ID（ON 为 1），第 4 位 = CAN 终端电阻（本电机 ID=1、终端 ON）。

/*
 * ControlTask.cpp —— 第二次作业（第一部分考核）
 * GM6020 分别跟随正弦速度曲线 / 正弦位置曲线，并用独立看门狗保护。
 *
 * 控制链路：初始化 → 1kHz 定时中断 → 算目标 → PID → 编码 → CAN 发送
 * CubeMX 生成的 Core/ 不改，只在 main.c 的 USER CODE 段调一次 ControlTaskInit()。
 */

#include "ControlTask.h"

#include "main.h"
#include "can.h"        /* hcan1 */
#include "tim.h"        /* htim6 */
#include "iwdg.h"       /* hiwdg */
#include "Gm6020.h"
#include "Pid.h"

#include <cmath>        /* sinf */

/* ============================ 可调参数 ============================ */

/* 1 = 速度跟随正弦（速度单环）
 * 2 = 位置跟随正弦（位置外环 + 速度内环，双环）
 * 改宏重新编译下载即可，和第一次作业的 HW_TASK 一个套路。 */
#define CONTROL_MODE  1

#if (CONTROL_MODE != 1) && (CONTROL_MODE != 2)
#error "CONTROL_MODE 只能取 1（速度）或 2（位置）"
#endif

#define MOTOR_ID  1          /* 电机拨码开关设置的 ID，1~7 */

/* 目标曲线。GM6020 空载调速范围 0~320 rpm ≈ 33.5 rad/s，幅值先取小 */
#define VEL_AMP   6.0f       /* 速度幅值 (rad/s) */
#define VEL_FREQ  0.2f       /* 速度频率 (Hz)，周期 5 s */
#define POS_AMP   1.5708f    /* 位置幅值 (rad)，π/2 ≈ 90° */
#define POS_FREQ  0.1f       /* 位置频率 (Hz)，周期 10 s */

/*
 * 速度环：输入 rad/s，输出 = 电流给定（±16384 ≈ ±3A；电机当前是电流模式，
 * 若以后改回电压模式，量纲变回 ±25000，增益和限幅都要重新取）
 * 位置环：输入 rad，  输出 = 速度环的目标 (rad/s) */
#define VEL_KP   25.0f
#define VEL_KI    2.0f
#define VEL_KD    1.50f
/* 摩擦前馈（分级）：纯 PI 只能靠"误差×增益"攒出推力——增益小了推不动、
 * 大了又振。前馈把已知的摩擦力按目标方向直接送出，PI 只管残差。
 * ★ 静摩擦(破土)≈1000 与 动摩擦(维持)≈425 相差一倍多，必须分开补：
 *   启动瞬间给 VEL_FF_STiction，转起来以后只补 VEL_FF_Running，
 *   否则"过零不卡"和"幅度精准"永远不可兼得。 */
#define VEL_FF_STICTION  800.0f   /* 静摩擦前馈：|fdb|<阈值时生效 */
#define VEL_FF_RUNNING   380.0f   /* 动摩擦前馈：转起来后生效 */
#define VEL_OUT_MAX   8000.0f    /* ±8000 ≈ ±1.5A，空载足够转 */
#define VEL_OUT_MIN  -8000.0f

/* 反馈速度低通滤波系数：filt += α×(raw − filt)。
 * α=0.1 @1kHz ≈ 截止 16Hz；目标 0.2Hz，相位损失可忽略。
 * 太小→平滑但滞后；太大→滤波白搭。整定区间 0.05~0.2 */
#define VEL_FILT_ALPHA   0.1f

#define POS_KP    5.0f
#define POS_KI    0.5f
#define POS_KD    0.0f
#define POS_OUT_MAX   12.0f
#define POS_OUT_MIN  -12.0f

/* 反馈掉线判定：手册说 GM6020 反馈是 1kHz，连续 50 拍(50ms) 收不到就判掉线。
 * 为什么必须有这个：CAN 线松了 / 电机掉电的时候，g_motor 里的数据会冻结在
 * 最后一帧，PID 拿着过期反馈继续积分，输出电压会一直顶到限幅；等总线恢复，
 * 电机会拿着这个满幅电压猛地冲一下。掉线后立刻停输出 + 清积分就没有这个问题。
 * ★ 注意这跟 IWDG 是两码事：IWDG 管的是"主控程序卡死"，管不了"外部总线断"。 */
#define RX_TIMEOUT_TICKS  50

/* ============================ 内部状态 ============================ */

static const float kTwoPi      = 6.28318530718f;
static const float kSampleTime = 0.001f;    /* TIM6 周期，1kHz */

static Gm6020 g_motor(MOTOR_ID);
static Pid    g_pidVel(VEL_KP, VEL_KI, VEL_KD, VEL_OUT_MAX, VEL_OUT_MIN);
static Pid    g_pidPos(POS_KP, POS_KI, POS_KD, POS_OUT_MAX, POS_OUT_MIN);

static volatile uint32_t g_tick      = 0;     /* 1kHz 计数 */
static volatile bool     g_online    = false; /* 反馈在线（最近收到过且没超时） */
static volatile uint32_t g_lastRxTick = 0;    /* 最近一次收到反馈时的 g_tick */
static volatile float    g_angle0    = 0.0f;  /* 上线时的反馈角度，位置正弦以此为原点 */
static float g_phaseVel = 0.0f;
static float g_phasePos = 0.0f;
static float g_velFilt  = 0.0f;   /* 低通后的反馈速度，进 PID 也进 g_fdb */
static bool  g_ffLow    = true;   /* 前馈档位：true=静摩擦档（迟滞切换） */

/* ===================== 调试观测变量（给 Ozone 看，不参与控制） =====================
 * 把每拍算出来的中间结果落到内存里，好处是能用 Ozone 的 Watch / Data Sampling
 * 窗口直接把"目标"和"实际"画在同一张图上 —— 这就是作业要的"跟随正弦曲线"证据，
 * 比盯着变量跳数字直观得多。
 * 故意不加 static：Ozone 的 Global Data 窗口按符号名搜索，外部链接的符号最好找。 */
volatile float    g_ref      = 0.0f;   /* 本拍目标：速度模式 = 角速度(rad/s)，位置模式 = 角度(rad) */
volatile float    g_fdb      = 0.0f;   /* 本拍反馈：与 g_ref 同量纲，两线画一起 */
volatile float    g_innerRef = 0.0f;   /* 位置模式专用：位置环输出的速度目标(rad/s)，看内环用 */
volatile float    g_voltage  = 0.0f;   /* 本拍输出电压给定，判断有没有长期贴着限幅 */
volatile uint32_t g_lostCnt  = 0;      /* 连续多少拍没收到反馈 */

/* ============================ 内部函数 ============================ */

/* 把指令裁到电机允许范围，防止算飞了直接怼满。
 * 电流模式（0x1FE）：给定范围 ±16384 对应 ±3A 转矩电流 */
static int16_t toVoltage(float u)
{
  if (u >  16384.0f) return  16384;
  if (u < -16384.0f) return -16384;
  return (int16_t)u;
}

/* 发一帧控制报文。GM6020 一条帧最多带 4 个电机，
 * Gm6020::encode() 只填本电机那 2 字节，其余留 0。 */
static void sendControlFrame(void)
{
  CAN_TxHeaderTypeDef tx = {0};
  uint8_t  data[8] = {0};
  uint32_t mailbox;

  tx.StdId = g_motor.txId();     /* ID<=4 → 0x1FF，ID>=5 → 0x2FF */
  tx.IDE   = CAN_ID_STD;
  tx.RTR   = CAN_RTR_DATA;
  tx.DLC   = 8;                  /* 必须凑满 8 字节 */
  tx.TransmitGlobalTime = DISABLE;

  g_motor.encode(data);

  /* 在中断里跑，不能用阻塞发送；邮箱满就直接跳过这一拍 */
  (void)HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox);
}

/* 速度内环（含分级摩擦前馈）：速度模式直接用，位置模式的外环输出也喂给它。
 * 抽成函数就是为了两个模式共用同一套已整定的内环——
 * 位置环的稳定性几乎全押在内环身上，绝不能让它用"裸 PID"。 */
static int16_t velControl(float refVel)
{
  float u = g_pidVel.calc(refVel, g_velFilt);
  /* 分级摩擦前馈：目标给方向，滤波速度选静/动摩擦档。
   * 切换带迟滞（>1.5 进 RUNNING，<0.5 回 STICTION），
   * 防止量化噪声把档位打得来回跳。±0.05 死区防止过零点方向抖动 */
  if (g_velFilt >  1.5f) g_ffLow = false;
  if (g_velFilt <  0.5f &&
      g_velFilt > -0.5f) g_ffLow = true;
  if (g_velFilt < -1.5f) g_ffLow = false;
  const float ff = g_ffLow ? VEL_FF_STICTION : VEL_FF_RUNNING;
  if (refVel > 0.05f)       u += ff;
  else if (refVel < -0.05f) u -= ff;
  return toVoltage(u);
}

/* ============================ 对外接口 ============================ */

/* 上电初始化：配 CAN 滤波器、启动 CAN、启动 1kHz 定时器 */
void ControlTaskInit(void)
{
  /* 调试时冻结 IWDG：置 DBGMCU_APB1_FZ 的 DBG_IWDG_STOP 位。
   * 调试器把内核停下来时 IWDG 也暂停计数，这样在 Ozone 里打断点/停住看变量
   * 就不会 0.8 秒后被看门狗复位（AGENTS.md 坑 #10）。
   * ★ 只影响「内核被调试器 halt 住」这一种情况；正常运行时看门狗照常计数，
   *   保护能力一点没削弱。 */
  __HAL_DBGMCU_FREEZE_IWDG();

  /* 滤波器：只放行高 3 位为 010 的帧（0x200~0x2FF）。
   * GM6020 反馈帧 = 0x204 + ID，落在这个范围里；其余 ID 不进 FIFO。 */
  CAN_FilterTypeDef filter = {0};
  filter.FilterIdHigh         = (0x200u << 5);   /* 11 位 ID 左对齐到 bit15:5 */
  filter.FilterMaskIdHigh     = (0x700u << 5);   /* 掩码位为 1 表示必须匹配 */
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterBank           = 0;
  filter.FilterMode           = CAN_FILTERMODE_IDMASK;
  filter.FilterScale          = CAN_FILTERSCALE_32BIT;
  filter.FilterActivation     = ENABLE;
  filter.SlaveStartFilterBank = 14;              /* 双 CAN 时 CAN2 从 14 号起 */
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) Error_Handler();

  if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
  if (HAL_CAN_ActivateNotification(
          &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();

  /* TIM6：84MHz / 84 / 1000 = 1kHz */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();

  g_tick = 0;
  g_online = false;
  g_lastRxTick = 0;
  g_lostCnt = 0;
  g_phaseVel = 0.0f;
  g_phasePos = 0.0f;
  g_ref = 0.0f;
  g_fdb = 0.0f;
  g_innerRef = 0.0f;
  g_voltage = 0.0f;
  g_pidVel.reset();
  g_pidPos.reset();
  g_motor.setVoltage(0);
}

/* 1kHz 控制主任务，由 TIM6 周期中断调用 */
void MainTask(void)
{
  g_tick = g_tick + 1;   /* C++20 里 volatile 的 ++ 已弃用，写成赋值 */

  /* 喂狗。IWDG 超时 = (400+1)×64/32000 ≈ 0.80 s（见 iwdg.c）：
   * 只要这个 1kHz 中断还在跑就不会复位；中断一旦停摆，0.8s 后芯片自动复位。 */
  HAL_IWDG_Refresh(&hiwdg);

  /* 反馈掉线检测。g_tick / g_lastRxTick 都是 uint32_t，减法天然处理回绕。 */
  g_lostCnt = g_tick - g_lastRxTick;
  if (g_online && (g_lostCnt > RX_TIMEOUT_TICKS)) {
    g_online = false;     /* 判掉线：下面 PID 那段不执行，输出回到 0 */
    g_pidVel.reset();     /* 清积分，避免总线恢复瞬间顶着满幅冲出去 */
    g_pidPos.reset();
  }

  /* 算目标曲线。用相位累加而不是 t×频率，避免浮点长时间累加漂移。 */
#if CONTROL_MODE == 1
  g_phaseVel += kTwoPi * VEL_FREQ * kSampleTime;
  if (g_phaseVel >= kTwoPi) g_phaseVel -= kTwoPi;
  const float refVel = VEL_AMP * sinf(g_phaseVel);
#else
  g_phasePos += kTwoPi * POS_FREQ * kSampleTime;
  if (g_phasePos >= kTwoPi) g_phasePos -= kTwoPi;
  const float refPos = g_angle0 + POS_AMP * sinf(g_phasePos);
#endif

  int16_t voltage = 0;

  /* 反馈速度一阶低通：电机转速是驱动器每 1ms 数编码器脉冲算的，
   * 低速时量化抖动可达 ±1.5 rad/s。目标才 0.2Hz，滤到 ~16Hz 几乎零代价。
   * 滤波值既进 PID 也进 g_fdb 观测——图上看到的就是真正参与控制的量。 */
  g_velFilt += VEL_FILT_ALPHA * (g_motor.vel() - g_velFilt);

  /* 没上线（还没收到反馈 / 反馈超时）就先输出 0，等 CAN 通了再开始跟随 */
  if (g_online) {
#if CONTROL_MODE == 1
    /* 速度单环：目标角速度(rad/s) → 电流给定（电流模式） */
    voltage = velControl(refVel);
#else
    /* 位置双环：外环把角度误差算成目标角速度，交给已整定的速度内环。
     * 顺序就是纪要强调的：先调速度环，稳了再套角度环。 */
    const float refVelFromPos = g_pidPos.calc(refPos, g_motor.angle());
    voltage = velControl(refVelFromPos);
    g_innerRef = refVelFromPos;
#endif
  }

  /* 观测变量：不管在线与否都刷新。掉线时能看到"目标还在跑、反馈是一条平线"，
   * 一眼就能区分"没跟"和"跟丢"，比看数字猜强。 */
#if CONTROL_MODE == 1
  g_ref = refVel;
  g_fdb = g_velFilt;   /* 画滤波后的速度：干净，且就是 PID 实际看到的反馈 */
#else
  g_ref = refPos;
  g_fdb = g_motor.angle();
#endif
  g_voltage = static_cast<float>(voltage);

  g_motor.setVoltage(voltage);
  sendControlFrame();
}

/* CAN 接收回调：只解码 + 记状态，不做 PID、不发报文 */
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data)
{
  if (std_id != g_motor.rxId()) return;   /* 不是本电机的反馈（0x204 + ID） */

  g_motor.decode(data);
  g_lastRxTick = g_tick;   /* 刷新掉线计时 */

  /* 上线（以及掉线后重新上线）都重新取原点：电机停在哪就以哪为 0 点，
   * 这样切到位置模式时不会猛地往绝对零点甩，中途掉线也不会把参考点甩到别处。 */
  if (!g_online) {
    g_angle0 = g_motor.angle();
    g_online = true;
  }
}

/* ============================ HAL 回调 ============================ */

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) {
    MainTask();
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];

  if (hcan->Instance != CAN1) return;
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, data) != HAL_OK) return;
  if (rx_header.IDE != CAN_ID_STD) return;

  CanFeedbackCallback(rx_header.StdId, data);
}

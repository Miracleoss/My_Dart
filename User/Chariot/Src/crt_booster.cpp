/**
 * @file crt_booster.cpp
 * @author cjw
 * @brief 发射机构
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"

/* Private macros ------------------------------------------------------------*/

int time_test_pushing = 0;

//测试用
// int aaa_3 = 0;

float GM6020_angle_RELOAD[4] = {118.0f * PI / 180.0f, 240.5f * PI / 180.0f, 360.0f * PI / 180.0f, 479.0f * PI / 180.0f};
float GM6020_angle_ELUDE[4] = {178.0f * PI / 180.0f, 300.0f * PI / 180.0f, 415.0f * PI / 180.0f, 539.0f * PI / 180.0f};

int aasasa = 0;

float pull_test = 0.25f;

// float GM6020_angle_1_ELUDE = 0.0f;
// float GM6020_angle_2_ELUDE = 0.0f;
// float GM6020_angle_3_ELUDE = 0.0f;
// float GM6020_angle_4_ELUDE = 0.0f;

// float GM6020_angle_1_RELOAD = 0.0f;
// float GM6020_angle_2_RELOAD = 0.0f;
// float GM6020_angle_3_RELOAD = 0.0f;
// float GM6020_angle_4_RELOAD = 0.0f;


int test123 = 0;
int PB3_GPIO = 0;
int PD7_GPIO = 0;
int PA5_GPIO = 0;
int PE15_GPIO = 0;

// PB3 中断锁存：按下一次即记住，直到状态机消费
static volatile bool pb3_press_event_latched = false;
volatile uint32_t pb3_exti_irq_count = 0;
volatile uint32_t pb3_event_consumed_count = 0;

// PD7 中断锁存：按下一次即记住，直到状态机消费（Push 前端检测）
static volatile bool pd7_press_event_latched = false;
volatile uint32_t pd7_exti_irq_count = 0;
volatile uint32_t pd7_event_consumed_count = 0;

// PA5 中断锁存：拉力电机向上微动开关
static volatile bool pa5_press_event_latched = false;
volatile uint32_t pa5_exti_irq_count = 0;
volatile uint32_t pa5_event_consumed_count = 0;

bool push_ready_or_switch = 0;

//换弹次数
int reload_count = 0;
// 换弹完成后发给 Shooting 的一次性放行 token（仅可消费一次）
static uint32_t reload_done_token = 0;
static uint32_t last_consumed_reload_done_token = 0;

float test_Motor_Reload_Linear_Target = 0.5f; // 换弹直线电机测试目标位置
float test_reload_servo_angle = 220.0f; // 舵机测试目标角度

//push电机target能够容忍的误差
float push_target_tolerance = 0.006f;

// 校准是否完成相关标志位
bool Push_Calibration_Finished = false;
bool Pull_Calibration_Finished = false;

// 是否允许发射相关标志位
// bool Loading_Slider_Ready; // 上膛滑块机构就位
bool Referee_Allow_Shoot = false; // 裁判系统允许发射
int test_allow_fire = 0; // 测试用，允许发射标志位

// 上位机离线保底发射备案
static bool minipc_fallback_prev = false;

// 已发镖数量
// static int dart_fired_count = 0;
// static int last_dart_fired_count = 0;

int dart_fired_count = 0;
int last_dart_fired_count = 0;

// READY_PRE：push到位事件时间戳（-1 表示尚未到位）
// static int ready_pre_push_reached_time = -1;
int ready_pre_push_reached_time = -1;

// 拉力误差连续满足阈值的累计时间（单位：ms）
uint16_t tension_in_range_time_ms = 0;
// Pull 位置环稳定计时（单位：ms）
static uint16_t pull_pos_stable_ms = 0;

// int servo_test;
int servo_test_flag = 0;
float test_0_1_push = 0.03f;
float test_0_1_pull = 0.9f;
float test_0_1_reload_linear = 0.0f;

// Pull 校准完成后在校准状态机内部锁位
float pull_hold_after_calib_pos = 0.9f;

// 在换弹机构中舵机延时相关变量（临时） 后续可能会换成总线舵机
int reload_servo_flag_drop = 0;
int reload_servo_flag_lift = 0;
int reload_servo_drop_time = 0;
int reload_servo_lift_time = 0;

/*解决 允许发射信号 边沿检测问题 换弹 HOLD*/
static bool referee_allow_prev = false;
static uint32_t referee_allow_rise_cnt = 0;
static constexpr uint32_t REFEREE_ALLOW_SHOOT_MAX_COUNT = 4;
// 发完第 2 发后暂停 30s，再允许继续发第 3、4 发
static constexpr uint8_t DART_PAUSE_AFTER_COUNT = 2;//定义发完第2发后暂停
static constexpr uint16_t DART_MID_PAUSE_MS = 30000;//30s暂停时间
static bool referee_allow_overlimit_stop = false;// 超过允许次数后停止发射，直到重置

// 发射命令上升沿 token：每次上升沿+1；由发射状态机消费
static uint32_t shoot_cmd_token = 0;
static uint32_t shoot_cmd_token_consumed = 0;
// 单次发射循环激活标志：接收一次命令后，完整执行一轮（后续发次含换弹）
static bool shooting_cycle_active = false;

//使能发射机构的enable_booster_flag
uint8_t enable_booster_flag = 0;
// static bool claw_closed_for_calibration = false;

// 上位机离线保底备案：MiniPC指针（在Init时由Class_Chariot设置）
Class_MiniPC *MiniPC_For_Booster = nullptr;
Class_Referee *Referee_For_Booster = nullptr;

void Update_Referee_Allow_Edge()
{
    // ============ 上位机离线保底备案 ============
    // 条件: 上位机掉线 + 比赛已开始 + 舱门已开启 → 自动置位 Referee_Allow_Shoot
    // 使用裁判系统实时数据（UART10直连），非上位机转发
    if (MiniPC_For_Booster != nullptr && Referee_For_Booster != nullptr &&//空指针保护
        MiniPC_For_Booster->Get_MiniPC_Status() == MiniPC_Status_DISABLE)
    {
        bool game_started = (Referee_For_Booster->Get_Game_Stage() == Referee_Game_Status_Stage_BATTLE);
        bool hatch_open = (Referee_For_Booster->Get_Dart_Command_Status() == Referee_Data_Robot_Dart_Command_Status_OPEN);
        bool fallback_condition = game_started && hatch_open;
        if (fallback_condition && !minipc_fallback_prev && Push_Calibration_Finished && Pull_Calibration_Finished)
        {
			//一套流程 包括 yaw跑到固定的位置 发射
            // Referee_Allow_Shoot = true;//暂时不启用
        }
    }
    minipc_fallback_prev = (MiniPC_For_Booster != nullptr &&
                            MiniPC_For_Booster->Get_MiniPC_Status() == MiniPC_Status_DISABLE);

    // ============ 上位机联合自动化发射 ============
    // 空闲条件：校准完成 + booster使能 + 非发射中 + 非已允许 + 镖未打完
    // 冷却：每发结束后等 300ms 再发起下一次请求
    static uint16_t shoot_cooldown_ms = 0;
    if (shooting_cycle_active || Referee_Allow_Shoot)
    {
        shoot_cooldown_ms = 0;
    }
    else if (shoot_cooldown_ms < 300)
    {
        shoot_cooldown_ms++;
    }
    bool cooldown_ready = (shoot_cooldown_ms >= 300);//冷却准备就绪

    // 只在第 2 发已完成、且当前不在发射流程中时计 30s 暂停
    static uint16_t dart_mid_pause_ms = 0;
    if (dart_fired_count == DART_PAUSE_AFTER_COUNT && !shooting_cycle_active && !Referee_Allow_Shoot)
    {
        if (dart_mid_pause_ms < DART_MID_PAUSE_MS)
        {
            dart_mid_pause_ms++;
        }
    }
    else if (dart_fired_count != DART_PAUSE_AFTER_COUNT)
    {
        dart_mid_pause_ms = 0;
    }
    // 非第 2 发后，或第 2 发后的 30s 已结束，才允许向上位机申请下一发
    bool dart_mid_pause_ready = (dart_fired_count != DART_PAUSE_AFTER_COUNT) || (dart_mid_pause_ms >= DART_MID_PAUSE_MS);

    bool is_idle_ready = Push_Calibration_Finished && Pull_Calibration_Finished
                         && (enable_booster_flag == 1)
                         && !shooting_cycle_active
                         && !Referee_Allow_Shoot
                         && (dart_fired_count < 4)
                         && cooldown_ready
                         && dart_mid_pause_ready;

    // aaa_3 = is_idle_ready;

    if (MiniPC_For_Booster != nullptr)
    {
        // a) 发射申请：空闲时通过 CAN TX [7] 告诉上位机"我准备好了"
        MiniPC_For_Booster->Set_CAN_Tx_Shoot_Request(is_idle_ready ? 1u : 0u);

        // b) 上位机上升沿检测：CAN RX [4] 从 0→1 时置位 Referee_Allow_Shoot 一次
        static bool prev_allow_shoot_from_minipc = false;
        bool curr_allow_shoot = (MiniPC_For_Booster->Get_CAN_Rx_MiniPC_Allow_Shoot() != 0);

        // 暂停期间即使上位机提前给 Allow_Shoot，也不生成本次发射命令
        if (is_idle_ready && curr_allow_shoot && !prev_allow_shoot_from_minipc)
        {
            Referee_Allow_Shoot = true;
        }
        prev_allow_shoot_from_minipc = curr_allow_shoot;
    }

    // ============ 正常Referee_Allow_Shoot边沿检测 ============
    if (Referee_Allow_Shoot && !referee_allow_prev) {
        referee_allow_rise_cnt++;
        shoot_cmd_token++;
        if (referee_allow_rise_cnt > REFEREE_ALLOW_SHOOT_MAX_COUNT)
        {
            referee_allow_overlimit_stop = true;
            Referee_Allow_Shoot = false;
        }
    }
    referee_allow_prev = Referee_Allow_Shoot;
}

extern "C" void Booster_On_PB3_Exti(void)
{
    pb3_exti_irq_count++;
    pb3_press_event_latched = true;
}

extern "C" void Booster_On_PD7_Exti(void)
{
    pd7_exti_irq_count++;
    pd7_press_event_latched = true;
}

bool Consume_PB3_Press_Event()
{
    __disable_irq();
    bool has_event = pb3_press_event_latched;
    pb3_press_event_latched = false;
    __enable_irq();
    if (has_event)
    {
        pb3_event_consumed_count++;
    }
    return has_event;
}

bool Consume_PD7_Press_Event()
{
    __disable_irq();
    bool has_event = pd7_press_event_latched;
    pd7_press_event_latched = false;
    __enable_irq();
    if (has_event)
    {
        pd7_event_consumed_count++;
    }
    return has_event;
}

extern "C" void Booster_On_PA5_Exti(void)
{
    pa5_exti_irq_count++;
    pa5_press_event_latched = true;
}

bool Consume_PA5_Press_Event()
{
    __disable_irq();
    bool has_event = pa5_press_event_latched;
    pa5_press_event_latched = false;
    __enable_irq();
    if (has_event)
    {
        pa5_event_consumed_count++;
    }
    return has_event;
}

/*-----------------------------------------------*/

/* Private types -------------------------------------------------------------*/
#define TENSION_DEADZONE 0.2f // 拉力死区
#define SCREW_LEAD 0.004f // 4mm 螺距（每转一圈线性移动4mm）
/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief  将当前角度线性映射到目标行程
 * @param  curr_angle   当前电机角度
 * @param  angle_start  起始点角度（通常是 Backward 角度）0
 * @param  angle_end    结束点角度（通常是 Forward 角度）1
 * @param  max_length   物理最大行程（例如 1.0 表示百分比，或者 200.0 表示 mm）
 * @return 映射后的位置值
 */
float Class_FSM_Push_Calibration::Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length)
{
    // 防止分母为0（极其罕见的情况，但为了安全）
    if (fabs(angle_end - angle_start) < 0.001f)
    {
        return 0.0f;
    }

    // 1. 计算归一化比例 (Ratio 0.0 ~ 1.0)
    // 公式: (x - min) / (max - min)
    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);

    // 2. 安全限幅 (Clamping)
    // 这一步非常重要：如果当前角度因为惯性稍微超过了校准值，
    // 不限幅会导致 PID 计算出的误差反向剧增，引发震荡。
    if (ratio < 0.0f)
        ratio = 0.0f;
    if (ratio > 1.0f)
        ratio = 1.0f;

    // 3. 映射到物理长度
    return ratio * max_length;
}

float Class_FSM_Pull_Calibration::Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length)
{
    // 防止分母为0（极其罕见的情况，但为了安全）
    if (fabs(angle_end - angle_start) < 0.001f)
    {
        return 0.0f;
    }

    // 1. 计算归一化比例 (Ratio 0.0 ~ 1.0)
    // 公式: (x - min) / (max - min)
    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);

    // 2. 安全限幅 (Clamping)
    // 这一步非常重要：如果当前角度因为惯性稍微超过了校准值，
    // 不限幅会导致 PID 计算出的误差反向剧增，引发震荡。
    if (ratio < 0.0f)
        ratio = 0.0f;
    if (ratio > 1.0f)
        ratio = 1.0f;

    // 3. 映射到物理长度
    return ratio * max_length;
}

// 已经通过串口读取到一拉力值
// 使用全局变量保存，单位为kg
// 拉力环比例系数
float K_tension = 0.000000120f;
// 拉力环积分系数
float K_tension_i = 0.0f;
// 拉力误差积分累计
static float tension_error_integral = 0.0f;
// 积分限幅，防止积分饱和
static constexpr float TENSION_ERROR_INTEGRAL_LIMIT = 120000.0f;
// 拉力目标斜坡：每次调用递增，避免阶跃
static float ramped_target_tension = 0.0f;
static constexpr float TENSION_RAMP_STEP = 1.8f;//斜坡步进值
// 扣锁检测阈值：测量值超过此值说明已扣住
static constexpr float TENSION_LATCH_THRESHOLD = 37500.0f;
static bool tension_latched = false;
static uint16_t latch_fallback_tick = 0;
static uint32_t latch_tick = 0;
static constexpr uint32_t LATCH_DELAY_MS = 300;
static constexpr float TENSION_LATCH_FALLBACK_THRESHOLD = TENSION_LATCH_THRESHOLD - 10000.0f;
static constexpr uint16_t TENSION_LATCH_FALLBACK_TIMEOUT_MS = 500;
static constexpr uint32_t LATCH_SETTLE_TIMEOUT_MS = 800;
static constexpr uint16_t TENSION_SETTLE_STABLE_MS = 80;
static constexpr float TENSION_SETTLE_DELTA_THRESHOLD = 1500.0f;//连续80ms内读数变化小于1500
static bool tension_f0_captured = false;
static uint16_t tension_settle_stable_ms = 0;
static float tension_settle_last_value = 0.0f;
static float tension_start_f0 = 0.0f;
/**
 * @brief 拉力外环控制（将拉力误差映射为 Pull 电机的目标位置）
 *
 */
void Class_Booster::Pull_Tension_Control(bool is_first_run)
{
    // 1. [关键] 首运行初始化：防止电机突然跳变
    if (is_first_run)
    {
        target_tension_position_pull = Get_Now_position_pull();
        tension_error_integral = 0.0f;
        tension_latched = false;
        latch_fallback_tick = 0;
        latch_tick = 0;
        tension_f0_captured = false;
        tension_settle_stable_ms = 0;
        tension_settle_last_value = 0.0f;
        tension_start_f0 = 0.0f;
    }

    {
        // 读取测量值与最终目标
        now_tension_value = Get_Measured_Tension();
        target_tension_value = Get_Target_Tension();

        // 扣锁检测：力值突增说明刚扣住，记录时刻并从当前力值开始斜坡
        bool should_latch_tension = false;
        if (!tension_latched)
        {
            if (now_tension_value >= TENSION_LATCH_THRESHOLD)
            {
                should_latch_tension = true;
                latch_fallback_tick = 0;
            }
            else if (now_tension_value >= TENSION_LATCH_FALLBACK_THRESHOLD)
            {
                if (latch_fallback_tick < TENSION_LATCH_FALLBACK_TIMEOUT_MS)
                {
                    latch_fallback_tick++;
                }
                should_latch_tension = latch_fallback_tick >= TENSION_LATCH_FALLBACK_TIMEOUT_MS;
            }
            else
            {
                latch_fallback_tick = 0;
            }
        }

        if (!tension_latched && should_latch_tension)
        {
            tension_latched = true;
            latch_fallback_tick = 0;
            latch_tick = 0;
            tension_f0_captured = false;
            tension_settle_stable_ms = 0;
            tension_settle_last_value = now_tension_value;
            tension_start_f0 = now_tension_value;
            ramped_target_tension = now_tension_value;
            tension_error_integral = 0.0f;
        }

        // 未扣锁时不做PID控制
        if (!tension_latched)
        {
            return;
        }

        // 扣锁后等待300ms再开始PID控制，让机构稳定
        // Capture F0 only after force readings settle, while keeping the old 300 ms minimum delay.
        if (!tension_f0_captured)
        {
            if (latch_tick < LATCH_SETTLE_TIMEOUT_MS)
            {
                latch_tick++;
            }

            const float settle_delta = fabs(now_tension_value - tension_settle_last_value);
            tension_settle_last_value = now_tension_value;

            if (settle_delta < TENSION_SETTLE_DELTA_THRESHOLD)
            {
                if (tension_settle_stable_ms < 0xFFFF)
                {
                    tension_settle_stable_ms++;
                }
            }
            else
            {
                tension_settle_stable_ms = 0;
            }

            const bool latch_delay_done = latch_tick >= LATCH_DELAY_MS;
            const bool impact_settled = tension_settle_stable_ms >= TENSION_SETTLE_STABLE_MS;
            const bool settle_timeout = latch_tick >= LATCH_SETTLE_TIMEOUT_MS;

            if (!latch_delay_done || (!impact_settled && !settle_timeout))
            {
                return;
            }

            tension_start_f0 = now_tension_value;
            ramped_target_tension = tension_start_f0;
            tension_error_integral = 0.0f;
            tension_f0_captured = true;
        }

        // 斜坡：逐步逼近最终目标，消除阶跃超调
        float ramp_diff = target_tension_value - ramped_target_tension;
        if (fabs(ramp_diff) > TENSION_RAMP_STEP)
        {
            ramped_target_tension += (ramp_diff > 0.0f) ? TENSION_RAMP_STEP : -TENSION_RAMP_STEP;
        }
        else
        {
            ramped_target_tension = target_tension_value;
        }

        float tension_error = ramped_target_tension - now_tension_value;

        // 增加死区防止抖动
        if (fabs(tension_error) < TENSION_DEADZONE)
        {
            tension_error = 0.0f;
        }

        // 积分分离：输出已在边界且误差继续推动越界时，不继续累积积分
        const bool integral_blocked_by_saturation =
            (target_tension_position_pull <= 0.05f && tension_error > 0.0f) ||
            (target_tension_position_pull >= 0.98f && tension_error < 0.0f);

        if (!integral_blocked_by_saturation)
        {
            tension_error_integral += tension_error;
            if (tension_error_integral > TENSION_ERROR_INTEGRAL_LIMIT)
            {
                tension_error_integral = TENSION_ERROR_INTEGRAL_LIMIT;
            }
            else if (tension_error_integral < -TENSION_ERROR_INTEGRAL_LIMIT)
            {
                tension_error_integral = -TENSION_ERROR_INTEGRAL_LIMIT;
            }
        }

        const float tension_delta = K_tension * tension_error + K_tension_i * tension_error_integral;
        target_tension_position_pull -= tension_delta;

        // 限幅
        if (target_tension_position_pull > 0.98f)
        {
            target_tension_position_pull = 0.98f;
        }
        if (target_tension_position_pull < 0.05f)
        {
            target_tension_position_pull = 0.05f;
        }

        Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Motor_Pull.Set_Target_Radian(target_tension_position_pull);
    }
}

void Class_FSM_Push_Calibration::Push_Calibration_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0): // 向前堵转 目前用微动开关
    {
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        Booster->Motor_Push_L.Set_Target_Omega_Radian(speed);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(speed);

        // 进入该状态第一帧清除旧事件，避免跨状态误触发
        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PD7_Press_Event();
        }
        
        if(PD7_GPIO == 1 && Consume_PD7_Press_Event())//前左侧微动开关触发
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Angle_Forward_L = Booster->Motor_Push_L.Get_Now_Angle();
            Angle_Forward_R = Booster->Motor_Push_R.Get_Now_Angle();
            Set_Status(1);
        }
    }
    break;
    case (1): // 向后堵转 目前用微动开关
    {
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        Booster->Motor_Push_L.Set_Target_Omega_Radian(-speed);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(-speed);

        // 进入该状态第一帧清除旧事件，避免跨状态误触发
        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PB3_Press_Event();
        }
        
        if(PB3_GPIO == 1 || Consume_PB3_Press_Event())//后端左侧微动开关触发
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Angle_Backward_L = Booster->Motor_Push_L.Get_Now_Angle();
            Angle_Backward_R = Booster->Motor_Push_R.Get_Now_Angle();
            Set_Status(2);
        }
    }
    break;
    case (2): // 从底部重新上行，触发顶部微动开关
    {
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

        Booster->Motor_Push_L.Set_Target_Omega_Radian(240.0f);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(240.0f);

        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PD7_Press_Event();
        }

        if ((PD7_GPIO == 1) || Consume_PD7_Press_Event())
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Push_Calibration_Finished = true;
            Set_Status(3);
        }
    }
    break;
    case (3): // 校准完成保持态：保持在顶部位置，避免机构持续受力
    {
        // 定义上面是1.0f最大行程 下面是0.0f最小行程
        // 所以在函数映射的时候颠倒了位置
        float now_position_l = Linear_Map_Position(Booster->Motor_Push_L.Get_Now_Angle(), Angle_Backward_L, Angle_Forward_L, 1.0f);
        float now_position_r = Linear_Map_Position(Booster->Motor_Push_R.Get_Now_Angle(), Angle_Backward_R, Angle_Forward_R, 1.0f);
        float now_position = (now_position_l + now_position_r) / 2.0f;
        Booster->Set_Now_position_push(now_position); // 更新当前push电机位置
        // 更新PID输入值
        Booster->Motor_Push_L.Set_Transform_Angle(now_position);
        Booster->Motor_Push_R.Set_Transform_Angle(now_position);

        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Push_L.Set_Target_Radian(0.98f);
        Booster->Motor_Push_R.Set_Target_Radian(0.98f);

        if (Push_Calibration_Finished && Pull_Calibration_Finished)
        {
            Booster->Set_Booster_Control_Type(Booster_Control_Type_NORMAL);
            Set_Status(4);
        }
    }
    break;
    case (4): // 后校准观测态：持续更新位置映射，不再对Push电机下发保持指令
    {
        float now_position_l = Linear_Map_Position(Booster->Motor_Push_L.Get_Now_Angle(), Angle_Backward_L, Angle_Forward_L, 1.0f);
        float now_position_r = Linear_Map_Position(Booster->Motor_Push_R.Get_Now_Angle(), Angle_Backward_R, Angle_Forward_R, 1.0f);
        float now_position = (now_position_l + now_position_r) / 2.0f;
        Booster->Set_Now_position_push(now_position);
        Booster->Motor_Push_L.Set_Transform_Angle(now_position);
        Booster->Motor_Push_R.Set_Transform_Angle(now_position);
    }
    }
}

void Class_FSM_Pull_Calibration::Pull_Calibration_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    // 自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
    case (0): // 向上校准 -> 微动开关触发
    {
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Pull.Set_Target_Omega_Radian(speed);

        if (PA5_GPIO == 1 || Consume_PA5_Press_Event()) // PA5 微动开关触发
        {
            Angle_Forward = Booster->Motor_Pull.Get_Now_Angle();
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Pull.Set_Target_Omega_Radian(0.0f);
            Set_Status(1);
        }
    }
    break;
    case (1): // 向后堵转
    {
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Pull.Set_Target_Omega_Radian(-speed);

        if (fabs(Booster->Motor_Pull.Get_Now_Torque()) > Torque_Threshold)
        {
            Set_Status(2);
        }
    }
    break;
    case (2): // 后侧检测
    {
        if (Status[Now_Status_Serial].Time > 100)
        {
            Angle_Backward = Booster->Motor_Pull.Get_Now_Angle();
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Pull.Set_Target_Omega_Radian(0.0f);
            Set_Status(3);
        }
        else if (fabs(Booster->Motor_Pull.Get_Now_Torque()) < Torque_Threshold)
        {
            Set_Status(1);
        }
    }
    break;
    case (3): // 正常控制流程
    {
        Pull_Calibration_Finished = true;
        Set_Status(4);
    }
    break;
    case (4): // 校准检测
    {
        // 进入校准保持态首帧时，切到角度环并锁定到固定位置，避免“放空”漂移
        if (Status[Now_Status_Serial].Time == 1)
        {
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Pull.Set_Target_Radian(pull_hold_after_calib_pos);
        }

        // 定义上面是1.0f最大行程 下面是0.0f最小行程
        float now_position = Linear_Map_Position(Booster->Motor_Pull.Get_Now_Angle(), Angle_Backward, Angle_Forward, 1.0f); // 注意这里颠倒了
        Booster->Set_Now_position_pull(now_position);                                                                       // 更新当前pull电机位置
        // 更新PID输入值
        Booster->Motor_Pull.Set_Transform_Angle(now_position);


        if (Push_Calibration_Finished && Pull_Calibration_Finished)
        {
            Booster->Set_Booster_Control_Type(Booster_Control_Type_NORMAL);
        }
    }
    }
}

float pull_position_task_C[4] = {0.46f, 0.45f, 0.45f, 0.45f};

void Class_FSM_Shooting::Shooting_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    constexpr uint8_t kMaxDartCount = 4;
    constexpr uint16_t kDownLockServoCloseDelayMs = 500;
    constexpr float kPushDownOmega = -320.0f;
    constexpr float kPushUpOmega = 370.0f;
    constexpr float kPushBackoffOmega = -20.0f;
    constexpr float kPushBackoffDistance = 0.006f;
    constexpr float kReloadReachTolerance = 0.001f;
    constexpr float kSafeAngleTolerance = 0.002f;
    constexpr uint16_t kReloadClawOpenDelayMs = 800;
    constexpr uint16_t kReloadDropDelayMs = 1500;
    constexpr uint16_t kFireHoldMs = 500;
    constexpr float kTensionReadyThreshold = 80.0f;
    constexpr uint16_t kTensionStableMs = 150;

    static uint8_t down_lock_stage = 0;
    static uint16_t down_lock_close_start_time = 0;

    if (Booster->Get_Booster_Control_Type() != Booster_Control_Type_NORMAL)
    {
        shooting_cycle_active = false;
        Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle);//这里需要保持撒放器张开 撒放器张开才有触碰微动开关的机会
        Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Reload_Angle.Set_Target_SingleTurn_Radian_Nearest(Booster->init_position_reload_angle);
        Booster->target_position_reload_angle = Booster->Motor_Reload_Angle.Get_Target_Radian();
        Booster->Set_Reload_Status(Reload_Status_FINISHED);
        if (Now_Status_Serial != Shooting_Control_Type_IDLE)
        {
            Set_Status(Shooting_Control_Type_IDLE);
        }
        Shooting_Control_Type = static_cast<Enum_Shooting_Control_Type>(Now_Status_Serial);
        return;
    }

    uint8_t reload_index = 0;
    if (dart_fired_count > 0)
    {
        reload_index = static_cast<uint8_t>(dart_fired_count - 1);
    }
    if (reload_index >= kMaxDartCount)
    {
        reload_index = static_cast<uint8_t>(kMaxDartCount - 1);
    }

    const float reload_angle = GM6020_angle_RELOAD[reload_index] - 0.5f * PI / 180.0f;
    const float safe_elude_angle = GM6020_angle_ELUDE[reload_index] - 0.5f * PI / 180.0f;
    const float ready_fire_angle = isFirstShot ? Booster->init_position_reload_angle : safe_elude_angle;

    switch (Now_Status_Serial)
    {
    case (Shooting_Control_Type_IDLE):
    {
        Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle);//这里需要保持撒放器张开 撒放器张开才有触碰微动开关的机会
        Booster->Set_Reload_Status(Reload_Status_FINISHED);

        // if (isFirstShot)
        // {
        //     // 6020 绝对值编码器：首发前始终锁定在初始安全角
        //     Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        //     Booster->Motor_Reload_Angle.Set_Target_SingleTurn_Radian_Nearest(Booster->init_position_reload_angle);
        //     Booster->target_position_reload_angle = Booster->Motor_Reload_Angle.Get_Target_Radian();
        // }

        if (Status[Now_Status_Serial].Time == 1)
        {
            prep_task_a_done = false;
            prep_task_b_done = false;
            prep_task_c_done = false;
            push_top_reached = false;
            pull_loop_first_run = true;
            reload_stage = 0;
            down_lock_stage = 0;
            down_lock_close_start_time = 0;
            tension_in_range_time_ms = 0;
        }

        if (dart_fired_count >= kMaxDartCount)
        {
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Pull.Set_Target_Radian(pull_hold_after_calib_pos);
            break;
        }

        if (!shooting_cycle_active && (shoot_cmd_token > shoot_cmd_token_consumed))
        {
            shoot_cmd_token_consumed = shoot_cmd_token;
            shooting_cycle_active = true;
            Set_Status(Shooting_Control_Type_DOWN_LOCK);
        }
    }
    break;

    case (Shooting_Control_Type_DOWN_LOCK):
    {
        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PB3_Press_Event();
            Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle);//这里需要保持撒放器张开 撒放器张开才有触碰微动开关的机会
            down_lock_stage = 0;
            down_lock_close_start_time = 0;
        }

        // if (isFirstShot)
        // {
        //     Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        //     Booster->Motor_Reload_Angle.Set_Target_SingleTurn_Radian_Nearest(Booster->init_position_reload_angle);
        //     Booster->target_position_reload_angle = Booster->Motor_Reload_Angle.Get_Target_Radian();
        // }

        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        //新添加pull电机动作
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

        if (down_lock_stage == 0)
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(kPushDownOmega);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(kPushDownOmega);
            //新添加pull电机动作
            Booster->Motor_Pull.Set_Target_Radian(pull_hold_after_calib_pos);

            if ((PB3_GPIO == 1) || Consume_PB3_Press_Event())
            {
                Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
                Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
                // Push 下压触底后先闭合撒放器并等待一小段时间，避免扣不住
                Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle);
                down_lock_close_start_time = static_cast<uint16_t>(Status[Now_Status_Serial].Time);
                down_lock_stage = 1;
            }
        }
        else
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle);

            if ((Status[Now_Status_Serial].Time - down_lock_close_start_time) > kDownLockServoCloseDelayMs)
            {
                if (isFirstShot)
                {
                    Set_Status(Shooting_Control_Type_PREP_FIRE);
                }
                else
                {
                    Set_Status(Shooting_Control_Type_RELOAD);
                }
            }
        }
    }
    break;

    case (Shooting_Control_Type_RELOAD):
    {
        Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle);
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Pull.Set_Target_Radian(pull_hold_after_calib_pos);

        if (Status[Now_Status_Serial].Time == 1)
        {
            Booster->Set_Reload_Status(Reload_Status_RELOADING);
            Booster->target_position_reload_angle = reload_angle;
            reload_stage = 0;
            reload_drop_start_time = 0;
        }

        Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Reload_Angle.Set_Target_Radian(Booster->target_position_reload_angle);

        if ((reload_stage == 0)
            && (fabs(Booster->Motor_Reload_Angle.Get_Now_Radian() - Booster->target_position_reload_angle) < kReloadReachTolerance))
        {
            reload_drop_start_time = static_cast<uint16_t>(Status[Now_Status_Serial].Time);
            reload_stage = 1;
        }

        if ((reload_stage == 1)
            && (Status[Now_Status_Serial].Time - reload_drop_start_time > kReloadClawOpenDelayMs))
        {
            Booster->Servo_Claw[reload_index].Set_Target_Angle(Booster->claw_open_angle[reload_index]);
            reload_drop_start_time = static_cast<uint16_t>(Status[Now_Status_Serial].Time);
            reload_stage = 2;
        }

        if ((reload_stage == 2)
            && (Status[Now_Status_Serial].Time - reload_drop_start_time > kReloadDropDelayMs))
        {
            for (uint8_t i = 0; i < 3; i++)
            {
                Booster->Servo_Claw[i].Set_Target_Angle(Booster->claw_close_angle[i]);
            }

            last_dart_fired_count = dart_fired_count;
            reload_count += 1;
            reload_done_token += 1;
            Booster->Set_Reload_Status(Reload_Status_FINISHED);
            Set_Status(Shooting_Control_Type_PREP_FIRE);
        }
    }
    break;

    case (Shooting_Control_Type_PREP_FIRE):
    {
        Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle);

        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PD7_Press_Event();
            prep_task_a_done = false;
            prep_task_b_done = false;
            prep_task_c_done = false;
            push_top_reached = false;
            pull_loop_first_run = true;
            pull_pos_stable_ms = 0;//task_C 位置环的稳定时间清0
            tension_in_range_time_ms = 0;
            Booster->target_position_reload_angle = ready_fire_angle;
        }

        // Task A: 6020 转到安全避让角
        Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Reload_Angle.Set_Target_Radian(Booster->target_position_reload_angle);
        if (fabs(Booster->Motor_Reload_Angle.Get_Now_Radian() - Booster->target_position_reload_angle) < kSafeAngleTolerance)
        {
            prep_task_a_done = true;
        }

        // Task B: Push 上行触顶后反向微退保护开关
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        if (!push_top_reached)
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(kPushUpOmega);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(kPushUpOmega);

            if ((PD7_GPIO == 1) && Consume_PD7_Press_Event())
            {
                Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
                Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
                prep_top_position_meter =
                    (Booster->Motor_Push_L.Get_Now_Radian() + Booster->Motor_Push_R.Get_Now_Radian()) / 2.0f / (2.0f * PI) * SCREW_LEAD;
                push_top_reached = true;
            }
        }
        else if (!prep_task_b_done)
        {
            const float now_position_meter =
                (Booster->Motor_Push_L.Get_Now_Radian() + Booster->Motor_Push_R.Get_Now_Radian()) / 2.0f / (2.0f * PI) * SCREW_LEAD;

            Booster->Motor_Push_L.Set_Target_Omega_Radian(kPushBackoffOmega);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(kPushBackoffOmega);

            if (now_position_meter < prep_top_position_meter - kPushBackoffDistance)
            {
                Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
                Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
                prep_task_b_done = true;
            }
        }

        // // Task C: Pull 拉力闭环并等待稳定
        // Booster->Pull_Tension_Control(pull_loop_first_run);
        // pull_loop_first_run = false;

        // const float tension_error = fabs(static_cast<float>(Booster->Get_Target_Tension() - Booster->Get_Measured_Tension()));
        // if (tension_error < kTensionReadyThreshold)
        // {
        //     if (tension_in_range_time_ms < 0xFFFF)
        //     {
        //         tension_in_range_time_ms++;
        //     }
        // }
        // else
        // {
        //     tension_in_range_time_ms = 0;
        // }

        // if (tension_in_range_time_ms >= kTensionStableMs)
        // {
        //     prep_task_c_done = true;
        // }
        // /*---------------------------------------------------------------*/

        /*-------------TaskC：位置环--------------------------------*/
        const uint8_t pull_idx = dart_fired_count < kMaxDartCount ? static_cast<uint8_t>(dart_fired_count) : static_cast<uint8_t>(kMaxDartCount - 1);
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Pull.Set_Target_Radian(pull_position_task_C[pull_idx]);

        if (fabs(Booster->Motor_Pull.Get_Now_Radian() - pull_position_task_C[pull_idx]) < 0.005f)
        {
            // if (pull_pos_stable_ms < 0xFFFF) pull_pos_stable_ms++;
            prep_task_c_done = true;
        }
        // // else
        // // {
        // //     pull_pos_stable_ms = 0;
        // // }

        // // if (pull_pos_stable_ms >= 150)
        // // {
        // //     prep_task_c_done = true;
        // // }
        //  /*---------------------------------------------------------*/

        // 超时保护：超过 4 秒未到位则强制放行
        if (Status[Now_Status_Serial].Time > 4000)
        {
            prep_task_c_done = true;
        }

        if (prep_task_a_done && prep_task_b_done && prep_task_c_done)
        {
            Set_Status(Shooting_Control_Type_READY);
        }
    }
    break;

    case (Shooting_Control_Type_READY):
    {
        Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle);
        Booster->Set_Reload_Status(Reload_Status_FINISHED);

        Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Reload_Angle.Set_Target_Radian(ready_fire_angle);

        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);

        // Booster->Pull_Tension_Control(false);

        const bool safe_ready = fabs(Booster->Motor_Reload_Angle.Get_Now_Radian() - ready_fire_angle) < kSafeAngleTolerance;
        if (safe_ready)
        {
            Set_Status(Shooting_Control_Type_FIRE);
        }
    }
    break;

    case (Shooting_Control_Type_FIRE):
    {
        Booster->Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Reload_Angle.Set_Target_Radian(ready_fire_angle);

        // const bool safe_ready = fabs(Booster->Motor_Reload_Angle.Get_Now_Radian() - ready_fire_angle) < kSafeAngleTolerance;
        // if (!safe_ready)
        // {
        //     Set_Status(Shooting_Control_Type_READY);
        //     break;
        // }

        if (Status[Now_Status_Serial].Time == 1)
        {
            Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle);
        }

        if (Status[Now_Status_Serial].Time > kFireHoldMs)
        {
            dart_fired_count += 1;
            shooting_cycle_active = false;
            isFirstShot = false;
            // 单发结束后清命令位
            Referee_Allow_Shoot = false;
            Set_Status(Shooting_Control_Type_IDLE);
        }
    }
    break;

    default:
    {
        Set_Status(Shooting_Control_Type_IDLE);
    }
    break;
    }

    Shooting_Control_Type = static_cast<Enum_Shooting_Control_Type>(Now_Status_Serial);
}

// 测试参数
float Motor_L_test_P = 270.0f;
float Motor_L_test_I = 25.f;
float Motor_R_test_P = 320.0f;
float Motor_R_test_I = 28.f;

float Motor_Push_Angle_P_test = 4200.0f;
float Motor_Push_Angle_I_test = 0.0f;

float Motor_Pull_Omega_test_P = 900.0f;
float Motor_Pull_Omega_test_I = 145.0f;
float Motor_Pull_Angle_P_test = 2200.0f;
float Motor_Pull_Angle_I_test = 0.0f;

// 6020换弹角度电机
float Motor_Reload_6020_Omega_P_test = 925.0f;
float Motor_Reload_6020_Omega_I_test = 2400.0f;
float Motor_Reload_6020_Omega_D_test = 0.0f;
float Motor_Reload_6020_Anlge_P_test = 43.0f;
float Motor_Reload_6020_Anlge_I_test = 0.0f;
float Motor_Reload_6020_Anlge_D_test = 0.3f;


/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{
    FSM_Shooting.Booster = this;
    FSM_Shooting.Init(6, 0);//状态机上限是10

    FSM_Reload.Booster = this;
    FSM_Reload.Init(6, 0);

    FSM_Push_Calibration.Booster = this;
    FSM_Push_Calibration.Init(6, 0);

    FSM_Pull_Calibration.Booster = this;
    FSM_Pull_Calibration.Init(6, 0);

    // 舵机
    Servo_Trigger.Init(&htim2, TIM_CHANNEL_1, 270);
    Servo_Trigger.Set_Target_Angle(tirrger_fire_angle);

    Servo_Claw[0].Init(&htim2, TIM_CHANNEL_3, 270);
    Servo_Claw[0].Set_Target_Angle(claw_close_angle[0]);
    Servo_Claw[1].Init(&htim1, TIM_CHANNEL_1, 270);
    Servo_Claw[1].Set_Target_Angle(claw_close_angle[1]);
    Servo_Claw[2].Init(&htim1, TIM_CHANNEL_3, 270);
    Servo_Claw[2].Set_Target_Angle(claw_close_angle[2]);
    // claw_closed_for_calibration = false;

    // 拉力电机
    Motor_Pull.PID_Angle.Init(Motor_Pull_Angle_P_test, Motor_Pull_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 130.0f * PI);
    Motor_Pull.PID_Omega.Init(Motor_Pull_Omega_test_P, Motor_Pull_Omega_test_I, 0.0f, 0.0f, 2000, Motor_Pull.Get_Output_Max());
    Motor_Pull.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);

    // Push电机左
    Motor_Push_L.PID_Angle.Init(Motor_Push_Angle_P_test, Motor_Push_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Push_L.PID_Omega.Init(Motor_L_test_P, Motor_L_test_I, 0.0f, 0.0f, Motor_Push_L.Get_Output_Max() * 0.5f, Motor_Push_L.Get_Output_Max());
    Motor_Push_L.Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);

    // Push电机右
    Motor_Push_R.PID_Angle.Init(Motor_Push_Angle_P_test, Motor_Push_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Push_R.PID_Omega.Init(Motor_R_test_P, Motor_R_test_I, 0.0f, 0.0f, Motor_Push_R.Get_Output_Max() * 0.5f, Motor_Push_R.Get_Output_Max());
    Motor_Push_R.Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA, 1.0f);

    // 换弹电机角度
    Motor_Reload_Angle.Init(&hfdcan1, DJI_Motor_ID_0x205, DJI_Motor_Control_Method_ANGLE);
    Motor_Reload_Angle.PID_Angle.Init(Motor_Reload_6020_Anlge_P_test, Motor_Reload_6020_Anlge_I_test, Motor_Reload_6020_Anlge_D_test, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Reload_Angle.PID_Omega.Init(Motor_Reload_6020_Omega_P_test, Motor_Reload_6020_Omega_I_test, Motor_Reload_6020_Omega_D_test, 0.0f, Motor_Reload_Angle.Get_Output_Max() * 0.8f, Motor_Reload_Angle.Get_Output_Max(),0.0f,0.0f,0.0f);

    Set_Reload_Status(Reload_Status_FINISHED); // 初始化为换弹完成状态
}

/**
 * @brief 输出到电机
 *
 */
int testtnum = 0;
float test_position_b = 79.0f;
float test_b = 3.0f;
// float aaaaaaa = 22.0f;
float Target_test_b_pull = 0.5f;

// float test_servo_claw = 70.0f;
// int xuhao = 0;

void Class_Booster::Output()
{
    // Servo_Claw[xuhao].Set_Target_Angle(test_servo_claw);// 115 / 70

    // Servo_Reload.Set_Target_Angle(aaaaaaa);
    // // 下面是测试代码，正式使用时请删除
    // if(testtnum == 0)
    // {
    //     // 换弹角度电机转动60度
    //     target_position_reload_angle += 40.0f * PI / 180.0f;

    //     Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    //     Motor_Reload_Angle.Set_Target_Radian(test_position_b * PI / 180.0f);
    //     // Motor_Reload_Angle.Set_Target_Radian(target_position_reload_angle);

    //     FSM_Reload.Set_Status(Reload_Control_Type_Test);

    //     testtnum = 0;
    // }

    //  // 设置电机控制模式（调试用）
    // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

    // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

    // //角度环调参
    // Motor_Pull.PID_Angle.Set_K_P(Motor_Pull_Angle_P_test);
    // Motor_Pull.PID_Angle.Set_K_I(Motor_Pull_Angle_I_test);
    // Motor_Push_L.PID_Angle.Set_K_P(Motor_Push_Angle_P_test);
    // Motor_Push_R.PID_Angle.Set_K_P(Motor_Push_Angle_P_test);
    // Motor_Push_L.PID_Angle.Set_K_I(Motor_Push_Angle_I_test);
    // Motor_Push_L.PID_Angle.Set_K_I(Motor_Push_Angle_I_test);
    // Motor_Reload_Linear.PID_Angle.Set_K_P(Motor_Reload_C610_Anlge_P_test);
    // Motor_Reload_Linear.PID_Angle.Set_K_I(Motor_Reload_C610_Anlge_I_test);
    // Motor_Reload_Angle.PID_Angle.Set_K_P(Motor_Reload_6020_Anlge_P_test);
    // Motor_Reload_Angle.PID_Angle.Set_K_I(Motor_Reload_6020_Anlge_I_test);
    // Motor_Reload_Angle.PID_Angle.Set_K_D(Motor_Reload_6020_Anlge_D_test);

    // //角度环测试
    // Motor_Pull.Set_Target_Radian(Target_test_b_pull);
    // Motor_Push_L.Set_Target_Radian(0.95f);
    // Motor_Push_R.Set_Target_Radian(0.95f);

    // Motor_Push_L.Set_Target_Radian(target_position_b);
    // Motor_Push_R.Set_Target_Radian(target_position_b);
    // Motor_Reload_Linear.Set_Target_Radian(target_position_b);
    // Motor_Reload_Angle.Set_Target_Angle(test_angle_reload_multi_turn);

    // //速度环调参
    // Motor_Pull.PID_Omega.Set_K_P(Motor_Pull_Omega_test_P);
    // Motor_Pull.PID_Omega.Set_K_I(Motor_Pull_Omega_test_I);
    // Motor_Push_L.PID_Omega.Set_K_P(Motor_L_test_P);
    // Motor_Push_R.PID_Omega.Set_K_P(Motor_R_test_P);
    // Motor_Push_L.PID_Omega.Set_K_I(Motor_L_test_I);
    // Motor_Push_R.PID_Omega.Set_K_I(Motor_R_test_I);
    // Motor_Reload_Linear.PID_Omega.Set_K_P(Motor_Reload_C610_Omega_P_test);
    // Motor_Reload_Linear.PID_Omega.Set_K_I(Motor_Reload_C610_Omega_I_test);
    // Motor_Reload_Angle.PID_Omega.Set_K_P(Motor_Reload_6020_Omega_P_test);
    // Motor_Reload_Angle.PID_Omega.Set_K_I(Motor_Reload_6020_Omega_I_test);

    // //速度环测试
    // Motor_Pull.Set_Target_Omega_Radian(test_b);
    // Motor_Pull.Set_Transform_Angle(Motor_Pull.Get_Now_Radian());
    // Motor_Push_L.Set_Target_Omega_Radian(test_b);
    // Motor_Push_L.Set_Transform_Angle(Motor_Push_L.Get_Now_Radian());
    // Motor_Push_R.Set_Target_Omega_Radian(test_b);
    // Motor_Push_R.Set_Transform_Angle(Motor_Push_R.Get_Now_Radian());
    // Motor_Reload_Linear.Set_Target_Omega_Radian(test_b);
    // Motor_Reload_Linear.Set_Transform_Angle(Motor_Reload_Linear.Get_Now_Radian());
    // Motor_Reload_Linear.Set_Target_Omega_Radian(test_b);
    // Motor_Reload_Angle.Set_Transform_Angle(Motor_Reload_Angle.Get_Now_Angle());
    // Motor_Reload_Angle.Set_Target_Omega_Radian(test_b);

    switch (Booster_Control_Type)
    {
    case (Booster_Control_Type_DISABLE):
    {
        // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        // Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

        // Motor_Pull.Set_Target_Torque(0.f);
        // Motor_Push_L.Set_Target_Torque(0.f);
        // Motor_Push_R.Set_Target_Torque(0.f);
        // Motor_Reload_Linear.Set_Target_Torque(0.0f);
    }
    break;
    case (Booster_Control_Type_NORMAL): // 校准结束，进入正常控制状态（发射/换弹等）
    {
        // // 其实这里都可以不要 这个out_put没啥用 就调试的时候用用
        // // 东西都在状态机里面跑了
        // //  //-----------------------
        // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Motor_Reload_Linear.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        // Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

        // Motor_Push_L.Set_Target_Radian(test_position_b);
        // Motor_Push_R.Set_Target_Radian(test_position_b);

        // Motor_Reload_Linear.Set_Target_Radian(-test_b); // 注意：这个电机的正反转和位置定义相反，所以要取负值
        // Motor_Reload_Angle.Set_Target_Angle(init_position_reload_angle + 40.0f * PI / 180.0f );

    }
    break;
    }
}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{
    
    PB3_GPIO = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET ? 1 : 0;
    PD7_GPIO = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_SET ? 1 : 0;
    PA5_GPIO = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET ? 1 : 0;//pull电机上侧微动开关
    //调试代码
    PE15_GPIO = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET ? 1 : 0;

    //实时记录裁判系统允许发射的次数
    Update_Referee_Allow_Edge();

    // bool force_stop_by_referee_limit = referee_allow_overlimit_stop || (referee_allow_rise_cnt > REFEREE_ALLOW_SHOOT_MAX_COUNT);
    // if (force_stop_by_referee_limit)
    // {
    //     referee_allow_overlimit_stop = true;
    //     Referee_Allow_Shoot = false;
    // }

    if(enable_booster_flag == 1)
    {
        // if (!claw_closed_for_calibration)//这里是夹爪舵机的控制：只有校准开始的时候才闭合（开机15s）
        // {
        //     for (uint8_t i = 0; i < 3; i++)
        //     {
        //         Servo_Claw[i].Set_Target_Angle(claw_close_angle[i]);
        //     }
        //     claw_closed_for_calibration = true;
        // }

    // 拉力机数值更新
    Measured_Tension = TensionMeter.Get_Tension();

    // 皮筋校准
    FSM_Push_Calibration.Push_Calibration_TIM_Status_PeriodElapsedCallback();

    // 拉力校准
    FSM_Pull_Calibration.Pull_Calibration_TIM_Status_PeriodElapsedCallback();

    // 换弹流程已并入发射大状态机，不再单独调度 Reload FSM

    // 发射状态机
    FSM_Shooting.Shooting_TIM_Status_PeriodElapsedCallback();
    }
    else
    {
        // disable 或 Referee_Allow_Shoot 超限时：Push/Pull 停机，6020 角度电机锁定在安全角
        Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Reload_Angle.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

        Motor_Pull.Set_Target_Torque(0.f);
        Motor_Push_L.Set_Target_Torque(0.f);
        Motor_Push_R.Set_Target_Torque(0.f);

        // 始终回到并保持在换弹初始安全角
        Motor_Reload_Angle.Set_Target_SingleTurn_Radian_Nearest(init_position_reload_angle);
        target_position_reload_angle = Motor_Reload_Angle.Get_Target_Radian();
    }

    Output();

    // PID输出
    Motor_Pull.TIM_PID_PeriodElapsedCallback();
    Motor_Push_L.TIM_PID_PeriodElapsedCallback();
    Motor_Push_R.TIM_PID_PeriodElapsedCallback();
    Motor_Reload_Angle.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

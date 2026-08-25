/**
 * @file crt_booster.cpp
 * @brief Dart booster control.
 */

/* Includes ------------------------------------------------------------------*/

#include "crt_booster.h"
#include <float.h>

/* Private constants ---------------------------------------------------------*/

#define SCREW_LEAD 0.004f

static constexpr float kPullPrepTarget = 0.39f;
static constexpr uint16_t kShootCooldownMs = 300;
static constexpr uint16_t kDownLockServoCloseDelayMs = 500;
static constexpr uint16_t kFireHoldMs = 500;
static constexpr uint16_t kPrepTimeoutMs = 4000;
static constexpr float kPushDownOmega = -320.0f;
static constexpr float kPushUpOmega = 370.0f;
static constexpr float kPushBackoffOmega = -20.0f;
static constexpr float kPushBackoffDistance = 0.006f;
static constexpr float kPullReadyTolerance = 0.005f;

/* GPIO snapshots ------------------------------------------------------------*/

int PB3_GPIO = 0;
int PD7_GPIO = 0;
int PA5_GPIO = 0;

/* EXTI event latches --------------------------------------------------------*/

static volatile bool pb3_press_event_latched = false;
volatile uint32_t pb3_exti_irq_count = 0;
volatile uint32_t pb3_event_consumed_count = 0;

static volatile bool pd7_press_event_latched = false;
volatile uint32_t pd7_exti_irq_count = 0;
volatile uint32_t pd7_event_consumed_count = 0;

static volatile bool pa5_press_event_latched = false;
volatile uint32_t pa5_exti_irq_count = 0;
volatile uint32_t pa5_event_consumed_count = 0;

/* Cross-module state --------------------------------------------------------*/

uint8_t enable_booster_flag = 0;
bool Push_Calibration_Finished = false;
bool Pull_Calibration_Finished = false;
bool Referee_Allow_Shoot = false;

Class_MiniPC *MiniPC_For_Booster = nullptr;
Class_Referee *Referee_For_Booster = nullptr;

/* Shooting flow state -------------------------------------------------------*/

int dart_fired_count = 0;
float pull_hold_after_calib_pos = 0.9f;

static bool minipc_fallback_prev = false;
static bool referee_allow_prev = false;
static uint32_t shoot_cmd_token = 0;
static uint32_t shoot_cmd_token_consumed = 0;
static bool shooting_cycle_active = false;

static bool Pull_Is_Finite(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

static float Pull_Clamp_Unit_Ratio(float ratio)
{
    if (ratio < 0.0f)
    {
        return 0.0f;
    }
    if (ratio > 1.0f)
    {
        return 1.0f;
    }
    return ratio;
}

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
    else if (shoot_cooldown_ms < kShootCooldownMs)
    {
        shoot_cooldown_ms++;
    }

    bool is_idle_ready = Push_Calibration_Finished && Pull_Calibration_Finished
                         && (enable_booster_flag == 1)
                         && !shooting_cycle_active
                         && !Referee_Allow_Shoot
                         && (shoot_cooldown_ms >= kShootCooldownMs);

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
    if (Referee_Allow_Shoot && !referee_allow_prev)
    {
        shoot_cmd_token++;
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

extern "C" void Booster_On_PA5_Exti(void)
{
    pa5_exti_irq_count++;
    pa5_press_event_latched = true;
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

float Class_FSM_Push_Calibration::Linear_Map_Position(float curr_angle,
                                                       float angle_start,
                                                       float angle_end,
                                                       float max_length)
{
    if (fabs(angle_end - angle_start) < 0.001f)
    {
        return 0.0f;
    }

    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }
    return ratio * max_length;
}

float Class_FSM_Pull_Calibration::Linear_Map_Position(float curr_angle,
                                                       float angle_start,
                                                       float angle_end,
                                                       float max_length)
{
    if (fabs(angle_end - angle_start) < 0.001f)
    {
        return 0.0f;
    }

    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    if (ratio > 1.0f)
    {
        ratio = 1.0f;
    }
    return ratio * max_length;
}

void Class_Booster::Set_Pull_Stroke_Ratio(float stroke_ratio)
{
    if (!Pull_Is_Finite(stroke_ratio))
    {
        return;
    }

    target_position_pull = Pull_Clamp_Unit_Ratio(stroke_ratio);
}

void Class_FSM_Push_Calibration::Push_Calibration_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
    case (0):
    {
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_L.Set_Target_Omega_Radian(speed);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(speed);

        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PD7_Press_Event();
        }

        if (PD7_GPIO == 1 && Consume_PD7_Press_Event())
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Angle_Forward_L = Booster->Motor_Push_L.Get_Now_Angle();
            Angle_Forward_R = Booster->Motor_Push_R.Get_Now_Angle();
            Set_Status(1);
        }
    }
    break;

    case (1):
    {
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_L.Set_Target_Omega_Radian(-speed);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(-speed);

        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PB3_Press_Event();
        }

        if (PB3_GPIO == 1 || Consume_PB3_Press_Event())
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Angle_Backward_L = Booster->Motor_Push_L.Get_Now_Angle();
            Angle_Backward_R = Booster->Motor_Push_R.Get_Now_Angle();
            Set_Status(2);
        }
    }
    break;

    case (2):
    {
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_L.Set_Target_Omega_Radian(240.0f);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(240.0f);

        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PD7_Press_Event();
        }

        if (PD7_GPIO == 1 || Consume_PD7_Press_Event())
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
            Push_Calibration_Finished = true;
            Set_Status(3);
        }
    }
    break;

    case (3):
    {
        float now_position_l = Linear_Map_Position(Booster->Motor_Push_L.Get_Now_Angle(), Angle_Backward_L, Angle_Forward_L, 1.0f);
        float now_position_r = Linear_Map_Position(Booster->Motor_Push_R.Get_Now_Angle(), Angle_Backward_R, Angle_Forward_R, 1.0f);
        float now_position = (now_position_l + now_position_r) / 2.0f;
        Booster->Set_Now_position_push(now_position);
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

    case (4):
    {
        float now_position_l = Linear_Map_Position(Booster->Motor_Push_L.Get_Now_Angle(), Angle_Backward_L, Angle_Forward_L, 1.0f);
        float now_position_r = Linear_Map_Position(Booster->Motor_Push_R.Get_Now_Angle(), Angle_Backward_R, Angle_Forward_R, 1.0f);
        float now_position = (now_position_l + now_position_r) / 2.0f;
        Booster->Set_Now_position_push(now_position);
        Booster->Motor_Push_L.Set_Transform_Angle(now_position);
        Booster->Motor_Push_R.Set_Transform_Angle(now_position);
    }
    break;

    default:
    {
        Set_Status(0);
    }
    break;
    }
}

void Class_FSM_Pull_Calibration::Pull_Calibration_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    switch (Now_Status_Serial)
    {
    case (0):
    {
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Pull.Set_Target_Omega_Radian(speed);

        if (PA5_GPIO == 1 || Consume_PA5_Press_Event())
        {
            Angle_Forward = Booster->Motor_Pull.Get_Now_Angle();
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Pull.Set_Target_Omega_Radian(0.0f);
            Set_Status(1);
        }
    }
    break;

    case (1):
    {
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Pull.Set_Target_Omega_Radian(-speed);

        if (fabs(Booster->Motor_Pull.Get_Now_Torque()) > Torque_Threshold)
        {
            Set_Status(2);
        }
    }
    break;

    case (2):
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

    case (3):
    {
        Pull_Calibration_Finished = true;
        Set_Status(4);
    }
    break;

    case (4):
    {
        if (Status[Now_Status_Serial].Time == 1)
        {
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Pull.Set_Target_Radian(pull_hold_after_calib_pos);
        }

        float now_position = Linear_Map_Position(Booster->Motor_Pull.Get_Now_Angle(), Angle_Backward, Angle_Forward, 1.0f);
        Booster->Set_Now_position_pull(now_position);
        Booster->Motor_Pull.Set_Transform_Angle(now_position);

        if (Push_Calibration_Finished && Pull_Calibration_Finished)
        {
            Booster->Set_Booster_Control_Type(Booster_Control_Type_NORMAL);
        }
    }
    break;

    default:
    {
        Set_Status(0);
    }
    break;
    }
}

void Class_FSM_Shooting::Shooting_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    if (Booster->Get_Booster_Control_Type() != Booster_Control_Type_NORMAL)
    {
        shooting_cycle_active = false;
        Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle);
        if (Now_Status_Serial != Shooting_Control_Type_IDLE)
        {
            Set_Status(Shooting_Control_Type_IDLE);
        }
        Shooting_Control_Type = static_cast<Enum_Shooting_Control_Type>(Now_Status_Serial);
        return;
    }

    switch (Now_Status_Serial)
    {
    case (Shooting_Control_Type_IDLE):
    {
        Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle);
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Pull.Set_Target_Radian(pull_hold_after_calib_pos);

        if (Status[Now_Status_Serial].Time == 1)
        {
            prep_task_b_done = false;
            prep_task_c_done = false;
            push_top_reached = false;
            down_lock_stage = 0;
            down_lock_close_start_time = 0;
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
            Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle);
            down_lock_stage = 0;
            down_lock_close_start_time = 0;
        }

        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Pull.Set_Target_Radian(pull_hold_after_calib_pos);

        if (down_lock_stage == 0)
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(kPushDownOmega);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(kPushDownOmega);

            if (PB3_GPIO == 1 || Consume_PB3_Press_Event())
            {
                Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
                Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
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
                Set_Status(Shooting_Control_Type_PREP_FIRE);
            }
        }
    }
    break;

    case (Shooting_Control_Type_PREP_FIRE):
    {
        Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle);

        if (Status[Now_Status_Serial].Time == 1)
        {
            (void)Consume_PD7_Press_Event();
            prep_task_b_done = false;
            prep_task_c_done = false;
            push_top_reached = false;
        }

        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        if (!push_top_reached)
        {
            Booster->Motor_Push_L.Set_Target_Omega_Radian(kPushUpOmega);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(kPushUpOmega);

            if (PD7_GPIO == 1 && Consume_PD7_Press_Event())
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

        Booster->Set_Pull_Stroke_Ratio(kPullPrepTarget);
        Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        Booster->Motor_Pull.Set_Target_Radian(Booster->Get_Target_Pull_Stroke_Ratio());

        if (fabs(Booster->Motor_Pull.Get_Now_Radian() - Booster->Get_Target_Pull_Stroke_Ratio()) < kPullReadyTolerance)
        {
            prep_task_c_done = true;
        }
        // 超时保护：超过 4 秒未到位则强制放行
        if (Status[Now_Status_Serial].Time > kPrepTimeoutMs)
        {
            prep_task_c_done = true;
        }

        if (prep_task_b_done && prep_task_c_done)
        {
            Set_Status(Shooting_Control_Type_READY);
        }
    }
    break;

    case (Shooting_Control_Type_READY):
    {
        Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle);
        Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
        Booster->Motor_Push_L.Set_Target_Omega_Radian(0.0f);
        Booster->Motor_Push_R.Set_Target_Omega_Radian(0.0f);
        Set_Status(Shooting_Control_Type_FIRE);
    }
    break;

    case (Shooting_Control_Type_FIRE):
    {
        if (Status[Now_Status_Serial].Time == 1)
        {
            Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_fire_angle);
        }

        if (Status[Now_Status_Serial].Time > kFireHoldMs)
        {
            dart_fired_count += 1;
            shooting_cycle_active = false;
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

float Motor_L_test_P = 270.0f;
float Motor_L_test_I = 25.0f;
float Motor_R_test_P = 320.0f;
float Motor_R_test_I = 28.0f;

float Motor_Push_Angle_P_test = 4200.0f;
float Motor_Push_Angle_I_test = 0.0f;

float Motor_Pull_Omega_test_P = 900.0f;
float Motor_Pull_Omega_test_I = 145.0f;
float Motor_Pull_Angle_P_test = 2200.0f;
float Motor_Pull_Angle_I_test = 0.0f;

void Class_Booster::Init()
{
    FSM_Shooting.Booster = this;
    FSM_Shooting.Init(6, 0);//状态机上限是10

    FSM_Push_Calibration.Booster = this;
    FSM_Push_Calibration.Init(6, 0);

    FSM_Pull_Calibration.Booster = this;
    FSM_Pull_Calibration.Init(6, 0);

    Servo_Trigger.Init(&htim2, TIM_CHANNEL_1, 270);
    Servo_Trigger.Set_Target_Angle(tirrger_fire_angle);

    Motor_Pull.PID_Angle.Init(Motor_Pull_Angle_P_test, Motor_Pull_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 130.0f * PI);
    Motor_Pull.PID_Omega.Init(Motor_Pull_Omega_test_P, Motor_Pull_Omega_test_I, 0.0f, 0.0f, 2000, Motor_Pull.Get_Output_Max());
    Motor_Pull.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);

    Motor_Push_L.PID_Angle.Init(Motor_Push_Angle_P_test, Motor_Push_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Push_L.PID_Omega.Init(Motor_L_test_P, Motor_L_test_I, 0.0f, 0.0f, Motor_Push_L.Get_Output_Max() * 0.5f, Motor_Push_L.Get_Output_Max());
    Motor_Push_L.Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA, 1.0f);

    Motor_Push_R.PID_Angle.Init(Motor_Push_Angle_P_test, Motor_Push_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Push_R.PID_Omega.Init(Motor_R_test_P, Motor_R_test_I, 0.0f, 0.0f, Motor_Push_R.Get_Output_Max() * 0.5f, Motor_Push_R.Get_Output_Max());
    Motor_Push_R.Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA, 1.0f);
}

/**
 * @brief 输出到电机
 *
 */
void Class_Booster::Output()
{
    //此处无内容
}

void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{
    PB3_GPIO = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET ? 1 : 0;
    PD7_GPIO = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_7) == GPIO_PIN_SET ? 1 : 0;
    PA5_GPIO = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET ? 1 : 0;//pull电机上侧微动开关

    Update_Referee_Allow_Edge();

    if (enable_booster_flag == 1)
    {
        FSM_Push_Calibration.Push_Calibration_TIM_Status_PeriodElapsedCallback();
        FSM_Pull_Calibration.Pull_Calibration_TIM_Status_PeriodElapsedCallback();
        FSM_Shooting.Shooting_TIM_Status_PeriodElapsedCallback();
    }
    else
    {
        // disable 或 Referee_Allow_Shoot 超限时：Push/Pull 停机，6020 角度电机锁定在安全角
        shooting_cycle_active = false;
        Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
        Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

        Motor_Pull.Set_Target_Torque(0.0f);
        Motor_Push_L.Set_Target_Torque(0.0f);
        Motor_Push_R.Set_Target_Torque(0.0f);
        Servo_Trigger.Set_Target_Angle(tirrger_fire_angle);
    }

    Output();

    // PID输出
    Motor_Pull.TIM_PID_PeriodElapsedCallback();
    Motor_Push_L.TIM_PID_PeriodElapsedCallback();
    Motor_Push_R.TIM_PID_PeriodElapsedCallback();
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

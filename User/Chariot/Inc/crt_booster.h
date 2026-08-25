/**
 * @file crt_booster.h
 * @brief Dart booster control.
 */

#ifndef CRT_BOOSTER_H
#define CRT_BOOSTER_H

/* Includes ------------------------------------------------------------------*/

#include "alg_fsm.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "dvc_minipc.h"
#include "dvc_servo.h"

/* Exported types ------------------------------------------------------------*/

class Class_Booster;

enum Enum_Booster_Control_Type
{
    Booster_Control_Type_DISABLE = 0,
    Booster_Control_Type_NORMAL,
    Booster_Control_Type_Push_CALIBRATION,
    Booster_Control_Type_Pull_CALIBRATION,
};

enum Enum_Shooting_Control_Type
{
    Shooting_Control_Type_IDLE = 0,
    Shooting_Control_Type_DOWN_LOCK,
    Shooting_Control_Type_PREP_FIRE,
    Shooting_Control_Type_READY,
    Shooting_Control_Type_FIRE,
};

class Class_FSM_Shooting : public Class_FSM
{
public:
    Class_Booster *Booster;

    void Shooting_TIM_Status_PeriodElapsedCallback();
    Enum_Shooting_Control_Type Shooting_Control_Type = Shooting_Control_Type_IDLE;

    bool prep_task_b_done = false;
    bool prep_task_c_done = false;
    bool push_top_reached = false;
    float prep_top_position_meter = 0.0f;
    uint8_t down_lock_stage = 0;
    uint16_t down_lock_close_start_time = 0;
};

class Class_FSM_Push_Calibration : public Class_FSM
{
public:
    Class_Booster *Booster;

    float Torque_Threshold_up = 3000.0f;
    float Torque_Threshold_down = 9000.0f;
    float speed = 60.0f;

    float Angle_Forward_L = 0.0f;
    float Angle_Backward_L = 0.0f;
    float Angle_Forward_R = 0.0f;
    float Angle_Backward_R = 0.0f;

    int forward_flag_L = 0;
    int forward_flag_R = 0;
    int backward_flag_L = 0;
    int backward_flag_R = 0;

    void Push_Calibration_TIM_Status_PeriodElapsedCallback();
    float Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length);
};

class Class_FSM_Pull_Calibration : public Class_FSM
{
public:
    Class_Booster *Booster;

    float Torque_Threshold = 2500.0f;
    float speed = 40.0f;

    float Angle_Forward = 0.0f;
    float Angle_Backward = 0.0f;

    void Pull_Calibration_TIM_Status_PeriodElapsedCallback();
    float Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length);
};

class Class_Booster
{
public:
    Class_FSM_Shooting FSM_Shooting;
    friend class Class_FSM_Shooting;

    Class_FSM_Push_Calibration FSM_Push_Calibration;
    friend class Class_FSM_Push_Calibration;

    Class_FSM_Pull_Calibration FSM_Pull_Calibration;
    friend class Class_FSM_Pull_Calibration;

    Class_Referee *Referee;
    Class_MiniPC *MiniPC;

    Class_Servo Servo_Trigger;

    Class_DJI_Motor_C620 Motor_Pull;
    Class_DJI_Motor_C620 Motor_Push_L;
    Class_DJI_Motor_C620 Motor_Push_R;

    void Init();

    inline Enum_Booster_Control_Type Get_Booster_Control_Type();
    inline Enum_Shooting_Control_Type Get_Shooting_Control_Type();

    inline int Get_Target_PushMotor_Angle();
    inline int Get_Target_PullMotor_Angle();
    inline float Get_Target_Pull_Stroke_Ratio();
    inline float Get_Target_position_push();
    inline float Get_Now_position_push();
    inline float Get_Now_position_pull();

    inline void Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);
    inline void Set_Shooting_Control_Type(Enum_Shooting_Control_Type __Shooting_Control_Type);
    inline void Set_Target_PushMotor_Angle(float __Target_PushMotor_Angle);
    inline void Set_Target_PullMotor_Angle(float __Target_PullMotor_Angle);
    void Set_Pull_Stroke_Ratio(float stroke_ratio);
    inline void Set_Target_position_push(float __target_position_push);
    inline void Set_Now_position_push(float __now_position_push);
    inline void Set_Now_position_pull(float __now_position_pull);

    void TIM_Calculate_PeriodElapsedCallback();
    void Output();

protected:
    float target_position_push = 0.95f;
    float target_position_pull = 0.5f;
    // Pull 位置坐标始终为 0=底部、1=顶部。

    float now_position_push = 0.0f;
    float now_position_pull = 0.0f;

    float tirrger_fire_angle = 90.0f;
    float tirrger_reset_angle = 210.0f;

    Enum_Booster_Control_Type Booster_Control_Type = Booster_Control_Type_DISABLE;

    float Target_PushMotor_Angle = 0.0f;
    float Target_PullMotor_Angle = 0.0f;
};

/* Inline functions ----------------------------------------------------------*/

inline Enum_Booster_Control_Type Class_Booster::Get_Booster_Control_Type()
{
    return Booster_Control_Type;
}

inline Enum_Shooting_Control_Type Class_Booster::Get_Shooting_Control_Type()
{
    return FSM_Shooting.Shooting_Control_Type;
}

inline int Class_Booster::Get_Target_PushMotor_Angle()
{
    return Target_PushMotor_Angle;
}

inline int Class_Booster::Get_Target_PullMotor_Angle()
{
    return Target_PullMotor_Angle;
}

inline float Class_Booster::Get_Target_Pull_Stroke_Ratio()
{
    return target_position_pull;
}

inline float Class_Booster::Get_Target_position_push()
{
    return target_position_push;
}

inline float Class_Booster::Get_Now_position_push()
{
    return now_position_push;
}

inline float Class_Booster::Get_Now_position_pull()
{
    return now_position_pull;
}

inline void Class_Booster::Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
{
    Booster_Control_Type = __Booster_Control_Type;
}

inline void Class_Booster::Set_Shooting_Control_Type(Enum_Shooting_Control_Type __Shooting_Control_Type)
{
    FSM_Shooting.Shooting_Control_Type = __Shooting_Control_Type;
}

inline void Class_Booster::Set_Target_PushMotor_Angle(float __Target_PushMotor_Angle)
{
    Target_PushMotor_Angle = __Target_PushMotor_Angle;
}

inline void Class_Booster::Set_Target_PullMotor_Angle(float __Target_PullMotor_Angle)
{
    Target_PullMotor_Angle = __Target_PullMotor_Angle;
}

inline void Class_Booster::Set_Target_position_push(float __target_position_push)
{
    target_position_push = __target_position_push;
}

inline void Class_Booster::Set_Now_position_push(float __now_position_push)
{
    now_position_push = __now_position_push;
}

inline void Class_Booster::Set_Now_position_pull(float __now_position_pull)
{
    now_position_pull = __now_position_pull;
}

#endif /* CRT_BOOSTER_H */

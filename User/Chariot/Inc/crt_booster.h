/**
 * @file crt_booster.h
 * @author cjw
 * @brief 发射机构
 * @version 0.1
 * @date 2025-07-1 0.1 26赛季定稿
 *
 * @copyright ZLLC 2026
 *
 */

/**
 * @brief 摩擦轮编号
 * 1 2
 */

#ifndef CRT_BOOSTER_H
#define CRT_BOOSTER_H

/* Includes ------------------------------------------------------------------*/

#include "alg_fsm.h"
#include "dvc_referee.h"
#include "dvc_djimotor.h"
#include "dvc_minipc.h"
#include "dvc_servo.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

class Class_Booster;

/**
 * @brief 发射机构控制类型
 *
 */
enum Enum_Booster_Control_Type
{
    Booster_Control_Type_DISABLE = 0,
    Booster_Control_Type_NORMAL,//暂时不用
    Booster_Control_Type_Push_CALIBRATION,
    Booster_Control_Type_Pull_CALIBRATION,
    Booster_Control_Type_Shooting,
    
};

/**
 * @brief 发射过程控制类型
 *
 */
enum Enum_Shooting_Control_Type
{
    //Booster_Control_Type_DISABLE = 0,
    // Booster_Control_Type_READY,//可以发射的状态，初始化之后的状态。PUSH和PULL电机位置保持在目标位置
    // Booster_Control_Type_READY_Tension,//进一步的状态，PUSH电机位置保持在目标位置，PULL电机进入拉力环
    // Booster_Control_Type_DART_IN_FLIGHT,//发射中状态1,此时Push上膛压块在上（1的位置）飞镖在轨道内滑行。
    // Booster_Control_Type_DART_IN_AIR,//发射中状态2,此时Push上膛压块正在下降（正在从1位置回到0位置）飞镖在空中
    Shooting_Control_Type_DISABLE = 0,
    Shooting_Control_Type_READY,

};



/**
 * @brief Specialized, 发射策略有限自动机
 *
 */
class Class_FSM_Shooting : public Class_FSM
{
public:
    Class_Booster *Booster;

    void Reload_TIM_Status_PeriodElapsedCallback();
    Enum_Shooting_Control_Type Shooting_Control_Type = Shooting_Control_Type_DISABLE;

};

/**
 * @brief Specialized, 发射策略有限自动机
 *
 */
class Class_FSM_Push_Calibration  : public Class_FSM
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

    void Reload_TIM_Status_PeriodElapsedCallback();
    float Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length);
};

/**
 * @brief Specialized, 发射策略有限自动机
 *
 */
class Class_FSM_Pull_Calibration  : public Class_FSM
{
public:
    Class_Booster *Booster;

    float Torque_Threshold = 4000.0f;
    float speed = 10.0f;

    float Angle_Forward = 0.0f;
    float Angle_Backward = 0.0f;

    void Reload_TIM_Status_PeriodElapsedCallback();
    float Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length);
};

/**
 * @brief Specialized, 发射机构类
 *
 */
class Class_Booster
{
public:

    //发射有限自动机
    Class_FSM_Shooting FSM_Shooting;
    friend class Class_FSM_Shooting;

    //皮筋电机校准
    Class_FSM_Push_Calibration FSM_Push_Calibration;
    friend class Class_FSM_Push_Calibration;

    //拉力电机校准
    Class_FSM_Pull_Calibration FSM_Pull_Calibration;
    friend class Class_FSM_Pull_Calibration;

    //裁判系统
    Class_Referee *Referee;
    //上位机
    Class_MiniPC *MiniPC;

    //180°舵机->撒放器
    Class_Servo Servo_Trigger;

    //发射电机
    Class_DJI_Motor_C610 Motor_Pull;

    Class_DJI_Motor_C620 Motor_Push_L;
    Class_DJI_Motor_C620 Motor_Push_R;

    

    void Pull_Tension_Control();

    void Init();

    inline Enum_Booster_Control_Type Get_Booster_Control_Type();

    inline int Get_Target_PushMotor_Angle();
    inline int Get_Target_PullMotor_Angle();
    inline int Get_Measured_Tension();
    inline int Get_Target_Tension();
    inline float Get_Target_position_push();
    inline float Get_Target_position_pull();

    inline void Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type);
    inline void Set_Shooting_Control_Type(Enum_Shooting_Control_Type __Shooting_Control_Type);
    
    inline void Set_Target_PushMotor_Angle(float __Target_PushMotor_Angle);
    inline void Set_Target_PullMotor_Angle(float __Target_PullMotor_Angle);
    inline void Set_Measured_Tension(int __Measured_Tension);
    inline void Set_Target_Tension(int __Target_Tension);
    inline void Set_Target_position_push(float __target_position_push);
    inline void Set_Target_position_pull(float __target_position_pull);

    void TIM_Calculate_PeriodElapsedCallback();
	void Output();
		
protected:
    //初始化相关常量

    //校准完成标志位
    bool Push_Calibration_Finished = false;
    bool Pull_Calibration_Finished = false;

    float target_position_push = 0.01f;//校准完成后push电机目标位置
    float target_position_pull = 0.5f;//校准完成后pull电机目标位置

    //舵机相关
    float tirrger_fire_angle = 235.0f;//舵机发射角度
    float tirrger_reset_angle = 90.0f;//舵机复位角度

    //内部
    //拉力相关变量
    int Measured_Tension = 0;
    int Target_Tension = 0;

    //拉力环相关变量
    float now_tension_value = 0.0f;           // 当前测得的拉力值
    float target_tension_value = 0.0f;        // 目标拉力值    
    float target_tension_position_pull = target_position_pull;// 拉力环下 Pull 电机目标位置，初始由 target_position_pull 提供

    //发射机构状态
    Enum_Booster_Control_Type Booster_Control_Type = Booster_Control_Type_DISABLE;

    //读写变量
    float Target_PushMotor_Angle = 0.0f;

    float Target_PullMotor_Angle = 0.0f;

    //内部函数

    
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 获得发射机构状态
 *
 * @return Enum_Booster_Control_Type 发射机构状态
 */
Enum_Booster_Control_Type Class_Booster::Get_Booster_Control_Type()
{
    return (Booster_Control_Type);
}

int Class_Booster::Get_Target_PushMotor_Angle()
{
    return Target_PushMotor_Angle;
}

int Class_Booster::Get_Target_PullMotor_Angle()
{
    return Target_PullMotor_Angle;
}

/**
 * @brief 获取当前拉力,
 *
 * @return int 获取当前拉力
 */
inline int Class_Booster::Get_Measured_Tension()
{
    return (Measured_Tension);
}

inline int Class_Booster::Get_Target_Tension()
{
    return (Target_Tension);
}

inline float Class_Booster::Get_Target_position_push()
{
    return (target_position_push);
}

inline float Class_Booster::Get_Target_position_pull()
{
    return (target_position_pull);
}

/**
 * @brief 设定发射机构状态
 *
 * @param __Booster_Control_Type 发射机构状态
 */
void Class_Booster::Set_Booster_Control_Type(Enum_Booster_Control_Type __Booster_Control_Type)
{
    Booster_Control_Type = __Booster_Control_Type;
}

/**
 * @brief 设定发射过程控制状态
 *
 * @param __Shooting_Control_Type 发射过程控制状态
 */
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

/**
 * @brief 设定测量拉力
 *
 * @param __Measured_Tension 测量拉力
 */
void Class_Booster::Set_Measured_Tension(int __Measured_Tension)
{
    Measured_Tension = __Measured_Tension;
}

/**
 * @brief 设置目标拉力,
 *
 * @return int 设置目标拉力
 */
void Class_Booster::Set_Target_Tension(int __Target_Tension)
{
    Target_Tension = __Target_Tension;
}

inline void Class_Booster::Set_Target_position_push(float __target_position_push)
{
    target_position_push = __target_position_push;
}

inline void Class_Booster::Set_Target_position_pull(float __target_position_pull)
{
    target_position_pull = __target_position_pull;
}

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

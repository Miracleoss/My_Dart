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
    extern bool Push_Calibration_Finished = false;
    extern bool Pull_Calibration_Finished = true;

    //int servo_test;
    int servo_test_flag = 0;
    float test_0_1_push = 0.01f;

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief  将当前角度线性映射到目标行程
 * @param  curr_angle   当前电机角度
 * @param  angle_start  起始点角度（通常是 Backward 角度）
 * @param  angle_end    结束点角度（通常是 Forward 角度）
 * @param  max_length   物理最大行程（例如 1.0 表示百分比，或者 200.0 表示 mm）
 * @return 映射后的位置值
 */
float Class_FSM_Push_Calibration::Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length)
{
    // 防止分母为0（极其罕见的情况，但为了安全）
    if (fabs(angle_end - angle_start) < 0.001f) {
        return 0.0f;
    }

    // 1. 计算归一化比例 (Ratio 0.0 ~ 1.0)
    // 公式: (x - min) / (max - min)
    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);

    // 2. 安全限幅 (Clamping)
    // 这一步非常重要：如果当前角度因为惯性稍微超过了校准值，
    // 不限幅会导致 PID 计算出的误差反向剧增，引发震荡。
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    // 3. 映射到物理长度
    return ratio * max_length;
}

float Class_FSM_Pull_Calibration::Linear_Map_Position(float curr_angle, float angle_start, float angle_end, float max_length)
{
    // 防止分母为0（极其罕见的情况，但为了安全）
    if (fabs(angle_end - angle_start) < 0.001f) {
        return 0.0f;
    }

    // 1. 计算归一化比例 (Ratio 0.0 ~ 1.0)
    // 公式: (x - min) / (max - min)
    float ratio = (curr_angle - angle_start) / (angle_end - angle_start);

    // 2. 安全限幅 (Clamping)
    // 这一步非常重要：如果当前角度因为惯性稍微超过了校准值，
    // 不限幅会导致 PID 计算出的误差反向剧增，引发震荡。
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;

    // 3. 映射到物理长度
    return ratio * max_length;
}

// 已经通过串口读取到一拉力值
// 使用全局变量保存，单位为kg

// 拉力环比例系数
float K_tension = 0.01f; 
/**
 * @brief 拉力外环控制（将拉力误差映射为 Pull 电机的目标位置）
 *
 */
void Class_Booster::Pull_Tension_Control()
{
    // 读取测量值与目标值
    now_tension_value = Get_Measured_Tension();
    target_tension_value = Get_Target_Tension();

    float tension_error = target_tension_value - now_tension_value;
    target_tension_position_pull += K_tension * tension_error;

    // 限幅
    if (target_tension_position_pull > 0.9f) {
        target_tension_position_pull = 0.9f;
    }
    if (target_tension_position_pull < 0.1f) {
        target_tension_position_pull = 0.1f;
    }

    Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    Motor_Pull.Set_Target_Radian(target_tension_position_pull);

}



void Class_FSM_Push_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0)://向前堵转
        {
            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            Booster->Motor_Push_L.Set_Target_Omega_Radian(speed);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(speed);

            if(fabs(Booster->Motor_Push_L.Get_Now_Torque()) > Torque_Threshold_up &&
                fabs(Booster->Motor_Push_R.Get_Now_Torque()) > Torque_Threshold_up){
                Set_Status(1);
            }
        }
        break;
        case (1)://前侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Forward_L = Booster->Motor_Push_L.Get_Now_Angle();
                Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Push_L.Set_Target_Torque(0.f);
                Booster->Motor_Push_L.Set_Out(0.f);
                forward_flag_L = 1;
            }
            else if (fabs(Booster->Motor_Push_L.Get_Now_Torque()) < Torque_Threshold_up){
                Set_Status(0);
                forward_flag_L = 0;
                forward_flag_R = 0;
            }
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Forward_R = Booster->Motor_Push_R.Get_Now_Angle();
                Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Push_R.Set_Target_Torque(0.f);
                Booster->Motor_Push_R.Set_Out(0.f);
                forward_flag_R = 1;
            }
            else if (fabs(Booster->Motor_Push_R.Get_Now_Torque()) < Torque_Threshold_up){
                Set_Status(0);
                forward_flag_L = 0;
                forward_flag_R = 0;
            }
            if(forward_flag_L == 1 && forward_flag_R == 1){
                Set_Status(2);
            }
        }
        break;
        case (2)://向后堵转
        {
            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

            Booster->Motor_Push_L.Set_Target_Omega_Radian(-speed);
            Booster->Motor_Push_R.Set_Target_Omega_Radian(-speed);

            if(fabs(Booster->Motor_Push_L.Get_Now_Torque()) > Torque_Threshold_down && 
                fabs(Booster->Motor_Push_R.Get_Now_Torque()) > Torque_Threshold_down){
                Set_Status(3);
            }
        }
        break;
        case (3)://后侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Backward_L = Booster->Motor_Push_L.Get_Now_Angle();
                Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Push_L.Set_Target_Torque(0.f);
                Booster->Motor_Push_L.Set_Out(0.f);
                backward_flag_L = 1;
            }
            else if (fabs(Booster->Motor_Push_L.Get_Now_Torque()) < Torque_Threshold_down){
                Set_Status(2);
                backward_flag_L = 0;
                backward_flag_R = 0;
            }
            
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Backward_R = Booster->Motor_Push_R.Get_Now_Angle();
                Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Push_R.Set_Target_Torque(0.f);
                Booster->Motor_Push_R.Set_Out(0.f);
                backward_flag_R = 1;
            }
            else if (fabs(Booster->Motor_Push_R.Get_Now_Torque()) < Torque_Threshold_down){
                Set_Status(2);
                backward_flag_L = 0;
                backward_flag_R = 0;
            }
            if(backward_flag_L == 1 && backward_flag_R == 1){
                //舵机控制
                Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle);//测试舵机用
                Set_Status(4);
            }
        }
        break;
        case (4)://正常控制流程
        {
            //舵机控制
            Booster->Servo_Trigger.Set_Target_Angle(Booster->tirrger_reset_angle);
            Push_Calibration_Finished = true;
            Set_Status(5);
        }
        break;
        case (5)://校准检测
        {
            //定义上面是1.0f最大行程 下面是0.0f最小行程
            //所以在函数映射的时候颠倒了位置
            float now_position_l = Linear_Map_Position(Booster->Motor_Push_L.Get_Now_Angle(), Angle_Backward_L, Angle_Forward_L,1.0f);
            float now_position_r = Linear_Map_Position(Booster->Motor_Push_R.Get_Now_Angle(), Angle_Backward_R, Angle_Forward_R,1.0f);
            float now_position = (now_position_l + now_position_r) / 2.0f;
            Booster->Motor_Push_L.Set_Transform_Angle(now_position);
            Booster->Motor_Push_R.Set_Transform_Angle(now_position);

            if(Push_Calibration_Finished && Pull_Calibration_Finished){
                Booster->Set_Booster_Control_Type(Booster_Control_Type_NORMAL);
            }

        }
    }
}

void Class_FSM_Pull_Calibration::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (0)://向前堵转
        {
            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Pull.Set_Target_Omega_Radian(speed);
            
            if(fabs(Booster->Motor_Pull.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(1);
            }
        }
        break;
        case (1)://前侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Forward = Booster->Motor_Pull.Get_Now_Angle();
                Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Pull.Set_Target_Torque(0.f);
                Booster->Motor_Pull.Set_Out(0.f);
                Set_Status(2);
            }
            else if (fabs(Booster->Motor_Pull.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(0);
            }
        }
        break;
        case (2)://向后堵转
        {

            Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
            Booster->Motor_Pull.Set_Target_Omega_Radian(-speed);

            if(fabs(Booster->Motor_Pull.Get_Now_Torque()) > Torque_Threshold){
                Set_Status(3);
            }
        }
        break;
        case (3)://后侧检测
        {
            if(Status[Now_Status_Serial].Time > 100){
                Angle_Backward = Booster->Motor_Pull.Get_Now_Angle();
                Booster->Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
                Booster->Motor_Pull.Set_Target_Torque(0.f);
                Booster->Motor_Pull.Set_Out(0.f);
                Set_Status(4);
            }
            else if (fabs(Booster->Motor_Pull.Get_Now_Torque()) < Torque_Threshold){
                Set_Status(2);
            }
            
        }
        break;
        case (4)://正常控制流程
        {
            Pull_Calibration_Finished = true;
            Set_Status(5);
        }
        break;
        case (5)://校准检测
        {
            //定义上面是1.0f最大行程 下面是0.0f最小行程
            float now_position = Linear_Map_Position(Booster->Motor_Pull.Get_Now_Angle(), Angle_Backward, Angle_Forward,1.0f);//注意这里颠倒了
            Booster->Motor_Pull.Set_Transform_Angle(now_position);

            if(Push_Calibration_Finished && Pull_Calibration_Finished){
                Booster->Set_Booster_Control_Type(Booster_Control_Type_NORMAL);
            }
        }
    }
}

void Class_FSM_Shooting::Reload_TIM_Status_PeriodElapsedCallback()
{
    Status[Now_Status_Serial].Time++;

    //自己接着编写状态转移函数
    switch (Now_Status_Serial)
    {
        case (Shooting_Control_Type_DISABLE):
        {
            
        }
        break;
        case (Shooting_Control_Type_READY)://
        {
            //-----------------------
            //Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Booster->Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            //Motor_Pull.Set_Target_Radian(target_position_pull);
            Booster->Motor_Push_L.Set_Target_Radian(Booster->target_position_push);
            Booster->Motor_Push_R.Set_Target_Radian(Booster->target_position_push);
            //-------------------------------------------------
        }
        break;
        case (2)://向后堵转
        {
    
        }
        break;
        case (3)://后侧检测
        {

        }
        break;
        case (4)://正常控制流程
        {

        }
        break;
    }
}

//测试参数
float Motor_L_test_P = 270.0f;
float Motor_L_test_I = 25.f;
float Motor_R_test_P = 320.0f;
float Motor_R_test_I = 28.f;

float Motor_Push_Angle_P_test = 4200.0f;
float Motor_Push_Angle_I_test = 0.0f;

float Motor_Pull_Omega_test_P = 2300.0f;
float Motor_Pull_Omega_test_I = 370.0f;

float Motor_Pull_Angle_P_test = 300.0f;
float Motor_Pull_Angle_I_test = 0.0f;

/**
 * @brief 发射机构初始化
 *
 */
void Class_Booster::Init()
{
    FSM_Shooting.Booster = this;
    FSM_Shooting.Init(9, 0);

    FSM_Push_Calibration.Booster = this;
    FSM_Push_Calibration.Init(6, 0);

    FSM_Pull_Calibration.Booster = this;
    FSM_Pull_Calibration.Init(6, 0);

    //舵机
    Servo_Trigger.Init(&htim2, TIM_CHANNEL_1, 270);
    Servo_Trigger.Set_Target_Angle(tirrger_fire_angle);

    //拉力电机
    Motor_Pull.PID_Angle.Init(Motor_Pull_Angle_P_test, Motor_Pull_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 35.0f * PI);
    Motor_Pull.PID_Omega.Init(Motor_Pull_Omega_test_P, Motor_Pull_Omega_test_I, 0.001f, 0.0f, 2000, Motor_Pull.Get_Output_Max());
    Motor_Pull.Init(&hfdcan1, DJI_Motor_ID_0x201, DJI_Motor_Control_Method_OMEGA);

    //Push电机左
    Motor_Push_L.PID_Angle.Init(Motor_Push_Angle_P_test, Motor_Push_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Push_L.PID_Omega.Init(Motor_L_test_P, Motor_L_test_I, 0.0f, 0.0f, Motor_Push_L.Get_Output_Max() * 0.5f, Motor_Push_L.Get_Output_Max());
    Motor_Push_L.Init(&hfdcan1, DJI_Motor_ID_0x202, DJI_Motor_Control_Method_OMEGA,1.0f);

    //Push电机右
    Motor_Push_R.PID_Angle.Init(Motor_Push_Angle_P_test, Motor_Push_Angle_I_test, 0.0f, 0.0f, 5.0f * PI, 150.0f * PI);
    Motor_Push_R.PID_Omega.Init(Motor_R_test_P, Motor_R_test_I, 0.0f, 0.0f, Motor_Push_R.Get_Output_Max() * 0.5f, Motor_Push_R.Get_Output_Max());
    Motor_Push_R.Init(&hfdcan1, DJI_Motor_ID_0x203, DJI_Motor_Control_Method_OMEGA,1.0f);
}

/**
 * @brief 输出到电机
 *
 */
// float test_b = -10.0f;//速度环测试目标速度
// float target_position_b = 0.5f;//角度环测试目标位置


void Class_Booster::Output()
{
//  // 设置电机控制模式（调试用）
    // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
    // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);
    // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_OMEGA);

// //角度环调参
//    Motor_Pull.PID_Angle.Set_K_P(Motor_Pull_Angle_P_test);
//    Motor_Pull.PID_Angle.Set_K_I(Motor_Pull_Angle_I_test);
//    Motor_Push_L.PID_Angle.Set_K_P(Motor_Push_Angle_P_test);
//    Motor_Push_R.PID_Angle.Set_K_P(Motor_Push_Angle_P_test);
//    Motor_Push_L.PID_Angle.Set_K_I(Motor_Push_Angle_I_test);
//    Motor_Push_L.PID_Angle.Set_K_I(Motor_Push_Angle_I_test);
    
// //角度环测试
//    Motor_Pull.Set_Target_Radian(Target);
//    Motor_Push_L.Set_Target_Radian(target_position_b);
//    Motor_Push_R.Set_Target_Radian(target_position_b);

// //速度环调参
//    Motor_Pull.PID_Omega.Set_K_P(Motor_Pull_Omega_test_P);
//    Motor_Pull.PID_Omega.Set_K_I(Motor_Pull_Omega_test_I);
//    Motor_Push_L.PID_Omega.Set_K_P(Motor_L_test_P);
//    Motor_Push_R.PID_Omega.Set_K_P(Motor_R_test_P);
//    Motor_Push_L.PID_Omega.Set_K_I(Motor_L_test_I);
//    Motor_Push_R.PID_Omega.Set_K_I(Motor_R_test_I);

// //速度环测试    
//    Motor_Pull.Set_Target_Omega_Radian(test_b);
//    Motor_Pull.Set_Transform_Angle(Motor_Pull.Get_Now_Radian());
//     Motor_Push_L.Set_Target_Omega_Radian(test_b);
//     Motor_Push_L.Set_Transform_Angle(Motor_Push_L.Get_Now_Radian());
//     Motor_Push_R.Set_Target_Omega_Radian(test_b);
//     Motor_Push_R.Set_Transform_Angle(Motor_Push_R.Get_Now_Radian());

    switch (Booster_Control_Type)
    {
        case (Booster_Control_Type_DISABLE):
        {
            // Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
            // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);
            // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_TORQUE);

            // Motor_Pull.Set_Target_Torque(0.f);
            // Motor_Push_L.Set_Target_Torque(0.f);
            // Motor_Push_R.Set_Target_Torque(0.f);

            // Motor_Pull.Set_Out(0.f);
            // Motor_Push_L.Set_Out(0.f);
            // Motor_Push_R.Set_Out(0.f);
        }
        break;
        case (Booster_Control_Type_NORMAL)://校准结束，进入准备发射状态
        {
            //-----------------------
            //Motor_Pull.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
            Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

            //Motor_Pull.Set_Target_Radian(target_position_pull);
            Motor_Push_L.Set_Target_Radian(target_position_push);
            Motor_Push_R.Set_Target_Radian(target_position_push);
            //-------------------------------------------------

            //进入Shooting状态机
            Set_Shooting_Control_Type(Shooting_Control_Type_READY);
        }
        break;

        // case (Booster_Control_Type_READY_Tension)://准备发射状态下一步，PUSH不动，Pull电机进入拉力环
        // {
        //     // //-----------------------
        //     // Pull_Tension_Control();

        //     // Motor_Push_L.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);
        //     // Motor_Push_R.Set_DJI_Motor_Control_Method(DJI_Motor_Control_Method_ANGLE);

        //     // Motor_Push_L.Set_Target_Radian(target_position_push);
        //     // Motor_Push_R.Set_Target_Radian(target_position_push);
        //     // //-------------------------------------------------
        // }
        // break;

    }
    if(servo_test_flag == 1){
        Servo_Trigger.Set_Target_Angle(tirrger_fire_angle);
    }
    Set_Target_position_push(test_0_1_push);


}

/**
 * @brief 定时器计算函数
 *
 */
void Class_Booster::TIM_Calculate_PeriodElapsedCallback()
{    
    //皮筋校准
    FSM_Push_Calibration.Reload_TIM_Status_PeriodElapsedCallback();

    //拉力校准
    //FSM_Pull_Calibration.Reload_TIM_Status_PeriodElapsedCallback();

    //发射状态机

    FSM_Shooting.Reload_TIM_Status_PeriodElapsedCallback();

    Output();
    
    //PID输出
    //Motor_Pull.TIM_PID_PeriodElapsedCallback();
    Motor_Push_L.TIM_PID_PeriodElapsedCallback();
    Motor_Push_R.TIM_PID_PeriodElapsedCallback();

    // //Servo_Trigger.Set_Target_Angle(servo_test);//测试舵机用

}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

#pragma once

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include "2_Device/Motor/Motor_DM/drv_motor_dm.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 发射模块的状态
 * 
 */
enum Shoot_Status
{
    Shoot_Disenable = 0,          // 拨盘电机未使能
    Shoot_Enable,                 // 拨盘电机使能
    Shoot_Fireing,                // 拨盘电机正在发射
    Shoot_StuckReleasing,         // 拨盘电机卡住后释放
};

/**
 * @brief 拨盘电机触发的事件
 *
 */
enum Shoot_Event
{
    Shoot_Event_None = 0,               // 无事件
    Shoot_Event_Enable,                 // 使能事件
    Shoot_Event_Disenable,              // 失能事件
    Shoot_Event_Fire,                   // 开火发射事件
    Shoot_Event_FireDone,               // 开火发射完成事件
    Shoot_Event_Stuck,                  // 卡住事件
    Shoot_Event_StuckRealseDone,        // 卡住后释放完成事件
};

class Class_Shoot
{
    public:
    ::Shoot_Status Shoot_Status;
    
    ::Shoot_Event  Shoot_Event;

    void Ammo_Init (bool *__Fire_Signal,bool *__Enable_Signal);     //初始化拨盘电机，绑定拨盘电机和开火信号源，记录初始化时间戳
    
    void Shoot_FSM_Run();                                           //状态机运行
    
    void Shoot_Event_update();                                      //事件触发更新

    bool Shoot_Stuck_Check();                                       //堵转检测函数
    
    inline void Dial_Motor_Disenable();                             //失能拨盘电机

    inline void Dial_Motor_Enable();                                //使能拨盘电机

    void Dial_Motor_ToZero();                                       //拨盘电机正向回零

    void Dial_Motor_Deal();                                         //正常处理拨盘电机

    void Shoot_Run();                                               //发射运行任务
    //内部变量
    float State_time;                                               //单位：毫秒

    float Begin_Time;                                               //单位：毫秒

    float Current_Time;                                             //单位：毫秒
    
    private:

    bool* Fire_Signal;                                              //开火信号源

    bool* Enable_Signal;                                            //使能信号源

    bool  ToZero_Signal = true;                                     //回零标志位 

    float ToZero_Sample_Angle = NAN;                                 //回零时记录的一次角度, NAN表示未记录

    float Current_Target_Angle;                                     //当前目标角度值

    float Last_Target_Angle;                                        //上一次的目标角度值，用来堵弹时回到上一次目标值

};
/* Exported variables --------------------------------------------------------*/
extern Class_Motor_DM_Normal Motor_DM_4340P;

/* Exported function declarations --------------------------------------------*/

extern "C" void Shoot_Task_Fuc(void *argument);

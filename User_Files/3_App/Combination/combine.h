#pragma once
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
enum Combine_Side
{
    Left_Side = 0,
    Right_Side,
};

enum Combine_FSM_State
{
    Combine_State_DisSetZero =0,                        //未归零点
    Combine_State_SetingZero,                           //归零点中
    Combine_State_Freeing,                              //自由中状态
    Combine_State_Free,                                 //自由状态
    Combine_State_Combining,                            //组合中状态
    Combine_State_Combined,                             //组合完成状态    
    Combine_State_ControlMode,                          //人为控制状态
};

enum Combine_FSM_Event
{
    Combine_Event_None = 0,                             //无事件
    Combine_Event_SetZero,                              //归零点事件
    Combine_Event_Stock,                                //堵转事件
    Combine_Event_To_Free,                              //回到释放状态事件
    Combine_Event_FreeDone,                             //释放完成事件
    Combine_Event_To_Combined,                          //进行组合中事件
    Combine_Event_CombinedDone,                         //组合完成事件
    Combine_Event_Swtich_Control,                       //转换人为控制事件
};

class Combine_FSM
{
public:
    Combine_FSM_State State;
    
    Combine_FSM_Event Event;

    void Combine_FSM_Init();                                //将状态和事件进行初始化

    void Combine_FSM_Run();                                 //状态机运行函数

    void Combine_FSM_Event_Update();                        //事件更新函数

    bool Combine_FSM_Stock_Check(Combine_Side __side);      //堵转检测函数

    bool Combine_FSM_Position_Check(Combine_Side __side);   //到位检测函数

    inline void Combine_FSM_SetZero();                      //归零点事件触发函数

    inline void Combine_FSM_Free();                         //释放事件触发函数

    inline void Combine_FSM_To_Combined();                  //组合事件触发函数

    inline void Combine_FSM_Swtich_Control();               //切换为人为控制事件触发函数

    float ZeroPosition[2];                                  //零点位置

    float CurrentPosition[2];                               //当前位置

    bool Stock_Flag[2];                                     //堵转标志位

    uint32_t Stock_Time[2];                                 //堵转时间戳
private:
    float State_time;                                       //单位：毫秒

    float Begin_Time;                                       //单位：毫秒

    float Current_Time;                                     //当前时间

    float Scale=2.0f;                                       //丝杆刻度    
};
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/



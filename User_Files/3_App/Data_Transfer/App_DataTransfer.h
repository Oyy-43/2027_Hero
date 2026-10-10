#pragma once

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
enum Enum_Remote_Key_Status
{
    Remote_Key_Status_FREE = 0,
    Remote_Key_Status_TRIG_FREE_PRESSED,
    Remote_Key_Status_TRIG_PRESSED_FREE,
    Remote_Key_Status_PRESSED,    
};

struct Struct_Remote_Press_State
{
    Enum_Remote_Key_Status Key_Status;
    bool Pre_Press_State;
    bool Now_Press_State;
};

class Shoot_Decision 
{
    public:

    inline bool* Fire_Signal_Init();           //给任务返回开火信号源

    inline bool* Enable_Signal_Init();         //给任务返回使能信号源

    void Fire_Signal_Decide();          //开火信号决策
    
    void Enable_Signal_Decide();        //使能信号决策

    private:

    bool Fire_Signal;                   //开火信号，仅在这个类中决策，被更改

    bool Enable_Signal;                 //使能信号
};

class Remote_Data
{
    public:

    void Remote_Key_Scan();                      //遥控器按键扫描

    void Remote_Key_Init();                      //遥控器按键初始化

    Struct_Remote_Press_State Remote_Key_Right;        //遥控器右侧按键状态

    inline Enum_Remote_Key_Status Get_Remote_Key_Status() const;
};
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern Shoot_Decision Shoot_Decision_Transfer;
extern Remote_Data Remote_Data_Transfer;
/* Exported function declarations --------------------------------------------*/

inline bool* Shoot_Decision::Fire_Signal_Init()
{
    return &Fire_Signal;
}

inline bool* Shoot_Decision::Enable_Signal_Init()
{
    return &Enable_Signal;
}

inline Enum_Remote_Key_Status Remote_Data::Get_Remote_Key_Status() const
{
    return Remote_Key_Right.Key_Status;
}

extern "C" void Robo_Dec_Fuc(void *argument);



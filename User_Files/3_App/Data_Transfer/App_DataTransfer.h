#pragma once

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
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

inline bool* Shoot_Decision::Fire_Signal_Init()
{
    return &Fire_Signal;
}

inline bool* Shoot_Decision::Enable_Signal_Init()
{
    return &Enable_Signal;
}

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern Shoot_Decision Shoot_Decision_Transfer;
/* Exported function declarations --------------------------------------------*/

extern "C" void Robo_Dec_Fuc(void *argument);



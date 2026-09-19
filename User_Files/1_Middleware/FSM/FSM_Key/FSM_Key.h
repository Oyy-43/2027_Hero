#pragma once

/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum 
{
    KEY_STATE_IDLE = 0,             // 空闲状态
    KEY_DOWN_DEBOUNCE,              // 按下消抖状态

    KEY_STATE_DOWN,                 // 按下状态
    KEY_UP_DEBOUNCE,                // 抬起消抖状态

    KEY_STATE_LONGPRESSED,          // 长按状态
    KEY_LOONGPRESS_DEBOUNCE,        // 长按抬起消抖状态

    KEY_STATE_UP,                   // 抬起状态
    KEY_DOUBLE_DOWN_DEBOUNCE,       // 双击按下消抖状态
    KEY_DOUBLE_DOWN,                // 双击按下状态
    KEY_DOUBLE_UP_DEBOUNCE          // 双击抬起消抖状态

} key_state_t;// 按键状态枚举

typedef enum 
{
    KEY_EVENT_NONE = 0,             // 无事件
    KEY_EVENT_PRESS,                // 按键按下事件
    KEY_EVENT_RELEASE,              // 按键释放事件
    KEY_EVENT_TIMEOUT,              // 按键超时事件
    KEY_EVENT_LONGPRESSED,          // 长按事件
    KEY_EVENT_DOUBLECLICK,          // 双击事件

    KEY_EVENT_IDLE

} key_event_t;// 按键事件枚举

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/


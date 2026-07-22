#ifndef __ALG_PID_H_
#define __ALG_PID_H_
/* Includes ------------------------------------------------------------------*/
#include "main.h"


/* Exported macros -----------------------------------------------------------*/
#define ABS(x) ((x > 0) ? x : -x)

/* Exported types ------------------------------------------------------------*/
//PID优化功能枚举
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        //0000 0000(无)
    Integral_Limit = 0x01,              //0000 0001(积分限幅)
    Derivative_On_Measurement = 0x02,   //0000 0010(微分先行)
    Trapezoid_Intergral = 0x04,         //0000 0100(梯形积分)
    Proportional_On_Measurement = 0x08, //0000 1000(比例先行，暂不涉及)
    OutputFilter = 0x10,                //0001 0000(输出滤波)
    ChangingIntegralRate = 0x20,        //0010 0000(变积分)
    DerivativeFilter = 0x40,            //0100 0000(微分滤波)
    ErrorHandle = 0x80,                 //1000 0000(异常处理)
} pid_Improvement_e;

/**
 * @brief PID错误类型枚举
 * 
 */

typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    Motor_Blocked = 0x01U
} ErrorType_e;

/** 
 * @brief PID错误处理结构体
 * 
 */

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

typedef struct _PID_TypeDef
{
    float Target;
    float LastNoneZeroTarget;
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float KffStaticPos;
    float KffStaticNeg;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;
    float Fout;

    float Output;
    float Last_Output;
    float Last_Dout;

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ControlPeriod;
    float MaxErr;
    float ScalarA; //变积分参数
    float ScalarB; //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_Filtering_Coefficient;
    float Derivative_Filtering_Coefficient;

    uint32_t thistime;
    uint32_t lasttime;
    uint8_t dtime;

    uint8_t Improve;//用于使能功能所使用的参数

    PID_ErrorHandler_t ERRORHandler;

    void (*PID_param_init)(
        struct _PID_TypeDef *pid,
        float maxOut,
        float integralLimit,
        float deadband,
        float Kp,
        float ki,
        float kd,
        float kf,
        float kff_static_pos,
        float kff_static_neg,
        float A,
        float B,
        float output_filtering_coefficient,
        float derivative_filtering_coefficient,
        uint8_t improve);

    void (*PID_reset)(
        struct _PID_TypeDef *pid,
        float Kp,
        float Ki,
        float Kd,
        float Kf);
} PID_TypeDef;

/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/

void PID_Init(
    PID_TypeDef *pid,
    float max_out,
    float intergral_limit,
    float deadband,

    float kp,
    float ki,
    float kd,
    float kf,
    float kff_static_pos,
    float kff_static_neg,

    float A,
    float B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,

    uint8_t improve);
float PID_Calculate(PID_TypeDef *pid, float measure, float target, float Delta_T);
		





#endif /* __ALG_PID_H */
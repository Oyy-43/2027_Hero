/**
 * @file alg_pid.c
 * @author Oyyp
 * @brief 移植于玺佬的pid，挺好用的
 * @version 0.1
 * @date 2026-03-30 0.1 初版
 *
 * @copyright Copyright
 *
 */

 /* Includes ------------------------------------------------------------------*/
#include "alg_pid.h"
#include "alg_basic.h"
 
 /* Private macros ------------------------------------------------------------*/
 
 /* Private types -------------------------------------------------------------*/
 
 /* Private variables ---------------------------------------------------------*/
 
 /* Private function declarations ---------------------------------------------*/
static void f_Trapezoid_Intergral(PID_TypeDef *pid);
static void f_Integral_Limit(PID_TypeDef *pid);
static void f_Derivative_On_Measurement(PID_TypeDef *pid);
static void f_Changing_Integral_Rate(PID_TypeDef *pid);
static void f_Output_Filter(PID_TypeDef *pid);
static void f_Derivative_Filter(PID_TypeDef *pid);
static void f_Output_Limit(PID_TypeDef *pid);
static void f_Proportion_Limit(PID_TypeDef *pid);
static void f_PID_ErrorHandle(PID_TypeDef *pid);

 /***************************PID param initialize******************************/
static void f_PID_param_init(
    PID_TypeDef *pid,
    float max_out,
    float intergral_limit,
    float deadband,
    float d_t,

    float kp,
    float Ki,
    float Kd,
    float kf,

    float Changing_Integral_A,
    float Changing_Integral_B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,

    uint8_t improve)
{
    //参数初始化
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->MaxErr = max_out * 2;
    pid->D_T = d_t;
    pid->Target = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = kf;
    pid->ITerm = 0;

    pid->ScalarA = Changing_Integral_A;
    pid->ScalarB = Changing_Integral_B;

    pid->Output_Filtering_Coefficient = output_filtering_coefficient;

    pid->Derivative_Filtering_Coefficient = derivative_filtering_coefficient;

    pid->Improve = improve;


    //异常处理初始化
    pid->ERRORHandler.ERRORCount = 0;
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE;

    pid->Output = 0;
}

/**************************PID param reset*********************************/
static void f_PID_reset(PID_TypeDef *pid, float Kp, float Ki, float Kd,float Kf)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Kf = Kf;
    if (pid->Ki == 0)
        pid->Iout = 0;
}

/***************************PID calculate**********************************/
float PID_Calculate(PID_TypeDef *pid, float measure, float target)
{
    if (pid->Improve & ErrorHandle) 
    {
        f_PID_ErrorHandle(pid); // 处理异常
        if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            pid->Output = 0;
            return 0; //Catch ERROR
        }
    }

    pid->Measure = measure;
    pid->Target = target;
    pid->Err = pid->Target - pid->Measure;
    pid->abs_Err = ABS(pid->Err);


    if (pid->abs_Err > pid->DeadBand)
    {
        if(pid->Err >0)
        {
            pid->Err -= pid->DeadBand;
        }
        else if(pid->Err < 0)
        {
            pid->Err += pid->DeadBand;
        }
        pid->Pout = pid->Kp * pid->Err;
        pid->ITerm = pid->Ki * pid->Err * pid->D_T;
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err)/pid->D_T;

        //梯形积分使能判断
        if (pid->Improve & Trapezoid_Intergral)
            f_Trapezoid_Intergral(pid);
        //变积分使能判断
        if (pid->Improve & ChangingIntegralRate)
            f_Changing_Integral_Rate(pid);
        //积分限幅使能判断
        if (pid->Improve & Integral_Limit)
            f_Integral_Limit(pid);
        //微分先行使能判断
        if (pid->Improve & Derivative_On_Measurement)
            f_Derivative_On_Measurement(pid);
        //微分滤波使能判断
        if (pid->Improve & DerivativeFilter)
            f_Derivative_Filter(pid);

        pid->Iout += pid->ITerm;
        pid->Fout =pid->Kf * (pid->Target - pid->LastNoneZeroTarget);
        
        pid->Output = pid->Pout + pid->Iout + pid->Dout + pid->Fout ;

        //输出滤波使能判断
        if (pid->Improve & OutputFilter)
        f_Output_Filter(pid);

        //输出限幅
        f_Output_Limit(pid);

        //比例限幅
        f_Proportion_Limit(pid);
    }
    else
    {
        pid->Target = pid->Measure;
        pid->Err = 0;
        pid->abs_Err = 0;
        pid->Pout = 0.0f;
        pid->ITerm = 0.0f;
        pid->Dout = 0.0f;
        pid->Fout = 0.0f;
        pid->Output = 0.0f;
    }
    pid->Last_Measure = pid->Measure;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = pid->Dout;
    pid->Last_Err = pid->Err;
    pid->LastNoneZeroTarget =pid->Target;

    return pid->Output;
}

/*****************PID Improvement Function*********************/
static void f_Trapezoid_Intergral(PID_TypeDef *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2)*pid->D_T;
}

static void f_Changing_Integral_Rate(PID_TypeDef *pid)
{
    if (pid->Err * pid->Iout > 0)
    {
        //积分仍在增加
        if (ABS(pid->Err) <= pid->ScalarB)
            return; //完整积分
        if (ABS(pid->Err) <= (pid->ScalarA + pid->ScalarB))
            pid->ITerm *= (pid->ScalarA - ABS(pid->Err) + pid->ScalarB) / pid->ScalarA;
        else
            pid->ITerm = 0;//取消积分
    }
}

static void f_Integral_Limit(PID_TypeDef *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;
    temp_Output = pid->Pout + pid->Iout + pid->Dout;
    if (ABS(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)
        {
            //积分持续增加
            pid->ITerm = 0;
        }
    }

    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = pid->IntegralLimit;
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

static void f_Derivative_On_Measurement(PID_TypeDef *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure)/pid->D_T;
}

static void f_Derivative_Filter(PID_TypeDef *pid)
{
    pid->Dout = pid->Dout * pid->Derivative_Filtering_Coefficient +
                pid->Last_Dout * (1 - pid->Derivative_Filtering_Coefficient);
}

static void f_Output_Filter(PID_TypeDef *pid)
{
    pid->Output = pid->Output * pid->Output_Filtering_Coefficient +
                  pid->Last_Output * (1 - pid->Output_Filtering_Coefficient);
}

static void f_Output_Limit(PID_TypeDef *pid)
{
    if (pid->Output > pid->MaxOut)
    {
        pid->Output = pid->MaxOut;
    }
    if (pid->Output < -(pid->MaxOut))
    {
        pid->Output = -(pid->MaxOut);
    }
}

static void f_Proportion_Limit(PID_TypeDef *pid)
{
    //比例项限幅在控制的过程中没有很重要的作用，
    //但它会让变量图看起来更好看
    if (pid->Pout > pid->MaxOut)
    {
        pid->Pout = pid->MaxOut;
    }
    if (pid->Pout < -(pid->MaxOut))
    {
        pid->Pout = -(pid->MaxOut);
    }
}

/*****************PID ERRORHandle Function*********************/
static void f_PID_ErrorHandle(PID_TypeDef *pid)
{
    /*电机堵转处理*/
    if (pid->Output < pid->MaxOut * 0.01f)
        return;

    if ((ABS(pid->Target - pid->Measure) / pid->Target) > 0.9f)
    {
        //电机堵转计数
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;
    }

    if (pid->ERRORHandler.ERRORCount > 1000)
    {
        //电机堵转超过1000次
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/*****************PID structure initialize*********************/
void PID_Init(
    PID_TypeDef *pid,
    float max_out,
    float intergral_limit,
    float deadband,
    float d_t,

    float kp,
    float Ki,
    float Kd,
    float Kf,

    float A,
    float B,

    float output_filtering_coefficient,
    float derivative_filtering_coefficient,
    uint8_t improve)
{
    pid->PID_param_init = f_PID_param_init;
    pid->PID_reset = f_PID_reset;//连接kp，ki，kd参数重设函数
    pid->PID_param_init(pid, max_out, intergral_limit, deadband, d_t,
                        kp, Ki, Kd, Kf, A, B, output_filtering_coefficient, derivative_filtering_coefficient, improve);//连接并调用参数初始化函数
}
 
 /* Function prototypes -------------------------------------------------------*/
 
 

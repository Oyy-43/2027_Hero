#pragma once
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "1_Middleware/Driver/UART/drv_uart.h"

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef enum
{
    Data_Rid_Accel = 0,
    Data_Rid_Gyro,
    Data_Rid_Eul,
    Data_Rid_Quaternion,
}Data_Rid_t;

typedef struct __attribute__((packed))
{
    uint8_t Header;
    uint8_t Data_Type;
    uint8_t Slave_id;
	uint8_t Data_Rid;
    uint8_t Op;
	float Data[4];
	uint8_t Response_Code;
    uint8_t Reserved;
	uint8_t tail;
}DM_BMI088_Normal_Packet_t;

extern DM_BMI088_Normal_Packet_t Request_Packet_Accel_t;
extern DM_BMI088_Normal_Packet_t Request_Packet_Gyro_t;
extern DM_BMI088_Normal_Packet_t Request_Packet_Eul_t;


typedef struct
{
    float accel[3];
    float gyro[3];
    float eul[3];
}DM_BMI088_Data_t;

class Class_IMU_DM_BMI088
{
    public:
    void BMI088_Init(Struct_UART_Manage_Object* __UART,uint8_t Slave_id);        //绑定请求串口，从机地址。
    
    void Data_Unpacked(uint8_t *buf, uint16_t length);                           //数据解包

    void BMI_088_TIM1msCall_Send_Request();                                      //发送请求

    inline void Request_Accel();

    inline void Request_Gyro();

    inline void Request_Eul();

    inline void Request_Quaternion();

    //获取加速度计数据
    float Get_Accel_X();                                                    
    float Get_Accel_Y();
    float Get_Accel_Z();

    //获取角速度数据
    float Get_Gyro_X();
    float Get_Gyro_Y();
    float Get_Gyro_Z();

    //获取欧拉角数据
    float Get_Eul_Roll();
    float Get_Eul_Pitch();
    float Get_Eul_Yaw();

    protected:
    DM_BMI088_Data_t BMI088_Data;

    UART_HandleTypeDef *UART_Handler = nullptr;

    uint8_t Slave_id;
    uint8_t Next_Request = Data_Rid_Accel;
};
/* Exported constants --------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern Class_IMU_DM_BMI088 DM_BMI088;
/* Exported function declarations --------------------------------------------*/

void DM_BMI088_Idle_Callback(uint8_t *buf, uint16_t length);


inline void Class_IMU_DM_BMI088::Request_Accel()
{
    Request_Packet_Accel_t.Slave_id = Slave_id;
    UART_Transmit_Data(UART_Handler,reinterpret_cast<uint8_t *>(&Request_Packet_Accel_t),sizeof(Request_Packet_Accel_t));
}

inline void Class_IMU_DM_BMI088::Request_Gyro()
{
    Request_Packet_Gyro_t.Slave_id = Slave_id;
    UART_Transmit_Data(UART_Handler,reinterpret_cast<uint8_t *>(&Request_Packet_Gyro_t),sizeof(Request_Packet_Gyro_t));
}

inline void Class_IMU_DM_BMI088::Request_Eul()
{
    Request_Packet_Eul_t.Slave_id = Slave_id;
    UART_Transmit_Data(UART_Handler,reinterpret_cast<uint8_t *>(&Request_Packet_Eul_t),sizeof(Request_Packet_Eul_t));
}

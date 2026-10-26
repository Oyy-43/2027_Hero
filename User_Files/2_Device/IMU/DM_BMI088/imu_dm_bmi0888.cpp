/**
 * @file imu_dm_bmi0888.cpp
 * @author oyyp
 * @brief 达妙外置BMI088 RS485通讯代码
 * @version 0.1
 * @date 2026-10-23 0.1 init
 *
 * @copyright Copyright
 *
 */
/* Includes ------------------------------------------------------------------*/
#include "imu_dm_bmi088.h"


/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
Class_IMU_DM_BMI088 DM_BMI088;
RAM_D2_BUFFER DM_BMI088_Normal_Packet_t Request_Packet_Accel_t;
RAM_D2_BUFFER DM_BMI088_Normal_Packet_t Request_Packet_Gyro_t;
RAM_D2_BUFFER DM_BMI088_Normal_Packet_t Request_Packet_Eul_t;
/* Private function declarations ---------------------------------------------*/
void Class_IMU_DM_BMI088::BMI088_Init(Struct_UART_Manage_Object* __UART,uint8_t Slave_id)
{
    UART_Handler = __UART->UART_Handler;
    this->Slave_id = Slave_id;
    Next_Request = Data_Rid_Accel;
    Request_Packet_Accel_t = {0xA5,0x0D,Slave_id,Data_Rid_Accel,0x00,{0.0f,0.0f,0.0f,0.0f},0x00,0x00,0x5A};
    Request_Packet_Gyro_t = {0xA5,0x0D,Slave_id,Data_Rid_Gyro,0x00,{0.0f,0.0f,0.0f,0.0f},0x00,0x00,0x5A};
    Request_Packet_Eul_t = {0xA5,0x0D,Slave_id,Data_Rid_Eul,0x00,{0.0f,0.0f,0.0f,0.0f},0x00,0x00,0x5A};
}

void Class_IMU_DM_BMI088::BMI_088_TIM1msCall_Send_Request()
{
    if (UART_Handler == nullptr || UART_Handler->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    DM_BMI088_Normal_Packet_t *request = &Request_Packet_Accel_t;
    if (Next_Request == Data_Rid_Gyro)
    {
        request = &Request_Packet_Gyro_t;
    }
    else if (Next_Request == Data_Rid_Eul)
    {
        request = &Request_Packet_Eul_t;
    }

    if (UART_Transmit_Data(UART_Handler, reinterpret_cast<uint8_t *>(request), sizeof(*request)) == HAL_OK)
    {
        Next_Request = (Next_Request + 1U) % 3U;
    }
}

void Class_IMU_DM_BMI088::Data_Unpacked(uint8_t *buf, uint16_t length)
{
    if(buf == nullptr || length != sizeof(DM_BMI088_Normal_Packet_t))
    {
        return;
    }
    DM_BMI088_Normal_Packet_t *temp_Packet = reinterpret_cast<DM_BMI088_Normal_Packet_t *>(buf);
    if(temp_Packet->Header != 0xA5 || temp_Packet->tail != 0x5A ||
       temp_Packet->Data_Type != 0x0D || temp_Packet->Slave_id != this->Slave_id ||
       temp_Packet->Op != 0 || temp_Packet->Response_Code != 0)
    {
        return;
    }

    switch(temp_Packet->Data_Rid)
    {
        case Data_Rid_Accel:
            BMI088_Data.accel[0] = temp_Packet->Data[0];
            BMI088_Data.accel[1] = temp_Packet->Data[1];
            BMI088_Data.accel[2] = temp_Packet->Data[2];
            break;
        case Data_Rid_Gyro:
            BMI088_Data.gyro[0] = temp_Packet->Data[0];
            BMI088_Data.gyro[1] = temp_Packet->Data[1];
            BMI088_Data.gyro[2] = temp_Packet->Data[2];
            break;
        case Data_Rid_Eul:
            BMI088_Data.eul[0] = temp_Packet->Data[0];
            BMI088_Data.eul[1] = temp_Packet->Data[1];
            BMI088_Data.eul[2] = temp_Packet->Data[2];
            break;
        default:
            break;
    }
}

void DM_BMI088_Idle_Callback(uint8_t *buf, uint16_t length)
{
    static uint8_t packet[sizeof(DM_BMI088_Normal_Packet_t)];
    static uint16_t received = 0;
    if (buf == nullptr)
    {
        return;
    }

    for (uint16_t offset = 0; offset < length; ++offset)
    {
        if (received == 0 && buf[offset] != 0xA5)
        {
            continue;
        }
        packet[received++] = buf[offset];
        if (received == sizeof(packet))
        {
            if (packet[1] == 0x0D && packet[sizeof(packet) - 1] == 0x5A)
            {
                DM_BMI088.Data_Unpacked(packet, received);
                received = 0;
            }
            else
            {
                uint16_t next_header = 1;
                while (next_header < received && packet[next_header] != 0xA5)
                {
                    ++next_header;
                }
                received -= next_header;
                for (uint16_t index = 0; index < received; ++index)
                {
                    packet[index] = packet[next_header + index];
                }
            }
        }
    }
}


/* Function prototypes -------------------------------------------------------*/


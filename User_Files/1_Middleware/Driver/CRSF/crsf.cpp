/**
 * @file crsf.cpp
 * @author Zomfly
 * @brief 伟大的中南爷写的CRSF协议
 * @version 0.1
 * @date 2026-04-05 0.1 init
 *
 * @copyright Copyright
 *
 */

 /* Includes ------------------------------------------------------------------*/
 #include "crsf.h"
 

 /* Private macros ------------------------------------------------------------*/
 
 /* Private types -------------------------------------------------------------*/
RAM_D2_BUFFER uint8_t crsf_send_buffer[64] = {};

crsf_boardcast_frame_t crsf_frame = {0};

crsf_channels_t rc_channels;

int16_t Failsafe_count = 0;
 /* Private variables ---------------------------------------------------------*/
 
 /* Private function declarations ---------------------------------------------*/
 __STATIC_INLINE uint8_t crsf_calculate_crc(const uint8_t * ptr, const uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i=0; i<len; i++)
        crc = crc8tab[crc ^ *ptr++];
    return crc;
}

__STATIC_INLINE uint8_t crsf_rx_msg_check(const uint8_t * msg, const uint8_t len)
{
    if (len < 4 || len > 64)
        return 0;
    if (msg[1] != len - 2)
        return 0;
    if (crsf_calculate_crc(msg + 2, len - 3) != msg[len - 1])
        return 0;
    return 1;
}

void crsf_rx_idle_callback(uint8_t *buf, uint16_t length)
{
    if (!crsf_rx_msg_check(buf, length))
    {
        return;
    }

    crsf_frame.addr = buf[0];
    crsf_frame.length = buf[1];
    crsf_frame.type = buf[2];
    memcpy(crsf_frame.payload, &buf[3], crsf_frame.length - 2);
    crsf_frame.crc = buf[length - 1];

    switch (crsf_frame.addr)
    {
    case CRSF_ADDRESS_FLIGHT_CONTROLLER:
        switch (crsf_frame.type)
        {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            // 16ch 遥控
            crsf_unpack_flight_controller(&crsf_frame);
            break;
        
        default:
            break;
        }
        break;
    
    default:
        break;
    }
}

void __send_crsf_packet(const CRSF_FRAMETYPE_t frame_type, const uint8_t * payload, const uint8_t size_of_payload)
{
    // CRSF帧首字节是目的地址，遥测数据发给接收机由其转发到遥控器
    crsf_send_buffer[0] = CRSF_ADDRESS_CRSF_RECEIVER;
    crsf_send_buffer[1] = size_of_payload + 2;
    crsf_send_buffer[2] = frame_type;
    memcpy(&crsf_send_buffer[3], payload, size_of_payload);
    crsf_send_buffer[size_of_payload + (4 - 1)] = crsf_calculate_crc(&crsf_send_buffer[2], size_of_payload + 1);
    UART_Transmit_Data(&CRSF_UART, crsf_send_buffer, size_of_payload + 4);
}

float get_vbat_voltage(void)
{
    uint16_t adcx = 0;
    // 乘数算子
    // 3.3V / 65535
    //(10K Ω + 100K Ω)  / 10K Ω = 11
    const float multiplier = ((3.3f/65535) * 11.0f);
    float voltage;

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
        return 0.0f;
    }

    if (HAL_ADC_PollForConversion(&hadc1, 5) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0.0f;
    }

    adcx = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    
    voltage =  (float)adcx * multiplier;

    return voltage;
}

void Tele_TIM_PeriodElapsedCallback()
{
    float voltage = get_vbat_voltage();
    crsf_send_BatterySensor(voltage, 1145, ((voltage + .00005f) * 100000.f), 99);
}

// 发送函数指针
void (*send_crsf_packet)(const CRSF_FRAMETYPE_t, const uint8_t *, const uint8_t) = __send_crsf_packet;



/* Function prototypes -------------------------------------------------------*/
 
 

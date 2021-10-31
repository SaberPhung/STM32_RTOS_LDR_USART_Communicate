#include "ldr.h"
#include "main.h"
#include "adc.h"
unsigned char MSG[20];
void read_ldr_sensor(void)
{
    uint16_t raw_value;
    double temp,lux;
    HAL_ADC_PollForConversion(&hadc6, HAL_MAX_DELAY);

    raw_value = HAL_ADC_GetValue(&hadc6);
    temp = ((double)raw_value);
    lux = 3.0 * 1000000.0 * pow(temp, -1.367);
    int l_decimal=(int)(round(lux));
    sprintf(MSG, "Lux: %d\r\n", l_decimal);
    HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
    HAL_UART_Transmit(&huart3, MSG, sizeof(MSG), 100);
}
//this ldr_print use to transmit data to master using usart3
void ldr_print(UART_HandleTypeDef *uart)
{
    OS_ERR os_err;
    
    HAL_UART_Receive(uart, MSG, sizeof(MSG), 100);
    HAL_UART_Transmit(&huart2, MSG, sizeof(MSG), 100);
    OSTimeDlyHMSM(0, 0, 0, 1, OS_OPT_TIME_HMSM_STRICT, &os_err);
}
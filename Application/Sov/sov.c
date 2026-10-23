#include "sov.h"

// -------------------------- 气泵/电磁阀硬件配置 --------------------------
#define AIR_PUMP_PORT    GPIOA
#define AIR_PUMP_PIN     GPIO_PIN_4

#define SOLENOID_VALVE_PORT  GPIOA
#define SOLENOID_VALVE_PIN   GPIO_PIN_5

/**
 * @brief  控制PYD5004气泵启停
 * @param  state: 1-开启气泵，0-关闭气泵
 * @retval 无
 */

/**
 * @brief  触发电磁阀单次动作（高电平保持指定时间后复位）
 * @param  delay_ms: 高电平保持时间（毫秒），默认建议1000ms
 * @retval 无
 * @note   HAL_Delay在中断中不可用，若需中断中使用需替换为定时器延时
 */

    

void SolenoidValve_Trigger(uint32_t state)
{
    if (state == 1)
    {
        // 开启气泵：GPIO置高
        HAL_GPIO_WritePin(SOLENOID_VALVE_PORT, SOLENOID_VALVE_PIN, GPIO_PIN_SET);
    }
    else
    {
        // 关闭气泵：GPIO置低
        HAL_GPIO_WritePin(SOLENOID_VALVE_PORT, SOLENOID_VALVE_PIN, GPIO_PIN_RESET);
    }
}

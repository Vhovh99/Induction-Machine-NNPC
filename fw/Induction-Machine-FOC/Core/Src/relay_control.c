#include "relay_control.h"
#include "main.h"
#include "stm32g4xx_hal_gpio.h"


typedef struct {
    GPIO_TypeDef* Port;
    uint16_t Pin;
} Relay_Config_t;

// Map logical relay index to hardware definitions
static const Relay_Config_t Relay_Config[] = {
    {RELAY1_GPIO_Port, RELAY1_Pin},
    {RELAY2_GPIO_Port, RELAY2_Pin},
    {RELAY3_GPIO_Port, RELAY3_Pin},
    {RELAY4_GPIO_Port, RELAY4_Pin},
    {RELAY5_GPIO_Port, RELAY5_Pin},
    {RELAY6_GPIO_Port, RELAY6_Pin},
    {RELAY7_GPIO_Port, RELAY7_Pin},
    {RELAY8_GPIO_Port, RELAY8_Pin},
    {RELAY9_GPIO_Port, RELAY9_Pin},
    {RELAY10_GPIO_Port, RELAY10_Pin},
    {RELAY11_GPIO_Port, RELAY11_Pin},
};

#define RELAY_COUNT (sizeof(Relay_Config) / sizeof(Relay_Config[0]))

/**
 * @brief Sets the number of active resistors connected to the generator.
 * @param u8LoadCount Number of resistors to connect (0 to RELAY_COUNT).
 */
void Relay_SetLoad(uint8_t u8LoadCount) {
    if (u8LoadCount > RELAY_COUNT) {
        u8LoadCount = RELAY_COUNT;
    }

    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        if (i < u8LoadCount) {
            HAL_GPIO_WritePin(Relay_Config[i].Port, Relay_Config[i].Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(Relay_Config[i].Port, Relay_Config[i].Pin, GPIO_PIN_RESET);
        }
    }
}

void remove_load(void) {
    Relay_SetLoad(0);
}

void add_load(void) {
    Relay_SetLoad(RELAY_COUNT);
}

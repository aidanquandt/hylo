/*---------------------------------------------------------------------------
 * @file    platform_spi.c
 * @brief   Generic SPI hardware abstraction layer implementation
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_spi.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SPI_TIMEOUT_MS  (100U)

// TODO: Define your SPI handle - adjust based on your STM32CubeMX configuration
extern SPI_HandleTypeDef hspi1;  // Change to your SPI instance

// TODO: Define CS pin mapping
#define CS_PIN_DWM3000  (0U)  // Example CS identifier

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

platform_spi_status_e platform_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length)
{
    HAL_StatusTypeDef status;
    
    if (tx_data != NULL && rx_data != NULL) {
        status = HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)tx_data, rx_data, length, SPI_TIMEOUT_MS);
    } else if (tx_data != NULL) {
        status = HAL_SPI_Transmit(&hspi1, (uint8_t*)tx_data, length, SPI_TIMEOUT_MS);
    } else if (rx_data != NULL) {
        status = HAL_SPI_Receive(&hspi1, rx_data, length, SPI_TIMEOUT_MS);
    } else {
        return PLATFORM_SPI_ERROR;
    }
    
    if (status == HAL_OK) {
        return PLATFORM_SPI_SUCCESS;
    } else if (status == HAL_TIMEOUT) {
        return PLATFORM_SPI_TIMEOUT;
    } else {
        return PLATFORM_SPI_ERROR;
    }
}

platform_spi_status_e platform_spi_transmit(const uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return PLATFORM_SPI_ERROR;
    }
    
    HAL_StatusTypeDef status = HAL_SPI_Transmit(&hspi1, (uint8_t*)data, length, SPI_TIMEOUT_MS);
    
    if (status == HAL_OK) {
        return PLATFORM_SPI_SUCCESS;
    } else if (status == HAL_TIMEOUT) {
        return PLATFORM_SPI_TIMEOUT;
    } else {
        return PLATFORM_SPI_ERROR;
    }
}

platform_spi_status_e platform_spi_receive(uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return PLATFORM_SPI_ERROR;
    }
    
    HAL_StatusTypeDef status = HAL_SPI_Receive(&hspi1, data, length, SPI_TIMEOUT_MS);
    
    if (status == HAL_OK) {
        return PLATFORM_SPI_SUCCESS;
    } else if (status == HAL_TIMEOUT) {
        return PLATFORM_SPI_TIMEOUT;
    } else {
        return PLATFORM_SPI_ERROR;
    }
}

platform_spi_status_e platform_spi_set_speed(platform_spi_speed_e speed)
{
    // TODO: Adjust prescaler values based on your APB clock frequency
    // Example for 100 MHz APB clock:
    // - Slow: 100 MHz / 64 = ~1.56 MHz
    // - Fast: 100 MHz / 8 = ~12.5 MHz
    
    if (speed == PLATFORM_SPI_SPEED_SLOW) {
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    } else {
        hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    }
    
    if (HAL_SPI_Init(&hspi1) == HAL_OK) {
        return PLATFORM_SPI_SUCCESS;
    } else {
        return PLATFORM_SPI_ERROR;
    }
}

void platform_spi_cs_low(uint8_t cs_pin)
{
    switch (cs_pin) {
        case CS_PIN_DWM3000:
            HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_RESET);
            break;
        default:
            break;
    }
}

void platform_spi_cs_high(uint8_t cs_pin)
{
    switch (cs_pin) {
        case CS_PIN_DWM3000:
            HAL_GPIO_WritePin(SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}

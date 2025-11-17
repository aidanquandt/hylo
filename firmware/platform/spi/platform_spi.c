/*---------------------------------------------------------------------------
 * @file    platform_spi.c
 * @brief   Generic SPI hardware abstraction layer implementation
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_spi.h"
#include "common.h"
#include "main.h"
#include "spi.h"
#include "gpio.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SPI_TIMEOUT_MS  (100U)

// SPI peripheral handles
extern SPI_HandleTypeDef hspi1;  // DW3000 UWB
extern SPI_HandleTypeDef hspi5;  // BMI323 IMU

// CS pin mapping
typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    SPI_HandleTypeDef *hspi;
} spi_cs_map_t;

STATIC const spi_cs_map_t cs_map[] = {
    [PLATFORM_SPI_CS_DW3000] = { SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, &hspi1 },
    [PLATFORM_SPI_CS_BMI323] = { SPI5_CSn_GPIO_Port, SPI5_CSn_Pin, &hspi5 },
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
// Current SPI peripheral - set by cs_low() before each transaction
STATIC SPI_HandleTypeDef *current_spi = NULL;

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

platform_spi_status_E platform_spi_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t length)
{
    HAL_StatusTypeDef status;
    
    if (tx_data != NULL && rx_data != NULL) {
        status = HAL_SPI_TransmitReceive(current_spi, (uint8_t*)tx_data, rx_data, length, SPI_TIMEOUT_MS);
    } else if (tx_data != NULL) {
        status = HAL_SPI_Transmit(current_spi, (uint8_t*)tx_data, length, SPI_TIMEOUT_MS);
    } else if (rx_data != NULL) {
        status = HAL_SPI_Receive(current_spi, rx_data, length, SPI_TIMEOUT_MS);
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

platform_spi_status_E platform_spi_transmit(const uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return PLATFORM_SPI_ERROR;
    }
    
    HAL_StatusTypeDef status = HAL_SPI_Transmit(current_spi, (uint8_t*)data, length, SPI_TIMEOUT_MS);
    
    if (status == HAL_OK) {
        return PLATFORM_SPI_SUCCESS;
    } else if (status == HAL_TIMEOUT) {
        return PLATFORM_SPI_TIMEOUT;
    } else {
        return PLATFORM_SPI_ERROR;
    }
}

platform_spi_status_E platform_spi_receive(uint8_t *data, uint16_t length)
{
    if (data == NULL) {
        return PLATFORM_SPI_ERROR;
    }
    
    HAL_StatusTypeDef status = HAL_SPI_Receive(current_spi, data, length, SPI_TIMEOUT_MS);
    
    if (status == HAL_OK) {
        return PLATFORM_SPI_SUCCESS;
    } else if (status == HAL_TIMEOUT) {
        return PLATFORM_SPI_TIMEOUT;
    } else {
        return PLATFORM_SPI_ERROR;
    }
}

platform_spi_status_E platform_spi_set_speed(platform_spi_speed_E speed)
{
    if (current_spi == NULL) {
        return PLATFORM_SPI_ERROR;
    }
    
    // Adjust prescaler values based on your APB clock frequency
    // Example for 100 MHz APB clock:
    // - Slow: 100 MHz / 64 = ~1.56 MHz
    // - Fast: 100 MHz / 8 = ~12.5 MHz
    
    if (speed == PLATFORM_SPI_SPEED_SLOW) {
        current_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    } else {
        current_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    }
    
    if (HAL_SPI_Init(current_spi) == HAL_OK) {
        return PLATFORM_SPI_SUCCESS;
    } else {
        return PLATFORM_SPI_ERROR;
    }
}

void platform_spi_cs_low(platform_spi_cs_E cs_pin)
{
    if (cs_pin < PLATFORM_SPI_CS_COUNT) {
        // Select the SPI peripheral for this CS pin
        current_spi = cs_map[cs_pin].hspi;
        // Assert CS low
        HAL_GPIO_WritePin(cs_map[cs_pin].port, cs_map[cs_pin].pin, GPIO_PIN_RESET);
    }
}

void platform_spi_cs_high(platform_spi_cs_E cs_pin)
{
    if (cs_pin < PLATFORM_SPI_CS_COUNT) {
        // Deassert CS high
        HAL_GPIO_WritePin(cs_map[cs_pin].port, cs_map[cs_pin].pin, GPIO_PIN_SET);
    }
}

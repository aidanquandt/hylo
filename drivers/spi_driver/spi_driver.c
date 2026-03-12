/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "spi_driver.h"
#include "common.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"

#define SPI_TIMEOUT_MS (5U)

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
    SPI_HandleTypeDef* hspi;
} spi_cs_map_t;

#if (HWREV == 0)
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi5;
#elif (HWREV == 1)
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi2;
#endif

STATIC const spi_cs_map_t cs_map[] = {
#if (HWREV == 0)
    [SPI_DRIVER_CS_UWB] = {SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, &hspi1},
#elif (HWREV == 1)
    [SPI_DRIVER_CS_UWB]   = {SPI4_CSn_GPIO_Port, SPI4_CSn_Pin, &hspi4},
    [SPI_DRIVER_CS_IMU_0] = {BMI_CS2_GPIO_Port,  BMI_CS2_Pin,  &hspi2},
    [SPI_DRIVER_CS_IMU_1] = {BMI_CS1_GPIO_Port,  BMI_CS1_Pin,  &hspi2},
    [SPI_DRIVER_CS_IMU_2] = {BMI_CS3_GPIO_Port,  BMI_CS3_Pin,  &hspi2},
    [SPI_DRIVER_CS_IMU_3] = {BMI_CS4_GPIO_Port,  BMI_CS4_Pin,  &hspi2},
#endif
};

STATIC SPI_HandleTypeDef* current_spi             = NULL;
STATIC volatile uint32_t spi_transmit_count       = 0;
STATIC volatile uint32_t spi_receive_count        = 0;
STATIC volatile uint32_t spi_cs_low_count        = 0;
STATIC volatile uint32_t spi_cs_high_count        = 0;
STATIC volatile HAL_StatusTypeDef last_hal_status = HAL_OK;

spi_driver_status_E spi_driver_transfer(const uint8_t* tx_data, uint8_t* rx_data,
                                        uint16_t length)
{
    HAL_StatusTypeDef status;

    if (tx_data != NULL && rx_data != NULL)
    {
        status = HAL_SPI_TransmitReceive(current_spi, (uint8_t*)tx_data, rx_data, length,
                                         SPI_TIMEOUT_MS);
    }
    else if (tx_data != NULL)
    {
        status = HAL_SPI_Transmit(current_spi, (uint8_t*)tx_data, length, SPI_TIMEOUT_MS);
    }
    else if (rx_data != NULL)
    {
        status = HAL_SPI_Receive(current_spi, rx_data, length, SPI_TIMEOUT_MS);
    }
    else
    {
        return SPI_DRIVER_ERROR;
    }

    if (status == HAL_OK)
        return SPI_DRIVER_SUCCESS;
    if (status == HAL_TIMEOUT)
        return SPI_DRIVER_TIMEOUT;
    return SPI_DRIVER_ERROR;
}

spi_driver_status_E spi_driver_transmit(const uint8_t* data, uint16_t length)
{
    spi_transmit_count++;
    if (data == NULL || current_spi == NULL)
        return SPI_DRIVER_ERROR;
    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(current_spi, (uint8_t*)data, length, SPI_TIMEOUT_MS);
    last_hal_status = status;
    if (status == HAL_OK)
        return SPI_DRIVER_SUCCESS;
    if (status == HAL_TIMEOUT)
        return SPI_DRIVER_TIMEOUT;
    return SPI_DRIVER_ERROR;
}

spi_driver_status_E spi_driver_receive(uint8_t* data, uint16_t length)
{
    spi_receive_count++;
    if (data == NULL || current_spi == NULL)
        return SPI_DRIVER_ERROR;
    HAL_StatusTypeDef status = HAL_SPI_Receive(current_spi, data, length, SPI_TIMEOUT_MS);
    last_hal_status = status;
    if (status == HAL_OK)
        return SPI_DRIVER_SUCCESS;
    if (status == HAL_TIMEOUT)
        return SPI_DRIVER_TIMEOUT;
    return SPI_DRIVER_ERROR;
}

spi_driver_status_E spi_driver_set_speed(spi_driver_speed_E speed)
{
    if (current_spi == NULL)
        return SPI_DRIVER_ERROR;
    if (speed == SPI_DRIVER_SPEED_SLOW)
        current_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    else
        current_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    return (HAL_SPI_Init(current_spi) == HAL_OK) ? SPI_DRIVER_SUCCESS : SPI_DRIVER_ERROR;
}

void spi_driver_cs_low(spi_driver_cs_E cs_pin)
{
    spi_cs_low_count++;
    if (cs_pin < SPI_DRIVER_CS_COUNT)
    {
        current_spi = cs_map[cs_pin].hspi;
        HAL_GPIO_WritePin(cs_map[cs_pin].port, cs_map[cs_pin].pin, GPIO_PIN_RESET);
    }
}

void spi_driver_cs_high(spi_driver_cs_E cs_pin)
{
    spi_cs_high_count++;
    if (cs_pin < SPI_DRIVER_CS_COUNT)
        HAL_GPIO_WritePin(cs_map[cs_pin].port, cs_map[cs_pin].pin, GPIO_PIN_SET);
}

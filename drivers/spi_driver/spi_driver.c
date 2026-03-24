/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "spi_driver.h"
#include "common.h"
#include "FreeRTOS.h"
#include "gpio.h"
#include "main.h"
#include "semphr.h"
#include "spi.h"

#define SPI_TIMEOUT_MS          (5U)
#define SPI_MUTEX_TIMEOUT_MS    (100U)

typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
    SPI_HandleTypeDef* hspi;
    uint8_t bus_id;
} spi_cs_map_t;

#if (HWREV == 0)
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi5;
#define SPI_DRIVER_BUS_COUNT (1U)
#elif (HWREV == 1)
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi2;
#define SPI_DRIVER_BUS_COUNT (2U)
#endif

STATIC const spi_cs_map_t cs_map[] = {
#if (HWREV == 0)
    [SPI_DRIVER_INTERFACE_UWB] = {SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, &hspi1, 0},
#elif (HWREV == 1)
    [SPI_DRIVER_INTERFACE_UWB]   = {SPI4_CSn_GPIO_Port, SPI4_CSn_Pin, &hspi4, 1},
    [SPI_DRIVER_INTERFACE_IMU_0] = {BMI_CS2_GPIO_Port,  BMI_CS2_Pin,  &hspi2, 0},
    [SPI_DRIVER_INTERFACE_IMU_1] = {BMI_CS1_GPIO_Port,  BMI_CS1_Pin,  &hspi2, 0},
    [SPI_DRIVER_INTERFACE_IMU_2] = {BMI_CS3_GPIO_Port,  BMI_CS3_Pin,  &hspi2, 0},
    [SPI_DRIVER_INTERFACE_IMU_3] = {BMI_CS4_GPIO_Port,  BMI_CS4_Pin,  &hspi2, 0},
#endif
};

STATIC SemaphoreHandle_t bus_mutex[SPI_DRIVER_BUS_COUNT];
STATIC volatile bool init_done = false;

/** Per-bus session handle: which hspi is in session on this bus. NULL = no session. */
STATIC SPI_HandleTypeDef* session_hspi[SPI_DRIVER_BUS_COUNT];
STATIC volatile uint32_t spi_transmit_count       = 0;
STATIC volatile uint32_t spi_receive_count       = 0;
STATIC volatile uint32_t spi_cs_low_count        = 0;
STATIC volatile uint32_t spi_cs_high_count        = 0;
STATIC volatile HAL_StatusTypeDef last_hal_status = HAL_OK;

STATIC void cs_low(spi_driver_interface_E interface);
STATIC void cs_high(spi_driver_interface_E interface);

/** Validate interface and return bus_id; return SPI_DRIVER_BUS_COUNT if invalid. */
STATIC uint8_t get_bus_id(spi_driver_interface_E interface)
{
    if (interface >= SPI_DRIVER_INTERFACE_COUNT)
        return (uint8_t)SPI_DRIVER_BUS_COUNT;
    if (cs_map[interface].hspi == NULL)
        return (uint8_t)SPI_DRIVER_BUS_COUNT;
    return cs_map[interface].bus_id;
}

STATIC void cs_low(spi_driver_interface_E interface)
{
    spi_cs_low_count++;
    if (interface < SPI_DRIVER_INTERFACE_COUNT && cs_map[interface].hspi != NULL)
    {
        uint8_t bus_id = cs_map[interface].bus_id;
        session_hspi[bus_id] = cs_map[interface].hspi;
        HAL_GPIO_WritePin(cs_map[interface].port, cs_map[interface].pin, GPIO_PIN_RESET);
    }
}

STATIC void cs_high(spi_driver_interface_E interface)
{
    spi_cs_high_count++;
    if (interface < SPI_DRIVER_INTERFACE_COUNT)
        HAL_GPIO_WritePin(cs_map[interface].port, cs_map[interface].pin, GPIO_PIN_SET);
}

void spi_driver_init(void)
{
    for (size_t i = 0; i < SPI_DRIVER_BUS_COUNT; i++)
    {
        bus_mutex[i] = xSemaphoreCreateMutex();
        if (bus_mutex[i] == NULL)
        {
            /* Fail explicitly: do not leave partial state. */
            for (size_t j = 0; j < i; j++)
            {
                vSemaphoreDelete(bus_mutex[j]);
                bus_mutex[j] = NULL;
            }
            return;
        }
    }
    init_done = true;
}

static spi_driver_status_E take_bus_mutex(uint8_t bus_id)
{
    if (!init_done || bus_id >= SPI_DRIVER_BUS_COUNT || bus_mutex[bus_id] == NULL)
        return SPI_DRIVER_ERROR;
    if (xSemaphoreTake(bus_mutex[bus_id], pdMS_TO_TICKS(SPI_MUTEX_TIMEOUT_MS)) != pdTRUE)
        return SPI_DRIVER_TIMEOUT;
    return SPI_DRIVER_SUCCESS;
}

static void give_bus_mutex(uint8_t bus_id)
{
    if (bus_id < SPI_DRIVER_BUS_COUNT && bus_mutex[bus_id] != NULL)
        xSemaphoreGive(bus_mutex[bus_id]);
}

spi_driver_status_E spi_driver_acquire(spi_driver_interface_E interface)
{
    uint8_t bus_id = get_bus_id(interface);
    if (bus_id >= SPI_DRIVER_BUS_COUNT)
        return SPI_DRIVER_ERROR;
    spi_driver_status_E st = take_bus_mutex(bus_id);
    if (st != SPI_DRIVER_SUCCESS)
        return st;
    cs_low(interface);
    return SPI_DRIVER_SUCCESS;
}

void spi_driver_release(spi_driver_interface_E interface)
{
    uint8_t bus_id = get_bus_id(interface);
    cs_high(interface);
    /* Only give mutex if we actually hold it (session was started by acquire). */
    if (bus_id < SPI_DRIVER_BUS_COUNT && session_hspi[bus_id] != NULL)
    {
        session_hspi[bus_id] = NULL;
        give_bus_mutex(bus_id);
    }
}

spi_driver_status_E spi_driver_transfer(spi_driver_interface_E interface,
                                        const uint8_t* tx_data, uint8_t* rx_data, uint16_t length)
{
    uint8_t bus_id = get_bus_id(interface);
    if (bus_id >= SPI_DRIVER_BUS_COUNT)
        return SPI_DRIVER_ERROR;

    spi_driver_status_E st = take_bus_mutex(bus_id);
    if (st != SPI_DRIVER_SUCCESS)
        return st;

    cs_low(interface);

    HAL_StatusTypeDef status;
    if (tx_data != NULL && rx_data != NULL)
        status = HAL_SPI_TransmitReceive(cs_map[interface].hspi, (uint8_t*)tx_data, rx_data,
                                         length, SPI_TIMEOUT_MS);
    else if (tx_data != NULL)
        status = HAL_SPI_Transmit(cs_map[interface].hspi, (uint8_t*)tx_data, length,
                                 SPI_TIMEOUT_MS);
    else if (rx_data != NULL)
        status = HAL_SPI_Receive(cs_map[interface].hspi, rx_data, length, SPI_TIMEOUT_MS);
    else
    {
        cs_high(interface);
        session_hspi[bus_id] = NULL;
        give_bus_mutex(bus_id);
        return SPI_DRIVER_ERROR;
    }

    cs_high(interface);
    session_hspi[bus_id] = NULL; /* atomic path: no session, clear so release() won't give again */
    give_bus_mutex(bus_id);

    last_hal_status = status;
    if (status == HAL_OK)
        return SPI_DRIVER_SUCCESS;
    if (status == HAL_TIMEOUT)
        return SPI_DRIVER_TIMEOUT;
    return SPI_DRIVER_ERROR;
}

spi_driver_status_E spi_driver_transmit(spi_driver_interface_E interface, const uint8_t* data,
                                        uint16_t length)
{
    if (data == NULL)
        return SPI_DRIVER_ERROR;
    return spi_driver_transfer(interface, data, NULL, length);
}

spi_driver_status_E spi_driver_receive(spi_driver_interface_E interface, uint8_t* data,
                                       uint16_t length)
{
    if (data == NULL)
        return SPI_DRIVER_ERROR;
    return spi_driver_transfer(interface, NULL, data, length);
}

/* Session-only: pass same interface as acquire(); uses per-bus session state. */
spi_driver_status_E spi_driver_transfer_in_session(spi_driver_interface_E interface,
                                                   const uint8_t* tx_data, uint8_t* rx_data,
                                                   uint16_t length)
{
    uint8_t bus_id = get_bus_id(interface);
    if (bus_id >= SPI_DRIVER_BUS_COUNT || session_hspi[bus_id] == NULL)
        return SPI_DRIVER_ERROR;

    SPI_HandleTypeDef* hspi = session_hspi[bus_id];
    HAL_StatusTypeDef status;
    if (tx_data != NULL && rx_data != NULL)
        status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)tx_data, rx_data, length,
                                        SPI_TIMEOUT_MS);
    else if (tx_data != NULL)
        status = HAL_SPI_Transmit(hspi, (uint8_t*)tx_data, length, SPI_TIMEOUT_MS);
    else if (rx_data != NULL)
        status = HAL_SPI_Receive(hspi, rx_data, length, SPI_TIMEOUT_MS);
    else
        return SPI_DRIVER_ERROR;

    last_hal_status = status;
    if (status == HAL_OK)
        return SPI_DRIVER_SUCCESS;
    if (status == HAL_TIMEOUT)
        return SPI_DRIVER_TIMEOUT;
    return SPI_DRIVER_ERROR;
}

spi_driver_status_E spi_driver_transmit_in_session(spi_driver_interface_E interface,
                                                   const uint8_t* data, uint16_t length)
{
    uint8_t bus_id = get_bus_id(interface);
    if (data == NULL || bus_id >= SPI_DRIVER_BUS_COUNT || session_hspi[bus_id] == NULL)
        return SPI_DRIVER_ERROR;
    spi_transmit_count++;
    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(session_hspi[bus_id], (uint8_t*)data, length, SPI_TIMEOUT_MS);
    last_hal_status = status;
    if (status == HAL_OK)
        return SPI_DRIVER_SUCCESS;
    if (status == HAL_TIMEOUT)
        return SPI_DRIVER_TIMEOUT;
    return SPI_DRIVER_ERROR;
}

spi_driver_status_E spi_driver_receive_in_session(spi_driver_interface_E interface,
                                                  uint8_t* data, uint16_t length)
{
    uint8_t bus_id = get_bus_id(interface);
    if (data == NULL || bus_id >= SPI_DRIVER_BUS_COUNT || session_hspi[bus_id] == NULL)
        return SPI_DRIVER_ERROR;
    spi_receive_count++;
    HAL_StatusTypeDef status =
        HAL_SPI_Receive(session_hspi[bus_id], data, length, SPI_TIMEOUT_MS);
    last_hal_status = status;
    if (status == HAL_OK)
        return SPI_DRIVER_SUCCESS;
    if (status == HAL_TIMEOUT)
        return SPI_DRIVER_TIMEOUT;
    return SPI_DRIVER_ERROR;
}

spi_driver_status_E spi_driver_set_speed(spi_driver_interface_E interface,
                                        spi_driver_speed_E speed)
{
    uint8_t bus_id = get_bus_id(interface);
    if (bus_id >= SPI_DRIVER_BUS_COUNT || session_hspi[bus_id] == NULL)
        return SPI_DRIVER_ERROR;
    SPI_HandleTypeDef* hspi = session_hspi[bus_id];
    if (speed == SPI_DRIVER_SPEED_SLOW)
        hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    else
        hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    return (HAL_SPI_Init(hspi) == HAL_OK) ? SPI_DRIVER_SUCCESS : SPI_DRIVER_ERROR;
}

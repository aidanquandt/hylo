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
#include "task.h"
#include <string.h>

#define SPI_TIMEOUT_MS           (5U)
#define SPI_MUTEX_TIMEOUT_MS     (100U)
/** DW3000 stack may TX up to this in one SPI leg (deca_device_api.h). */
#define SPI_DMA_MAX_LEN          (1024U)
/** Below this length, use blocking HAL to avoid DMA+ISR overhead. */
#define SPI_DMA_MIN_LENGTH       (8U)

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
STATIC SemaphoreHandle_t dma_done_sem[SPI_DRIVER_BUS_COUNT];
STATIC volatile bool init_done = false;

/** Per-bus session handle: which hspi is in session on this bus. NULL = no session. */
STATIC SPI_HandleTypeDef* session_hspi[SPI_DRIVER_BUS_COUNT];
STATIC volatile uint32_t spi_transmit_count       = 0;
STATIC volatile uint32_t spi_receive_count       = 0;
STATIC volatile uint32_t spi_cs_low_count        = 0;
STATIC volatile uint32_t spi_cs_high_count        = 0;
STATIC volatile HAL_StatusTypeDef last_hal_status = HAL_OK;

/* DMA buffers in DMA-accessible RAM (RAM_D1 .dma_buffer). DTCM is not reachable by DMA on H7. */
__attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))
STATIC uint8_t spi_tx_dma_buf[SPI_DRIVER_BUS_COUNT][SPI_DMA_MAX_LEN];
__attribute__((section(".dma_buffer"))) __attribute__((aligned(32)))
STATIC uint8_t spi_rx_dma_buf[SPI_DRIVER_BUS_COUNT][SPI_DMA_MAX_LEN];

STATIC void cs_low(spi_driver_interface_E interface);
STATIC void cs_high(spi_driver_interface_E interface);

STATIC uint8_t bus_id_from_hspi(SPI_HandleTypeDef* hspi)
{
    if (hspi == NULL)
        return (uint8_t)SPI_DRIVER_BUS_COUNT;
#if (HWREV == 0)
    if (hspi->Instance == SPI1)
        return 0U;
    (void)hspi5;
#elif (HWREV == 1)
    if (hspi->Instance == SPI2)
        return 0U;
    if (hspi->Instance == SPI4)
        return 1U;
#endif
    return (uint8_t)SPI_DRIVER_BUS_COUNT;
}

STATIC bool hspi_dma_capable(SPI_HandleTypeDef* hspi)
{
    return hspi != NULL && hspi->hdmarx != NULL && hspi->hdmatx != NULL;
}

STATIC void dma_complete_from_isr(uint8_t bus_id)
{
    BaseType_t woken = pdFALSE;
    if (bus_id < SPI_DRIVER_BUS_COUNT && dma_done_sem[bus_id] != NULL)
        xSemaphoreGiveFromISR(dma_done_sem[bus_id], &woken);
    portYIELD_FROM_ISR(woken);
}

STATIC spi_driver_status_E wait_dma_complete(uint8_t bus_id, SPI_HandleTypeDef* hspi)
{
    if (xSemaphoreTake(dma_done_sem[bus_id], pdMS_TO_TICKS(SPI_TIMEOUT_MS)) != pdTRUE)
    {
        (void)HAL_SPI_Abort(hspi);
        return SPI_DRIVER_TIMEOUT;
    }
    if (HAL_SPI_GetState(hspi) != HAL_SPI_STATE_READY || hspi->ErrorCode != HAL_SPI_ERROR_NONE)
        return SPI_DRIVER_ERROR;
    return SPI_DRIVER_SUCCESS;
}

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
    size_t i;

    for (i = 0; i < SPI_DRIVER_BUS_COUNT; i++)
    {
        bus_mutex[i]    = xSemaphoreCreateMutex();
        dma_done_sem[i] = xSemaphoreCreateBinary();
        if (bus_mutex[i] == NULL || dma_done_sem[i] == NULL)
        {
            for (size_t j = 0; j < i; j++)
            {
                vSemaphoreDelete(bus_mutex[j]);
                bus_mutex[j] = NULL;
                vSemaphoreDelete(dma_done_sem[j]);
                dma_done_sem[j] = NULL;
            }
            if (bus_mutex[i] != NULL)
                vSemaphoreDelete(bus_mutex[i]);
            if (dma_done_sem[i] != NULL)
                vSemaphoreDelete(dma_done_sem[i]);
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

/**
 * Run one SPI transfer on the given bus/hspi.
 * When @p use_dma is true, caller pointers are not passed to HAL — data is copied via .dma_buffer.
 */
STATIC spi_driver_status_E spi_run_transfer(uint8_t bus_id, SPI_HandleTypeDef* hspi, const uint8_t* tx_data,
                                           uint8_t* rx_data, uint16_t length, bool use_dma)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (length > SPI_DMA_MAX_LEN)
        return SPI_DRIVER_ERROR;

    if (use_dma && hspi_dma_capable(hspi))
    {
        if (tx_data != NULL && rx_data != NULL)
        {
            memcpy(spi_tx_dma_buf[bus_id], tx_data, length);
            status = HAL_SPI_TransmitReceive_DMA(hspi, spi_tx_dma_buf[bus_id], spi_rx_dma_buf[bus_id],
                                                 length);
            last_hal_status = status;
            if (status != HAL_OK)
                return SPI_DRIVER_ERROR;
            {
                spi_driver_status_E w = wait_dma_complete(bus_id, hspi);
                if (w != SPI_DRIVER_SUCCESS)
                    return w;
            }
            memcpy(rx_data, spi_rx_dma_buf[bus_id], length);
            return SPI_DRIVER_SUCCESS;
        }
        if (tx_data != NULL)
        {
            memcpy(spi_tx_dma_buf[bus_id], tx_data, length);
            status = HAL_SPI_Transmit_DMA(hspi, spi_tx_dma_buf[bus_id], length);
            last_hal_status = status;
            if (status != HAL_OK)
                return SPI_DRIVER_ERROR;
            return wait_dma_complete(bus_id, hspi);
        }
        if (rx_data != NULL)
        {
            status = HAL_SPI_Receive_DMA(hspi, spi_rx_dma_buf[bus_id], length);
            last_hal_status = status;
            if (status != HAL_OK)
                return SPI_DRIVER_ERROR;
            {
                spi_driver_status_E w = wait_dma_complete(bus_id, hspi);
                if (w != SPI_DRIVER_SUCCESS)
                    return w;
            }
            memcpy(rx_data, spi_rx_dma_buf[bus_id], length);
            return SPI_DRIVER_SUCCESS;
        }
        return SPI_DRIVER_ERROR;
    }

    if (tx_data != NULL && rx_data != NULL)
        status = HAL_SPI_TransmitReceive(hspi, (uint8_t*)tx_data, rx_data, length, SPI_TIMEOUT_MS);
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

STATIC bool use_dma_for_length(uint16_t length, SPI_HandleTypeDef* hspi)
{
    return length >= SPI_DMA_MIN_LENGTH && hspi_dma_capable(hspi);
}

spi_driver_status_E spi_driver_transfer(spi_driver_interface_E interface, const uint8_t* tx_data,
                                        uint8_t* rx_data, uint16_t length)
{
    uint8_t bus_id = get_bus_id(interface);
    if (bus_id >= SPI_DRIVER_BUS_COUNT || length == 0U)
        return SPI_DRIVER_ERROR;

    spi_driver_status_E st = take_bus_mutex(bus_id);
    if (st != SPI_DRIVER_SUCCESS)
        return st;

    cs_low(interface);

    SPI_HandleTypeDef* hspi = cs_map[interface].hspi;
    st = spi_run_transfer(bus_id, hspi, tx_data, rx_data, length, use_dma_for_length(length, hspi));

    cs_high(interface);
    session_hspi[bus_id] = NULL;
    give_bus_mutex(bus_id);

    return st;
}

spi_driver_status_E spi_driver_transmit(spi_driver_interface_E interface, const uint8_t* data,
                                        uint16_t length)
{
    if (data == NULL)
        return SPI_DRIVER_ERROR;
    return spi_driver_transfer(interface, data, NULL, length);
}

spi_driver_status_E spi_driver_receive(spi_driver_interface_E interface, uint8_t* data, uint16_t length)
{
    if (data == NULL)
        return SPI_DRIVER_ERROR;
    return spi_driver_transfer(interface, NULL, data, length);
}

spi_driver_status_E spi_driver_transfer_in_session(spi_driver_interface_E interface, const uint8_t* tx_data,
                                                   uint8_t* rx_data, uint16_t length)
{
    uint8_t bus_id = get_bus_id(interface);
    if (bus_id >= SPI_DRIVER_BUS_COUNT || session_hspi[bus_id] == NULL || length == 0U)
        return SPI_DRIVER_ERROR;

    SPI_HandleTypeDef* hspi = session_hspi[bus_id];
    return spi_run_transfer(bus_id, hspi, tx_data, rx_data, length, use_dma_for_length(length, hspi));
}

spi_driver_status_E spi_driver_transmit_in_session(spi_driver_interface_E interface, const uint8_t* data,
                                                   uint16_t length)
{
    uint8_t bus_id = get_bus_id(interface);
    if (data == NULL || bus_id >= SPI_DRIVER_BUS_COUNT || session_hspi[bus_id] == NULL || length == 0U)
        return SPI_DRIVER_ERROR;
    spi_transmit_count++;
    return spi_run_transfer(bus_id, session_hspi[bus_id], data, NULL, length,
                            use_dma_for_length(length, session_hspi[bus_id]));
}

spi_driver_status_E spi_driver_receive_in_session(spi_driver_interface_E interface, uint8_t* data,
                                                  uint16_t length)
{
    uint8_t bus_id = get_bus_id(interface);
    if (data == NULL || bus_id >= SPI_DRIVER_BUS_COUNT || session_hspi[bus_id] == NULL || length == 0U)
        return SPI_DRIVER_ERROR;
    spi_receive_count++;
    return spi_run_transfer(bus_id, session_hspi[bus_id], NULL, data, length,
                            use_dma_for_length(length, session_hspi[bus_id]));
}

spi_driver_status_E spi_driver_set_speed(spi_driver_interface_E interface, spi_driver_speed_E speed)
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

/*---------------------------------------------------------------------------
 * HAL weak callbacks — DMA completion (same idea as uart_driver)
 *---------------------------------------------------------------------------*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    dma_complete_from_isr(bus_id_from_hspi(hspi));
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    dma_complete_from_isr(bus_id_from_hspi(hspi));
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    dma_complete_from_isr(bus_id_from_hspi(hspi));
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    dma_complete_from_isr(bus_id_from_hspi(hspi));
}

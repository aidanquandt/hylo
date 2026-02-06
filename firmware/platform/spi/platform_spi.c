/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "platform_spi.h"
#include "common.h"
#include "feature_config.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

/*---------------------------------------------------------------------------
 * DMA Usage Pattern:
 * 
 * FEATURE_SPI_USE_DMA toggles between blocking and non-blocking SPI transfers:
 * 
 * FEATURE_SPI_USE_DMA = 0 (Default):
 *   - Uses blocking HAL_SPI_* functions
 *   - CPU waits during each transfer (~10-20μs)
 *   - No additional hardware configuration required
 *   - Simple, reliable operation
 * 
 * FEATURE_SPI_USE_DMA = 1 (After CubeMX configuration):
 *   - Uses non-blocking HAL_SPI_*_DMA functions
 *   - CPU continues executing, DMA handles transfer
 *   - Task blocks waiting for interrupt notification
 *   - Requires CubeMX configuration:
 *     1. SPI1 DMA channels (TX: DMA1 Stream 3, RX: DMA1 Stream 2)
 *     2. SPI5 DMA channels (TX: DMA2 Stream 4, RX: DMA2 Stream 3)
 *     3. NVIC priorities: DMA IRQ < FreeRTOS (priority 5+)
 *     4. Regenerate code with CubeMX
 * 
 * To enable DMA:
 *   1. Configure hardware in CubeMX (cubemx.ioc)
 *   2. Set FEATURE_SPI_USE_DMA = 1 in feature_config.h
 *   3. Rebuild firmware
 * 
 * No application code changes needed - same API works for both modes!
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SPI_TIMEOUT_MS (100U)

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef struct
{
    GPIO_TypeDef* port;
    uint16_t pin;
    SPI_HandleTypeDef* hspi;
} spi_cs_map_t;

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi5;

STATIC const spi_cs_map_t cs_map[] = {
    [PLATFORM_SPI_CS_UWB] = {SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, &hspi1},
    [PLATFORM_SPI_CS_IMU] = {SPI5_CSn_GPIO_Port, SPI5_CSn_Pin, &hspi5},
};

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC SPI_HandleTypeDef* current_spi             = NULL;
STATIC volatile uint32_t spi_transmit_count       = 0;
STATIC volatile uint32_t spi_receive_count        = 0;
STATIC volatile uint32_t spi_cs_low_count         = 0;
STATIC volatile uint32_t spi_cs_high_count        = 0;
STATIC volatile HAL_StatusTypeDef last_hal_status = HAL_OK;

/* DMA Support Variables */
STATIC TaskHandle_t spi_dma_task_handle           = NULL;
STATIC volatile platform_spi_status_E dma_result  = PLATFORM_SPI_SUCCESS;
STATIC volatile uint32_t spi_dma_complete_count   = 0;
STATIC volatile uint32_t spi_dma_error_count      = 0;

/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

#if FEATURE_SPI_USE_DMA
STATIC platform_spi_status_E wait_for_dma_completion(uint32_t timeout_ms)
{
    uint32_t notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));
    
    if (notification == 0)
    {
        return PLATFORM_SPI_TIMEOUT;
    }
    
    return dma_result;
}
#endif

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/

platform_spi_status_E platform_spi_transfer(const uint8_t* tx_data, uint8_t* rx_data,
                                            uint16_t length)
{
#if FEATURE_SPI_USE_DMA
    /* DMA mode - non-blocking transfers with task notification */
    platform_spi_status_E status;
    
    if (tx_data != NULL && rx_data != NULL)
    {
        status = platform_spi_transfer_dma(tx_data, rx_data, length);
    }
    else if (tx_data != NULL)
    {
        status = platform_spi_transmit_dma(tx_data, length);
    }
    else if (rx_data != NULL)
    {
        status = platform_spi_receive_dma(rx_data, length);
    }
    else
    {
        return PLATFORM_SPI_ERROR;
    }
    
    if (status != PLATFORM_SPI_SUCCESS)
    {
        return status;
    }
    
    return wait_for_dma_completion(SPI_TIMEOUT_MS);
#else
    /* Blocking mode - original implementation */
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
        return PLATFORM_SPI_ERROR;
    }

    if (status == HAL_OK)
    {
        return PLATFORM_SPI_SUCCESS;
    }
    else if (status == HAL_TIMEOUT)
    {
        return PLATFORM_SPI_TIMEOUT;
    }
    else
    {
        return PLATFORM_SPI_ERROR;
    }
#endif
}

platform_spi_status_E platform_spi_transmit(const uint8_t* data, uint16_t length)
{
    spi_transmit_count++;

    if (data == NULL || current_spi == NULL)
    {
        return PLATFORM_SPI_ERROR;
    }

#if FEATURE_SPI_USE_DMA
    /* DMA mode */
    platform_spi_status_E status = platform_spi_transmit_dma(data, length);
    if (status != PLATFORM_SPI_SUCCESS)
    {
        return status;
    }
    return wait_for_dma_completion(SPI_TIMEOUT_MS);
#else
    /* Blocking mode */
    HAL_StatusTypeDef status =
        HAL_SPI_Transmit(current_spi, (uint8_t*)data, length, SPI_TIMEOUT_MS);
    last_hal_status = status;

    if (status == HAL_OK)
    {
        return PLATFORM_SPI_SUCCESS;
    }
    else if (status == HAL_TIMEOUT)
    {
        return PLATFORM_SPI_TIMEOUT;
    }
    else
    {
        return PLATFORM_SPI_ERROR;
    }
#endif
}

platform_spi_status_E platform_spi_receive(uint8_t* data, uint16_t length)
{
    spi_receive_count++;

    if (data == NULL || current_spi == NULL)
    {
        return PLATFORM_SPI_ERROR;
    }

#if FEATURE_SPI_USE_DMA
    /* DMA mode */
    platform_spi_status_E status = platform_spi_receive_dma(data, length);
    if (status != PLATFORM_SPI_SUCCESS)
    {
        return status;
    }
    return wait_for_dma_completion(SPI_TIMEOUT_MS);
#else
    /* Blocking mode */
    HAL_StatusTypeDef status = HAL_SPI_Receive(current_spi, data, length, SPI_TIMEOUT_MS);
    last_hal_status          = status;

    if (status == HAL_OK)
    {
        return PLATFORM_SPI_SUCCESS;
    }
    else if (status == HAL_TIMEOUT)
    {
        return PLATFORM_SPI_TIMEOUT;
    }
    else
    {
        return PLATFORM_SPI_ERROR;
    }
#endif
}

platform_spi_status_E platform_spi_set_speed(platform_spi_speed_E speed)
{
    if (current_spi == NULL)
    {
        return PLATFORM_SPI_ERROR;
    }

    if (speed == PLATFORM_SPI_SPEED_SLOW)
    {
        current_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    }
    else
    {
        current_spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    }

    if (HAL_SPI_Init(current_spi) == HAL_OK)
    {
        return PLATFORM_SPI_SUCCESS;
    }
    else
    {
        return PLATFORM_SPI_ERROR;
    }
}

void platform_spi_cs_low(platform_spi_cs_E cs_pin)
{
    spi_cs_low_count++;

    if (cs_pin < PLATFORM_SPI_CS_COUNT)
    {
        current_spi = cs_map[cs_pin].hspi;
        HAL_GPIO_WritePin(cs_map[cs_pin].port, cs_map[cs_pin].pin, GPIO_PIN_RESET);
    }
}

void platform_spi_cs_high(platform_spi_cs_E cs_pin)
{
    spi_cs_high_count++;

    if (cs_pin < PLATFORM_SPI_CS_COUNT)
    {
        HAL_GPIO_WritePin(cs_map[cs_pin].port, cs_map[cs_pin].pin, GPIO_PIN_SET);
    }
}

/*---------------------------------------------------------------------------
 * DMA Transfer Functions (Non-blocking)
 * Note: Caller must wait for completion via task notification
 *---------------------------------------------------------------------------*/

platform_spi_status_E platform_spi_transmit_dma(const uint8_t* data, uint16_t length)
{
    if (data == NULL || current_spi == NULL)
    {
        return PLATFORM_SPI_ERROR;
    }

    /* Store calling task handle for notification */
    spi_dma_task_handle = xTaskGetCurrentTaskHandle();
    dma_result = PLATFORM_SPI_SUCCESS;

    /* Start DMA transfer */
    HAL_StatusTypeDef status = HAL_SPI_Transmit_DMA(current_spi, (uint8_t*)data, length);
    
    if (status != HAL_OK)
    {
        spi_dma_task_handle = NULL;
        return (status == HAL_TIMEOUT) ? PLATFORM_SPI_TIMEOUT : PLATFORM_SPI_ERROR;
    }

    return PLATFORM_SPI_SUCCESS;
}

platform_spi_status_E platform_spi_receive_dma(uint8_t* data, uint16_t length)
{
    if (data == NULL || current_spi == NULL)
    {
        return PLATFORM_SPI_ERROR;
    }

    /* Store calling task handle for notification */
    spi_dma_task_handle = xTaskGetCurrentTaskHandle();
    dma_result = PLATFORM_SPI_SUCCESS;

    /* Start DMA transfer */
    HAL_StatusTypeDef status = HAL_SPI_Receive_DMA(current_spi, data, length);
    
    if (status != HAL_OK)
    {
        spi_dma_task_handle = NULL;
        return (status == HAL_TIMEOUT) ? PLATFORM_SPI_TIMEOUT : PLATFORM_SPI_ERROR;
    }

    return PLATFORM_SPI_SUCCESS;
}

platform_spi_status_E platform_spi_transfer_dma(const uint8_t* tx_data, uint8_t* rx_data,
                                                uint16_t length)
{
    if (tx_data == NULL || rx_data == NULL || current_spi == NULL)
    {
        return PLATFORM_SPI_ERROR;
    }

    /* Store calling task handle for notification */
    spi_dma_task_handle = xTaskGetCurrentTaskHandle();
    dma_result = PLATFORM_SPI_SUCCESS;

    /* Start DMA transfer */
    HAL_StatusTypeDef status = HAL_SPI_TransmitReceive_DMA(current_spi, (uint8_t*)tx_data,
                                                           rx_data, length);
    
    if (status != HAL_OK)
    {
        spi_dma_task_handle = NULL;
        return (status == HAL_TIMEOUT) ? PLATFORM_SPI_TIMEOUT : PLATFORM_SPI_ERROR;
    }

    return PLATFORM_SPI_SUCCESS;
}

/*---------------------------------------------------------------------------
 * HAL SPI DMA Callbacks
 * Note: Called from interrupt context
 *---------------------------------------------------------------------------*/

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef* hspi)
{
    spi_dma_complete_count++;

    if (spi_dma_task_handle != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(spi_dma_task_handle, &xHigherPriorityTaskWoken);
        spi_dma_task_handle = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef* hspi)
{
    spi_dma_complete_count++;

    if (spi_dma_task_handle != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(spi_dma_task_handle, &xHigherPriorityTaskWoken);
        spi_dma_task_handle = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
    spi_dma_complete_count++;

    if (spi_dma_task_handle != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(spi_dma_task_handle, &xHigherPriorityTaskWoken);
        spi_dma_task_handle = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
    spi_dma_error_count++;
    dma_result = PLATFORM_SPI_ERROR;

    if (spi_dma_task_handle != NULL)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(spi_dma_task_handle, &xHigherPriorityTaskWoken);
        spi_dma_task_handle = NULL;
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

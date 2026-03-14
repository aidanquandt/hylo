/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "sdcard_driver.h"

#if (HWREV == 1)
#include "fatfs.h"
#endif

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "uart_driver.h"
#include "stm32h7xx_hal.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SDCARD_QUEUE_SIZE        (32U)
#define SDCARD_LOG_FILENAME      "DATA.CSV"
#define SDCARD_SYNC_INTERVAL     (10U)
#define SDCARD_TASK_STACK_SIZE   (1024U)
#define SDCARD_TASK_PRIORITY     (2U)
#define SDCARD_TASK_PERIOD_MS    (100U)

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
#if (HWREV == 1)

static QueueHandle_t sdcard_queue = NULL;

static StaticTask_t sdcard_task_tcb;
static StackType_t  sdcard_task_stack[SDCARD_TASK_STACK_SIZE];

static TaskHandle_t sdcard_task_handle = NULL;

static bool          sdcard_mounted = false;
static bool          file_open = false;
static uint32_t      rows_since_sync = 0U;

#endif

extern SD_HandleTypeDef hsd2;
/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
#if (HWREV == 1)
static void sdcard_driver_task(void* argument);
static void sdcard_write_csv_header(void);
static void sdcard_write_event(const sdcard_driver_event_t* event);
static bool sdcard_mount_and_open(void);
#endif

/*---------------------------------------------------------------------------
 * Public Functions
 *---------------------------------------------------------------------------*/
 static void sdcard_lowlevel_test(void)
{
    HAL_SD_CardInfoTypeDef cardInfo;

    if (HAL_SD_GetCardInfo(&hsd2, &cardInfo) != HAL_OK)
    {
        uart_driver_print("[SDCard] HAL_SD_GetCardInfo FAILED\r\n");
        return;
    }

    uint64_t capacity =
        (uint64_t)cardInfo.BlockNbr * cardInfo.BlockSize;

    uart_driver_print("[SDCard] Block size: %lu\r\n", cardInfo.BlockSize);
    uart_driver_print("[SDCard] Block count: %lu\r\n", cardInfo.BlockNbr);
    uart_driver_print("[SDCard] Capacity: (%.2f GB)\r\n", capacity / 1e9);
    uart_driver_print("[SDCard] Card type: %d\r\n", cardInfo.CardType);
}

void sdcard_driver_init(void)
{
#if (HWREV == 1)
    if (sdcard_queue != NULL)
        return;

    sdcard_queue = xQueueCreate(SDCARD_QUEUE_SIZE, sizeof(sdcard_driver_event_t));
    
    if (!sdcard_queue)
        return;

    sdcard_task_handle = xTaskCreateStatic(
                        sdcard_driver_task,
                        "SD_LOG",
                        SDCARD_TASK_STACK_SIZE,
                        NULL,
                        SDCARD_TASK_PRIORITY,
                        sdcard_task_stack,
                        &sdcard_task_tcb);
#endif
}

sdcard_driver_push_status_E sdcard_driver_push_event(const sdcard_driver_event_t* event)
{
#if (HWREV == 1)
    if (!event)
        return SDCARD_DRIVER_PUSH_ERROR_NULL_PTR;

    if (event->type >= NUM_SDCARD_DRIVER_EVENT_TYPES)
        return SDCARD_DRIVER_PUSH_ERROR_INVALID_TYPE;

    if (!sdcard_queue)
        return SDCARD_DRIVER_PUSH_ERROR_NOT_INITIALIZED;

    if (xQueueSend(sdcard_queue, event, 0) != pdTRUE)
        return SDCARD_DRIVER_PUSH_ERROR_QUEUE_FULL;

    return SDCARD_DRIVER_PUSH_SUCCESS;
#else
    (void)event;
    return SDCARD_DRIVER_PUSH_ERROR_NOT_INITIALIZED;
#endif
}

bool sdcard_driver_is_logging(void)
{
#if (HWREV == 1)
    return sdcard_mounted && file_open;
#else
    return false;
#endif
}

/*---------------------------------------------------------------------------
 * Private Functions
 *---------------------------------------------------------------------------*/
#if (HWREV == 1)

static bool sdcard_mount_and_open(void)
{
    FRESULT result;

    file_open = false;

    // Mount filesystem
    result = f_mount(&SDFatFS, SDPath, 1);
    if (result != FR_OK)
    {
        sdcard_mounted = false;
        (void)uart_driver_print("[SDCard] mount failed (FRESULT=%d)\r\n", (int)result);
        return false;
    }

    sdcard_mounted = true;

    // Open or create file
    result = f_open(&SDFile, SDCARD_LOG_FILENAME, FA_OPEN_ALWAYS | FA_WRITE);
    if (result != FR_OK)
    {
        (void)f_mount(NULL, SDPath, 1);
        sdcard_mounted = false;
        (void)uart_driver_print("[SDCard] open file failed (FRESULT=%d)\r\n", (int)result);
        return false;
    }

    file_open = true;

    // Write header if file is empty
    if (f_size(&SDFile) == 0)
    {
        sdcard_write_csv_header();
    }

    // Move write pointer to end of file
    result = f_lseek(&SDFile, f_size(&SDFile));
    if (result != FR_OK)
    {
        (void)f_close(&SDFile);
        (void)f_mount(NULL, SDPath, 1);
        file_open = false;
        sdcard_mounted = false;
        (void)uart_driver_print("[SDCard] seek failed (FRESULT=%d)\r\n", (int)result);
        return false;
    }

    rows_since_sync = 0;

    (void)uart_driver_print("[SDCard] mounted and file opened\r\n");
    return true;
}

static void sdcard_write_csv_header(void)
{
    static const char header[] =
        "type,timestamp_ms,"
        "dist_m,anchor_addr,anchor_x,anchor_y,anchor_z,quality,rssi_dbm,"
        "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,"
        "pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,confidence,pos_valid\n";

    UINT written;
    if (f_write(&SDFile, header, sizeof(header) - 1, &written) != FR_OK || written != sizeof(header) - 1)
    {
        file_open = false;
        (void)uart_driver_print("[SDCard] failed to write header\r\n");
    }
}

static void sdcard_write_event(const sdcard_driver_event_t* event)
{
    if (!event || !file_open)
        return;

    char row[256];
    int len = 0;

    switch (event->type)
    {
        case SDCARD_DRIVER_EVENT_RANGING:
        {
            const sdcard_driver_ranging_event_t* r = &event->data.ranging;
            len = snprintf(row, sizeof(row),
                           "RANGING,%lu,%.4f,%u,%.3f,%.3f,%.3f,%.3f,%.1f,"
                           ",,,,,,,,"
                           ",,,,,,\n",
                           (unsigned long)event->timestamp_ms,
                           r->distance_m,
                           (unsigned)r->anchor_addr,
                           r->anchor_x, r->anchor_y, r->anchor_z,
                           r->quality,
                           r->rssi_dbm);
            break;
        }

        case SDCARD_DRIVER_EVENT_IMU:
        {
            const sdcard_driver_imu_event_t* imu = &event->data.imu;
            len = snprintf(row, sizeof(row),
                           "IMU,%lu,,,,,,,"
                           "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,"
                           ",,,,,,\n",
                           (unsigned long)event->timestamp_ms,
                           imu->accel_x, imu->accel_y, imu->accel_z,
                           imu->gyro_x, imu->gyro_y, imu->gyro_z,
                           imu->temp_c);
            break;
        }

        case SDCARD_DRIVER_EVENT_POSITION:
        {
            const sdcard_driver_position_event_t* p = &event->data.position;
            len = snprintf(row, sizeof(row),
                           "POSITION,%lu,,,,,,,"
                           ",,,,,,,,"
                           "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%d\n",
                           (unsigned long)event->timestamp_ms,
                           p->x, p->y, p->z,
                           p->vx, p->vy, p->vz,
                           p->confidence,
                           p->valid);
            break;
        }

        default:
            return;
    }

    if (len <= 0)
        return;

    if ((UINT)len >= sizeof(row))
        len = sizeof(row) - 1; // truncate if necessary

    UINT written;
    if (f_write(&SDFile, row, (UINT)len, &written) != FR_OK || written != (UINT)len)
    {
        file_open = false;
        (void)uart_driver_print("[SDCard] write failed\r\n");
    }

    rows_since_sync++;
    if (rows_since_sync >= SDCARD_SYNC_INTERVAL)
    {
        if (f_sync(&SDFile) != FR_OK)
        {
            file_open = false;
            (void)uart_driver_print("[SDCard] sync failed\r\n");
        }
        rows_since_sync = 0;
    }
}

static void sdcard_driver_task(void* argument)
{
    (void)argument;

    vTaskDelay(pdMS_TO_TICKS(1000));
    sdcard_lowlevel_test(); 
    for (;;)
    {
        // Attempt to mount/open file if not already
        if (!sdcard_mounted || !file_open)
        {
            if (!sdcard_mount_and_open())
            {
                vTaskDelay(pdMS_TO_TICKS(1000)); // retry delay
                continue;
            }
        }

        // Process queued events
        sdcard_driver_event_t event;
        while (xQueueReceive(sdcard_queue, &event, 0) == pdTRUE)
        {
            sdcard_write_event(&event);
        }

        vTaskDelay(pdMS_TO_TICKS(SDCARD_TASK_PERIOD_MS));
    }
}
#endif
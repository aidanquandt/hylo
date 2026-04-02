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
#include "task_config.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SDCARD_QUEUE_SIZE         (512U)
#define SDCARD_SYNC_INTERVAL_MS  (5000U)
#define SDCARD_POLL_MS            (100U)   /* queue receive timeout so task can run sync check */
#define SDCARD_REMOUNT_DELAY_MS  (1000U)
#define SDCARD_MAX_LOG_FILES     (1000U)
#define SDCARD_ROW_BUF_SIZE      (320U)

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
#if (HWREV == 1)
static QueueHandle_t sdcard_queue = NULL;
static StaticTask_t  sdcard_task_tcb;
static StackType_t   sdcard_task_stack[TASK_STACK_4KB];

static bool     sdcard_mounted = false;
static bool     file_open = false;
static uint32_t last_sync_tick = 0U;
static char     current_log_filename[16];
#endif

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
void sdcard_driver_init(void)
{
#if (HWREV == 1)
    if (sdcard_queue != NULL) return;

    sdcard_queue = xQueueCreate(SDCARD_QUEUE_SIZE, sizeof(sdcard_driver_event_t));
    if (!sdcard_queue) return;

    (void)xTaskCreateStatic(
        sdcard_driver_task,
        "SD_LOG",
        TASK_STACK_4KB,
        NULL,
        TASK_PRIORITY_SDCARD,
        sdcard_task_stack,
        &sdcard_task_tcb);
#endif
}

sdcard_driver_push_status_E sdcard_driver_push_event(const sdcard_driver_event_t* event)
{
#if (HWREV == 1)
    if (!event) return SDCARD_DRIVER_PUSH_ERROR_NULL_PTR;
    if (!sdcard_queue) return SDCARD_DRIVER_PUSH_ERROR_NOT_INITIALIZED;

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

    result = f_mount(&SDFatFS, SDPath, 1);
    if (result != FR_OK) {
        sdcard_mounted = false;
        return false;
    }
    sdcard_mounted = true;

    /* Find next free slot LOG000.CSV .. LOG999.CSV */
    FILINFO fno;
    uint32_t file_index;
    for (file_index = 0U; file_index < SDCARD_MAX_LOG_FILES; file_index++) {
        snprintf(current_log_filename, sizeof(current_log_filename), "LOG%03lu.CSV", (unsigned long)file_index);
        if (f_stat(current_log_filename, &fno) != FR_OK)
            break;
    }
    if (file_index >= SDCARD_MAX_LOG_FILES)
        return false;

    result = f_open(&SDFile, current_log_filename, FA_CREATE_NEW | FA_WRITE);
    if (result != FR_OK) return false;

    file_open = true;
    sdcard_write_csv_header();
    last_sync_tick = xTaskGetTickCount();
    return true;
}

static void sdcard_write_csv_header(void)
{
    static const char header[] =
        "type,timestamp_ms,"
        "dist_m,anchor_addr,anchor_x,anchor_y,anchor_z,quality,rssi_dbm,"
        "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,"
        "pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,confidence,pos_valid,imu_enable\n";

    UINT written;
    if (f_write(&SDFile, header, sizeof(header) - 1U, &written) != FR_OK)
        file_open = false;
}

static void sdcard_write_event(const sdcard_driver_event_t* event)
{
    if (!event || !file_open)
        return;

    char row[SDCARD_ROW_BUF_SIZE];
    int len = 0;

    switch (event->type) {
        case SDCARD_DRIVER_EVENT_RANGING: {
            const sdcard_driver_ranging_event_t* r = &event->data.ranging;
            len = snprintf(row, sizeof(row), 
                "RANGING,%lu,%.4f,%u,%.3f,%.3f,%.3f,%.3f,%.1f,,,,,,,,,,,,,,,,\n",
                (unsigned long)event->timestamp_ms, r->distance_m, (unsigned)r->anchor_addr,
                r->anchor_x, r->anchor_y, r->anchor_z, r->quality, r->rssi_dbm);
            break;
        }
        case SDCARD_DRIVER_EVENT_IMU: {
            const sdcard_driver_imu_event_t* i = &event->data.imu;
            len = snprintf(row, sizeof(row), 
                "IMU,%lu,,,,,,,,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,,,,,,,,,\n",
                (unsigned long)event->timestamp_ms, i->accel_x, i->accel_y, i->accel_z,
                i->gyro_x, i->gyro_y, i->gyro_z, i->temp_c);
            break;
        }
        case SDCARD_DRIVER_EVENT_POSITION: {
            const sdcard_driver_position_event_t* p = &event->data.position;
            len = snprintf(row, sizeof(row), 
                "POSITION,%lu,,,,,,,,,,,,,,,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%d,%d\n",
                (unsigned long)event->timestamp_ms, p->x, p->y, p->z, p->vx, p->vy, p->vz, p->confidence, (int)p->valid, p->imu_enable);
            break;
        }
        default:
            return;
    }

    if (len <= 0 || (size_t)len >= sizeof(row))
        return;

    UINT written;
    if (f_write(&SDFile, row, (UINT)len, &written) != FR_OK)
        file_open = false;
}

static void sdcard_driver_task(void* argument)
{
    (void)argument;
    sdcard_driver_event_t event;

    for (;;) {
        if (!sdcard_mounted || !file_open) {
            if (!sdcard_mount_and_open()) {
                vTaskDelay(pdMS_TO_TICKS(SDCARD_REMOUNT_DELAY_MS));
                continue;
            }
        }

        if (xQueueReceive(sdcard_queue, &event, pdMS_TO_TICKS(SDCARD_POLL_MS)) == pdTRUE) {
            sdcard_write_event(&event);
            while (xQueueReceive(sdcard_queue, &event, 0) == pdTRUE)
                sdcard_write_event(&event);
        }

        uint32_t now = xTaskGetTickCount();
        if ((now - last_sync_tick) >= pdMS_TO_TICKS(SDCARD_SYNC_INTERVAL_MS)) {
            if (file_open && f_sync(&SDFile) == FR_OK)
                last_sync_tick = now;
            else if (file_open)
                file_open = false;
        }
    }
}
#endif
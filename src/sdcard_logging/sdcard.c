/*---------------------------------------------------------------------------
 * @file    sdcard.c
 * @brief   SD card logging module - writes UWB, IMU, and position data to CSV
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Includes
 *---------------------------------------------------------------------------*/
#include "sdcard.h"
#include "common.h"
#include "error_handler.h"
#include "fatfs.h"
#include "feature_config.h"
#include "FreeRTOS.h"
#include "module.h"
#include "queue.h"
#include "state_machine.h"
#include "task.h"
#include "uart_manager.h"

/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/
#define SDCARD_QUEUE_SIZE        (32U)
#define SDCARD_LOG_FILENAME      "DATA.CSV"
#define SDCARD_RETRY_DELAY_MS    (3000U)
#define SDCARD_SYNC_INTERVAL     (10U)   // f_sync every N rows written

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/
typedef enum
{
    STATE_IDLE = 0,
    STATE_MOUNTING,
    STATE_OPENING_FILE,
    STATE_LOGGING,
    STATE_ERROR,
    NUM_SDCARD_STATES
} sdcard_state_e;

/*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/
STATIC void sdcard_init(void);
STATIC void sdcard_process_10Hz(void);
STATIC void sdcard_state_machine_sample_inputs(void);

STATIC uint16_t sdcard_transition_logic(uint16_t current_state, uint32_t state_timer);

STATIC void sdcard_state_mounting_on_entry(uint16_t prev_state);
STATIC void sdcard_state_opening_file_on_entry(uint16_t prev_state);
STATIC void sdcard_state_logging_process(void);
STATIC void sdcard_state_logging_on_exit(uint16_t next_state);
STATIC void sdcard_state_error_on_entry(uint16_t prev_state);

STATIC void sdcard_write_csv_header(void);
STATIC void sdcard_write_event(const sdcard_log_event_t* event);

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
#if HWREV == 1
const module_S sdcard_module = {
    .module_name          = "sdcard",
    .module_init          = sdcard_init,
    .module_create_task   = NULL,
    .module_process_1Hz   = NULL,
    .module_process_10Hz  = sdcard_process_10Hz,
    .module_process_100Hz = NULL,
    .module_process_1kHz  = NULL,
};
#elif HWREV == 0
const module_S sdcard_module = {
    .module_name          = "sdcard",
    .module_init          = NULL,
    .module_create_task   = NULL,
    .module_process_1Hz   = NULL,
    .module_process_10Hz  = NULL,
    .module_process_100Hz = NULL,
    .module_process_1kHz  = NULL,
};
#endif

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/
STATIC const state_s sdcard_states[NUM_SDCARD_STATES] = {
    [STATE_IDLE]         = {.process = NULL,                        .onEntry = NULL,                            .onExit = NULL},
    [STATE_MOUNTING]     = {.process = NULL,                        .onEntry = sdcard_state_mounting_on_entry,  .onExit = NULL},
    [STATE_OPENING_FILE] = {.process = NULL,                        .onEntry = sdcard_state_opening_file_on_entry, .onExit = NULL},
    [STATE_LOGGING]      = {.process = sdcard_state_logging_process,.onEntry = NULL,                            .onExit = sdcard_state_logging_on_exit},
    [STATE_ERROR]        = {.process = NULL,                        .onEntry = sdcard_state_error_on_entry,     .onExit = NULL},
};

STATIC state_machine_s sdcard_state_machine = {
    .prev_state     = STATE_IDLE,
    .curr_state     = STATE_IDLE,
    .next_state     = STATE_IDLE,
    .timer          = 0,
    .transitionLogic = sdcard_transition_logic,
    .states         = sdcard_states,
};

STATIC QueueHandle_t sdcard_queue    = NULL;
STATIC bool          mount_ok        = false;
STATIC bool          file_open       = false;
STATIC uint32_t      rows_since_sync = 0;

/*---------------------------------------------------------------------------
 * State Transition Logic
 *---------------------------------------------------------------------------*/
STATIC uint16_t sdcard_transition_logic(uint16_t current_state, uint32_t state_timer)
{
    uint16_t next_state = current_state;

    switch (current_state)
    {
        case STATE_IDLE:
            // Short startup delay then attempt mount
            if (state_timer >= MS_TO_10HZ_TICKS(500U))
            {
                next_state = STATE_MOUNTING;
            }
            break;

        case STATE_MOUNTING:
            if (mount_ok)
            {
                next_state = STATE_OPENING_FILE;
            }
            else if (state_timer >= MS_TO_10HZ_TICKS(2000U))
            {
                // Mount attempt timed out
                next_state = STATE_ERROR;
            }
            break;

        case STATE_OPENING_FILE:
            if (file_open)
            {
                next_state = STATE_LOGGING;
            }
            else if (state_timer >= MS_TO_10HZ_TICKS(2000U))
            {
                next_state = STATE_ERROR;
            }
            break;

        case STATE_LOGGING:
            if (!file_open)
            {
                // Lost file handle - recover
                next_state = STATE_ERROR;
            }
            break;

        case STATE_ERROR:
            if (state_timer >= MS_TO_10HZ_TICKS(SDCARD_RETRY_DELAY_MS))
            {
                next_state = STATE_MOUNTING;
            }
            break;

        default:
            break;
    }

    return next_state;
}

/*---------------------------------------------------------------------------
 * State Handlers
 *---------------------------------------------------------------------------*/
STATIC void sdcard_state_mounting_on_entry(uint16_t prev_state)
{
    (void)prev_state;
    mount_ok    = false;
    file_open   = false;

    FRESULT result = f_mount(&SDFatFS, "", 1 /* force mount now */);
    if (result == FR_OK)
    {
        mount_ok = true;
        uart_manager_print("[SDCard] Mounted OK\r\n");
    }
    else
    {
        uart_manager_print("[SDCard] Mount failed (%d)\r\n", (int)result);
    }
}

STATIC void sdcard_state_opening_file_on_entry(uint16_t prev_state)
{
    (void)prev_state;
    file_open = false;

    // FA_OPEN_APPEND | FA_WRITE: creates if absent, appends if present
    FRESULT result = f_open(&SDFile, SDCARD_LOG_FILENAME, FA_OPEN_APPEND | FA_WRITE);
    if (result != FR_OK)
    {
        uart_manager_print("[SDCard] f_open failed (%d)\r\n", (int)result);
        return;
    }

    // Write CSV header only when the file is brand-new (size == 0)
    if (f_size(&SDFile) == 0)
    {
        sdcard_write_csv_header();
    }

    file_open       = true;
    rows_since_sync = 0;
    uart_manager_print("[SDCard] Log file open: %s\r\n", SDCARD_LOG_FILENAME);
}

STATIC void sdcard_state_logging_process(void)
{
    sdcard_log_event_t event;

    // Drain up to all pending events per tick to avoid queue backup
    while (xQueueReceive(sdcard_queue, &event, 0) == pdTRUE)
    {
        sdcard_write_event(&event);
        rows_since_sync++;

        if (rows_since_sync >= SDCARD_SYNC_INTERVAL)
        {
            f_sync(&SDFile);
            rows_since_sync = 0;
        }
    }
}

STATIC void sdcard_state_logging_on_exit(uint16_t next_state)
{
    (void)next_state;
    if (file_open)
    {
        f_sync(&SDFile);
        f_close(&SDFile);
        file_open = false;
    }
}

STATIC void sdcard_state_error_on_entry(uint16_t prev_state)
{
    (void)prev_state;
    if (file_open)
    {
        f_close(&SDFile);
        file_open = false;
    }
    f_mount(NULL, "", 0); // Unmount to reset FatFS state
    mount_ok = false;
    uart_manager_print("[SDCard] Error state - retrying in %ums\r\n", SDCARD_RETRY_DELAY_MS);
}

/*---------------------------------------------------------------------------
 * CSV Helpers
 *---------------------------------------------------------------------------*/
STATIC void sdcard_write_csv_header(void)
{
    static const char header[] =
        "type,timestamp_ms,"
        "dist_m,anchor_addr,anchor_x,anchor_y,anchor_z,quality,rssi_dbm,"
        "accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp_c,"
        "pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,confidence,pos_valid\n";

    UINT written = 0;
    f_write(&SDFile, header, STRLEN_LITERAL(header), &written);
}

STATIC void sdcard_write_event(const sdcard_log_event_t* event)
{
    char row[256];
    int  len = 0;

    switch (event->type)
    {
        case SDCARD_EVENT_RANGING:
        {
            const sensor_ranging_data_t* r = &event->data.ranging;
            len = snprintf(row, sizeof(row),
                "RANGING,%lu,"
                "%.4f,%u,%.3f,%.3f,%.3f,%.3f,%.1f,"
                ",,,,,,,,"
                ",,,,,,\n",
                (unsigned long)event->timestamp_ms,
                (double)r->distance_m,
                (unsigned)r->anchor_addr,
                (double)r->anchor_position.x,
                (double)r->anchor_position.y,
                (double)r->anchor_position.z,
                (double)r->quality,
                (double)r->rssi_dbm);
            break;
        }

        case SDCARD_EVENT_IMU:
        {
            const sensor_imu_data_t* imu = &event->data.imu;
            len = snprintf(row, sizeof(row),
                "IMU,%lu,"
                ",,,,,,,"
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,"
                ",,,,,,\n",
                (unsigned long)event->timestamp_ms,
                (double)imu->accel_x, (double)imu->accel_y, (double)imu->accel_z,
                (double)imu->gyro_x,  (double)imu->gyro_y,  (double)imu->gyro_z,
                (double)imu->temp_c);
            break;
        }

        case SDCARD_EVENT_POSITION:
        {
            const sensor_fusion_position_t* p = &event->data.position;
            len = snprintf(row, sizeof(row),
                "POSITION,%lu,"
                ",,,,,,,"
                ",,,,,,,,"
                "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%d\n",
                (unsigned long)event->timestamp_ms,
                (double)p->x,  (double)p->y,  (double)p->z,
                (double)p->vx, (double)p->vy, (double)p->vz,
                (double)p->confidence,
                (int)p->valid);
            break;
        }

        default:
            return;
    }

    if (len > 0 && len < (int)sizeof(row))
    {
        UINT written = 0;
        FRESULT result = f_write(&SDFile, row, (UINT)len, &written);
        if (result != FR_OK || written != (UINT)len)
        {
            file_open = false; // Trigger error recovery
        }
    }
}

/*---------------------------------------------------------------------------
 * Module Function Implementations
 *---------------------------------------------------------------------------*/
STATIC void sdcard_init(void)
{
    sdcard_queue = xQueueCreate(SDCARD_QUEUE_SIZE, sizeof(sdcard_log_event_t));
    if (sdcard_queue == NULL)
    {
        error_handler_fatal("sdcard", "Failed to create log queue");
    }
}

STATIC void sdcard_process_10Hz(void)
{
    sdcard_state_machine_sample_inputs();
    state_machine_periodic(&sdcard_state_machine);
}

STATIC void sdcard_state_machine_sample_inputs(void)
{
    // Inputs (mount_ok, file_open) are updated directly by state handlers
}

/*---------------------------------------------------------------------------
 * Public Function Implementations
 *---------------------------------------------------------------------------*/
sdcard_push_status_e sdcard_push_event(const sdcard_log_event_t* event)
{
    if (event == NULL)
    {
        return SDCARD_PUSH_ERROR_NULL_PTR;
    }

    if (sdcard_queue == NULL)
    {
        return SDCARD_PUSH_ERROR_NOT_INITIALIZED;
    }

    if (event->type >= NUM_SDCARD_EVENT_TYPES)
    {
        return SDCARD_PUSH_ERROR_INVALID_TYPE;
    }

    if (xQueueSend(sdcard_queue, event, 0) != pdTRUE)
    {
        return SDCARD_PUSH_ERROR_QUEUE_FULL;
    }

    return SDCARD_PUSH_SUCCESS;
}

bool sdcard_is_logging(void)
{
    return (sdcard_state_machine.curr_state == STATE_LOGGING) && file_open;
}


/*---------------------------------------------------------------------------
 * Defines
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Typedefs
 *---------------------------------------------------------------------------*/

 /*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

 /*---------------------------------------------------------------------------
 * Private Function Prototypes
 *---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------
 * Module Functions
 *---------------------------------------------------------------------------*/
STATIC void sdcard_init(void);
STATIC void sdcard_process_10Hz(void);

#if HWREV == 1
const module_S sdcard_module= {
    .module_name         = "sdcard",
    .module_init         = sdcard_init,
    .module_create_task  = NULL,
    .module_process_1Hz   = NULL,
    .module_process_10Hz  = sdcard_process_10Hz,
    .module_process_100Hz = NULL,
    .module_process_1kHz  = NULL,
};
#elif HWREV == 0
const module_S sdcard_module= {
    .module_name         = "sdcard",
    .module_init         = NULL,
    .module_create_task  = NULL,
    .module_process_1Hz   = NULL,
    .module_process_10Hz  = NULL,
    .module_process_100Hz = NULL,
    .module_process_1kHz  = NULL,
};
#endif

/*---------------------------------------------------------------------------
 * Private Variables
 *---------------------------------------------------------------------------*/

 
/*---------------------------------------------------------------------------
 * Private Function Implementations
 *---------------------------------------------------------------------------*/

STATIC void sdcard_init(void)
{

}

 STATIC void sdcard_process_10Hz(void)
{
    state_machine_periodic(&sdcard_state_machine);
}

FRESULT result; /* FatFs function common result code */
uint32_t byteswritten, bytesread; /* File write/read counts */
static FRESULT mount_status = FR_NO_FILESYSTEM;

static uint8_t mount_sd_card();

void sd_save_sensors(filtered_sensors_t *sensor_data) {
    char csv_line[360];
    char header[] = "filter_lat,filter_long,filter_alt,filter_vN,filter_vE,filter_vD,filter_q0,filter_q1,filter_q2,filter_q3,roll,pitch,yaw,airspeed,fc_current,servo_current,time\n";

    snprintf(csv_line, sizeof(csv_line),
             "%.15f,%.15f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%d,%d,%.2f\n",
             sensor_data->filter_lat,
             sensor_data->filter_long,
             sensor_data->filter_alt,
             sensor_data->filter_vN,
             sensor_data->filter_vE,
             sensor_data->filter_vD,
             sensor_data->filter_q0,
             sensor_data->filter_q1,
             sensor_data->filter_q2,
             sensor_data->filter_q3,
             sensor_data->roll,
             sensor_data->pitch,
             sensor_data->yaw,
             sensor_data->airspeed,
			 sensor_data->fix_type,
			 sensor_data->rtk_status,
             sensor_data->time);

    result = mount_sd_card();
    if (result != FR_OK) return;

    result = f_open(&SDFile, "FDATA.CSV", FA_OPEN_APPEND | FA_WRITE);
    if (result != FR_OK) return;

    if (f_size(&SDFile) == 0) {
        f_write(&SDFile, header, strlen(header), (void *)&byteswritten);
    }

    f_write(&SDFile, csv_line, strlen(csv_line), (void *)&byteswritten);
    f_close(&SDFile);
}

static uint8_t mount_sd_card(){
	if (mount_status != FR_OK){
		mount_status = f_mount(&SDFatFS, "", 0);
	}
	return mount_status;
}
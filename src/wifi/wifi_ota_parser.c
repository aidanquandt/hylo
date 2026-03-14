#include "wifi_ota_parser.h"

// --------------------------
// OTA Settings
// --------------------------
typedef struct {
    float anchor_0_pos;
    float anchor_1_pos;
    float anchor_2_pos;
    float anchor_3_pos;
    int fusion_on;
    int imu_off;
} ota_settings_t;

static ota_settings_t ota_settings = {
    .anchor_0_pos = 0.0f,
    .anchor_1_pos = 0.0f,
    .anchor_2_pos = 0.0f,
    .anchor_3_pos = 0.0f,
    .fusion_on = 0,
    .imu_off = 0
};

// Getters
float get_anchor_0_pos(void) { return ota_settings.anchor_0_pos; }
float get_anchor_1_pos(void) { return ota_settings.anchor_1_pos; }
float get_anchor_2_pos(void) { return ota_settings.anchor_2_pos; }
float get_anchor_3_pos(void) { return ota_settings.anchor_3_pos; }
int get_imu_off(void) { return ota_settings.imu_off; }
int get_fusion_on(void) { return ota_settings.fusion_on; }

// Setters
void set_anchor_0_pos(float val) { ota_settings.anchor_0_pos = val; }
void set_anchor_1_pos(float val) { ota_settings.anchor_1_pos = val; }
void set_anchor_2_pos(float val) { ota_settings.anchor_2_pos = val; }
void set_anchor_3_pos(float val) { ota_settings.anchor_3_pos = val; }
void set_imu_off(int val) { ota_settings.imu_off = val; }
void set_fusion_on(int val) { ota_settings.fusion_on = val; }

static void process_ota_line(char* line_buf);

// --------------------------
// OTA Parsing
// --------------------------
#define OTA_LINE_BUF_SIZE 128
static char line_buf[OTA_LINE_BUF_SIZE];
static size_t line_len = 0;

void ota_parser_init(void)
{
    line_len = 0;
}

// Feed each received byte (from WiFi RX)
void ota_parser_process_byte(uint8_t c)
{
    if (c == '\n' || c == '\r') {
        if (line_len > 0) {
            line_buf[line_len] = '\0';
            process_ota_line(line_buf);
            line_len = 0;
        }
    } else if (line_len < OTA_LINE_BUF_SIZE - 1) {
        line_buf[line_len++] = c;
    }
}

static void process_ota_line(char* line_buf)
{
    char *type = strtok(line_buf, " ");
    char *cmd  = strtok(NULL, " ");
    char *arg1 = strtok(NULL, " ");
    char *arg2 = strtok(NULL, " ");

    if (!type || strcmp(type, "OTA") != 0) return; // ignore non-OTA
    
    if (!cmd) {
        return;
    }

    if (strcmp(cmd, "set-imu-off") == 0 && arg1) {
        set_imu_off(atoi(arg1));
    }
    else if (strcmp(cmd, "set-fusion") == 0 && arg1) {
        set_fusion_on(atoi(arg1));
    }
    else if (strcmp(cmd, "set-anchor") == 0 && arg1 && arg2) {
        int anchor_id = atoi(arg1);
        float pos = atof(arg2);
        switch (anchor_id) {
            case 0: set_anchor_0_pos(pos); break;
            case 1: set_anchor_1_pos(pos); break;
            case 2: set_anchor_2_pos(pos); break;
            case 3: set_anchor_3_pos(pos); break;
            default: break;
        }
    }
}
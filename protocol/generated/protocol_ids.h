/* Auto-generated from uart_protocol.proto by protocol/codegen_protocol.py */
#ifndef PROTOCOL_IDS_H
#define PROTOCOL_IDS_H
#include <stdint.h>

typedef enum {
  MSG_ID_PingRequest = 0,
  MSG_ID_PingResponse = 1,
  MSG_ID_SetAddressRequest = 2,
  MSG_ID_SetAddressResponse = 3,
  MSG_ID_GetConfigRequest = 4,
  MSG_ID_GetConfigResponse = 5,
  MSG_ID_LogLine = 6,
  MSG_ID_COUNT
} protocol_msg_id_t;

#endif /* PROTOCOL_IDS_H */

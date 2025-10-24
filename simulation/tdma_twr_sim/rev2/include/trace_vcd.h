#ifndef TRACE_VCD_H
#define TRACE_VCD_H

#include "node.h"

void vcd_start(const char* path);
void vcd_dump_step(void);
void vcd_stop(void);

// PHY trace hooks
void trace_phy_tx(const Message* msg);
void trace_phy_rx(const Message* msg, int receiver_id);

#endif /* TRACE_VCD_H */

#ifndef PHY_H
#define PHY_H

#include "node.h"

void phy_unicast(const Message* msg);
void phy_broadcast(int from_id, MsgType type, int ttl);
void phy_advance_message_queues(void);

#endif /* PHY_H */

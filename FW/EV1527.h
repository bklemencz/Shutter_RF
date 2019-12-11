#ifndef __EV1527_H
#define __EV1527_H

#include "stm8s.h"

void EV1527_Receive_Parse(bool EV1527_edge_type, uint16_t EV1527_edge_time_10us);
void EV1527_Receive_Check(void);

#endif
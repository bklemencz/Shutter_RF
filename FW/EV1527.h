#ifndef __EV1527_H
#define __EV1527_H

#include "stm8s.h"

void EV1527_Receive_Parse(void);
void EV1527_Receive_Check(void);
void EV1527_Transmit(uint32_t Code, uint8_t Length, uint8_t Repeat);

#endif
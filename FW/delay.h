#ifndef __DELAY_H
#define __DELAY_H

#define F_CPU 16000000UL

static void _delay_cycl( uint16_t ticks );
void _delay_us( uint16_t us );
void _delay_ms( uint16_t ms );

#endif
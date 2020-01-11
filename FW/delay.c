#include "stm8s.h"
#include "delay.h"

static void _delay_cycl( uint16_t ticks )
{
  #define T_COUNT(x) (( F_CPU * x / 1000000UL )-3)/3)
	// ldw X, __ticks ; insert automaticaly
	_asm("nop\n $N:\n decw X\n jrne $L\n nop\n ", ticks);
}

void _delay_us( uint16_t us )
{
	_delay_cycl( (uint16_t)( T_COUNT(us) );
}

void _delay_ms( uint16_t ms )
{
	while ( ms-- )
	{
		_delay_us( 1000 );
	}
}

#include "stm8s.h"

/*
PINOUT
PD1 - External Switch
PD2 - Relay CH1
PD3 - Relay CH2
PD5 - RS485_TX
PD6 - RS485_RX
PA1 - RF_RX
PA2 - RF_TX
PA3 - TEMPERATURE
PB4 - I2C_SCL
PB5 - I2C_SDA
PC3 - INPUT_VDD3.3V
PC5 - RS485_DE
PC7 - POSITION
*/


const uint8_t EV1527_PulseTime_Max_10us = 58;
const uint8_t EV1527_PulseTime_Min_10us = 48;
const uint16_t EV1527_Pulse_Timeout_10us = EV1527_PulseTime_Max_10us * 35;
const uint16_t EV1527_Packet_Repeat_Timeout_10us = EV1527_PulseTime_Max_10us * 75;
const uint8_t RELAY_DEADTIME_1ms = 10;			// relay contact release time 10ms

uint32_t EV1527_rec_data;
volatile uint16_t EV1527_last_edge;
uint8_t Relay_Change_Delay_1ms;
uint16_t Timer_ms_to_s;
volatile uint8_t Timer_10us_to_ms;


bool EV1527_sync_OK;
bool EV1527_bit_OK;
bool EV1527_packet_OK;

// TIM4 10us interrupts, base for RF decoding
void InitTIM4()
{
  TIM4->PSCR=0;				//1 frequency division, timer clock equals system clock = 16m

  TIM4->ARR=0XA0;			//10us reload value 0XA0

  TIM4->CNTR=0;				//It is necessary to clear down the counter
  TIM4->IER |= 1<<0;		//Enable tim4 update interrupt
  

  TIM4->SR1  |= 1<<0;		//Clear tim4 update interrupt flag

  TIM4->CR1 |= 1<<7;		//Allow reassembly to enable timer
  TIM4->CR1 |= 1<<0;		//Enabling tim4 counter
}

//checks and sets the relay outputs with deadtime at changes
void Relay_Channel_Set(uint8_t Channel)
{
	if (Channel == 0) 
	{
		GPIO_WriteLow(GPIOD, GPIO_PIN_2);
		GPIO_WriteLow(GPIOD,GPIO_PIN_3);
		Relay_Change_Delay_1ms = RELAY_DEADTIME_1ms;
	}
	else if ((Channel ==1) && (Relay_Change_Delay_1ms == 0))
	{
		GPIO_WriteHigh(GPIOD, GPIO_PIN_2);
		GPIO_WriteLow(GPIOD,GPIO_PIN_3);
		Relay_Change_Delay_1ms = RELAY_DEADTIME_1ms;
	}
	else if ((Channel ==2) && (Relay_Change_Delay_1ms == 0))
	{
		GPIO_WriteLow(GPIOD, GPIO_PIN_2);
		GPIO_WriteHigh(GPIOD,GPIO_PIN_3);
		Relay_Change_Delay_1ms = RELAY_DEADTIME_1ms;
	}
}

void Timer_handle(void)
{
	
	if (Timer_10us_to_ms == 0)
	{
		Timer_10us_to_ms = 100;
		Timer_ms_to_s --;
		//1 ms tasks
		if (Relay_Change_Delay_1ms) Relay_Change_Delay_1ms--;
	}
	if (Timer_ms_to_s == 0)
	{
		Timer_ms_to_s = 1000;
		//1s tasks
	}
}

main()
{

	while (1)
	{
		Timer_handle();
	}
	//GPIO_Init(GPIOA , GPIO_PIN_0, GPIO_MODE_IN_PU_IT);

}
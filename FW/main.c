
#include "stm8s.h"
#include "EV1527.h"
#include "eeprom.h"

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

FEATURES:
SHUTTER
Timeout to deenergize relays
External SW short press: start - stop - reverse - stop
RF - 	24bit base code: start - stop - reverse - stop
		32bit extended code : move to position (10 position)
		5 sepearte codes for group controls
External SW debouncing
Position time learning : shutter starts at about middle position - external sw (long press 5s) - short press at top end stop starts go down, short press at bottom endstop.

SERIAL PROTOCOL ( 8 bytes, if byte not used send 0):
Byte
1 - Device Function (6 Shutter)
2 - Group ID (255 Broadcast)
3 - Device ID (255 Broadcast)
4 - Command ID
	1 - Start - Stop - Reverse
	2 - Full UP
	3 - Full Down
	4 - Go To Position followed by 1 byte position
	5 - ACK request for auto discovery
	64 - Store remote code 1 followed by 4 bytes, High byte first
	65 - Store remote code 2 followed by 4 bytes, High byte first
	66 - Store remote code 3 followed by 4 bytes, High byte first
	67 - Store remote code 4 followed by 4 bytes, High byte first
	68 - Store remote code 5 followed by 4 bytes, High byte first
	90 - Store Shutter Travel time
	128 - Store Group ID followed by 1 byte Device ID

Reply: 
Byte
1 - ACK
	0 - OK 
	1 - NOK General
	2 - Unknown Command
*/

#define F_CPU 16000000UL

const uint8_t RELAY_DEADTIME_1ms = 10;			// relay contact release time 10ms
const uint8_t RELAY_ENDSTOP_TIMEOUT_S = 120;	// maximum relay energising time in seconds
const uint8_t EXT_SW_DEBOUNCE_MS = 10;			//external switch debounce time in ms
const uint8_t EXT_SW_LONG_PRESS_TIME_S = 5;
const uint8_t SERIAL_FUNCTION_ID = 6;

uint32_t EV1527_Rec_Code;
uint8_t EV1527_Rec_Data;
uint8_t EV1527_Repeat_Count;
bool EV1527_Valid_Code;
bool EV1527_Valid_Data;
uint32_t EV1527_Stored_Codes[5];				//storing a maximum of 5 codes


uint8_t Relay_Change_Delay_1ms;
uint8_t Relay_On_Time_s;
bool Relay_Changed;
uint8_t Relay_State,Relay_Prev_State;
uint16_t Timer_ms_to_s;
volatile uint8_t Timer_10us_to_ms;

uint8_t Shutter_Position;
uint8_t Shutter_Target_Position;
uint8_t Shutter_Time_To_Target_s;
uint8_t Shutter_Full_Motion_Time_s;
bool Shutter_Learning;

volatile bool Ext_SW_Pressed;
volatile bool Ext_SW_Released;
volatile uint8_t Ext_SW_Debounce_Timeout_ms;
bool Ext_SW_Short_Pressed;
bool Ext_SW_Long_Pressed;
uint8_t Ext_SW_Pressed_Time_s;

volatile uint8_t Serial_Rx_Buffer[8];
volatile uint8_t Serial_Rx_Counter;
uint8_t Serial_GroupID;
uint8_t Serial_DevID;




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

void Save_Codes(void)
{
	uint8_t i;
	EEPROM_Config();
	for(i=0;i<5;i++) 
	{
		if(EV1527_Stored_Codes[i] != 0) 
		{
			EEPROM_Program4Byte((uint16_t)((i*4)+4),EV1527_Stored_Codes[i]);
			_delay_ms(1);
		}
	}
}

void Save_Position(void)
{
	EEPROM_Config();
	EEPROM_ProgramByte(0,Shutter_Position);
	_delay_ms(1);
}

void Save_Learned_Time(void)
{
	EEPROM_Config();
	EEPROM_ProgramByte(1,Shutter_Full_Motion_Time_s);
	_delay_ms(1);
}

void Save_IDs(void)
{
	EEPROM_Config();
	EEPROM_ProgramByte(2,Serial_GroupID);
	_delay_ms(1);
	EEPROM_ProgramByte(3,Serial_DevID);
	_delay_ms(1);
}

void Load_Saved_Stuff(void)
{
	uint8_t i;
	Shutter_Position = EEPROM_ReadByte(0);
	Shutter_Full_Motion_Time_s = EEPROM_ReadByte(1);
	if(Shutter_Full_Motion_Time_s == 0) Shutter_Full_Motion_Time_s = RELAY_ENDSTOP_TIMEOUT_S;
	Serial_GroupID = EEPROM_ReadByte(2);
	Serial_DevID = EEPROM_ReadByte(3);
	for ( i=0; i<5; i++)
	{
		EV1527_Stored_Codes[i] = EEPROM_Read4Byte((i*4)+4);
	}
}

void Serial_Reply(uint8_t Value)
{
	_delay_ms(1);
	GPIO_WriteHigh(GPIOC,GPIO_PIN_5);
	_delay_ms(1);
	UART1_SendData8(Value);
	_delay_ms(1);
	GPIO_WriteLow(GPIOC,GPIO_PIN_5);
	_delay_ms(1);
	UART1_ITConfig(UART1_IT_RXNE_OR,ENABLE);
}

void IO_Init (void)
{
	CLK->CKDIVR &= (uint8_t)(~CLK_CKDIVR_HSIDIV);
	CLK->CKDIVR |= (uint8_t)1;								// Internal clock to 16Mhz
	FLASH_DeInit();
	GPIO_DeInit(GPIOA);
	GPIO_Init(GPIOA,GPIO_PIN_1,GPIO_MODE_IN_PU_IT);			//RF_RX
	GPIO_Init(GPIOA,GPIO_PIN_2,GPIO_MODE_OUT_PP_LOW_FAST);	//RF_TX
	GPIO_DeInit(GPIOC);
	GPIO_Init(GPIOC,GPIO_PIN_3,GPIO_MODE_IN_FL_NO_IT);		//VDD
	GPIO_Init(GPIOC,GPIO_PIN_5,GPIO_MODE_OUT_PP_LOW_FAST);	//RS485_DE
	GPIO_WriteLow(GPIOC,GPIO_PIN_5);						// Serial RX enable
	GPIO_Init(GPIOC,GPIO_PIN_7,GPIO_MODE_IN_PU_IT);			//POSITION
	GPIO_DeInit(GPIOD);
	GPIO_Init(GPIOD,GPIO_PIN_1,GPIO_MODE_IN_PU_IT);			//EXT_SW
	GPIO_Init(GPIOD,GPIO_PIN_2,GPIO_MODE_OUT_PP_LOW_FAST);	//Relay1
	GPIO_Init(GPIOD,GPIO_PIN_3,GPIO_MODE_OUT_PP_LOW_FAST);	//Relay2
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA,EXTI_SENSITIVITY_RISE_FALL);
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD,EXTI_SENSITIVITY_RISE_FALL);

	UART1_DeInit();
	
	UART1_Init((uint32_t)19200,UART1_WORDLENGTH_8D,UART1_STOPBITS_1,UART1_PARITY_NO,UART1_SYNCMODE_CLOCK_DISABLE,UART1_MODE_TXRX_ENABLE);
	UART1_ITConfig(UART1_IT_RXNE_OR,ENABLE);
	
	enableInterrupts();
}

// TIM4 10us interrupts, base system tick
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

void Shutter_Calc_Time_To_Target(uint8_t Pos_Target)
{
	uint8_t Shutter_One_Position_Time_s;
	Shutter_One_Position_Time_s = Shutter_Full_Motion_Time_s / 10;
	if (Pos_Target == Shutter_Position) Shutter_Time_To_Target_s = 0;
	else if (Pos_Target == 10)		//full up, go until timeout to deoffset
	{
		Shutter_Time_To_Target_s = 255;
		Relay_State = 1;
		Relay_Changed = TRUE;
	}
	else if (Pos_Target == 0)		//full down, go until timeout to deoffset
	{
		Shutter_Time_To_Target_s = 255;
		Relay_State = 2;
		Relay_Changed = TRUE;
	}
	else if (Pos_Target > Shutter_Position) 
	{
		Shutter_Time_To_Target_s = (Pos_Target - Shutter_Position) * Shutter_One_Position_Time_s;
		Relay_State = 1;
		Relay_Changed = TRUE;
	}
	else if (Pos_Target < Shutter_Position)
	{
		Shutter_Time_To_Target_s = (Shutter_Position - Pos_Target) * Shutter_One_Position_Time_s;
		Relay_State = 2;
		Relay_Changed = TRUE;
	}
}

//checks and sets the relay outputs with deadtime at changes
void Relay_Channel_Set(uint8_t Channel)
{
	if (Relay_Changed)
	{
		if (Channel == 0) 
		{
			GPIO_WriteLow(GPIOD, GPIO_PIN_2);
			GPIO_WriteLow(GPIOD,GPIO_PIN_3);
			Relay_Change_Delay_1ms = RELAY_DEADTIME_1ms;
			if ((Shutter_Time_To_Target_s!=0) && (Relay_On_Time_s < Shutter_Time_To_Target_s)) 			// if stopped before reaching target position, calculate new position
			{
				if (Relay_Prev_State == 1) 
				{
					Shutter_Position = Shutter_Position + (Relay_On_Time_s / (Shutter_Full_Motion_Time_s / 10));
					Shutter_Target_Position = Shutter_Position;
					Shutter_Time_To_Target_s = 0;
					Save_Position();

				}
				if (Relay_Prev_State == 2) 
				{
					Shutter_Position = Shutter_Position - (Relay_On_Time_s / (Shutter_Full_Motion_Time_s / 10));
					Shutter_Target_Position = Shutter_Position;
					Shutter_Time_To_Target_s = 0;
					Save_Position();
				}
			}
			Relay_On_Time_s = 0;
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
		Relay_Changed = FALSE;
	}
	// if reached target position stop
	if((Relay_State > 0) && (Relay_On_Time_s > Shutter_Time_To_Target_s))
	{
		Shutter_Position = Shutter_Target_Position;
		Shutter_Time_To_Target_s = 0;
		Relay_State = 0; Relay_Changed = TRUE;
		Relay_On_Time_s = 0;
		Save_Position();
	}
	//if relay reached end stop for a time, turn off relay
	if ((Relay_State > 0) && (Relay_On_Time_s > RELAY_ENDSTOP_TIMEOUT_S))
	{
		if (Relay_State == 1) Shutter_Position = 10;
		if (Relay_State == 2) Shutter_Position = 0;
		Shutter_Time_To_Target_s = 0;
		Relay_State = 0; Relay_Changed = TRUE;
		Relay_On_Time_s = 0;
		Save_Position();
	} 
}

void Shutter_RF_Task(void)
{
	uint8_t i = 0;
	if (EV1527_Valid_Code && EV1527_Repeat_Count == 0)
	{
		for (i = 0; i < 5; i++)
		{
			if (EV1527_Rec_Code == EV1527_Stored_Codes[i])
			{
				//Valid stored CODE received not repeat
				if (Relay_State == 0)
				{
					if (Relay_Prev_State == 1)
					{
						Relay_Prev_State = 2;
						Shutter_Calc_Time_To_Target(0);
					}
					else if (Relay_Prev_State == 2)
					{
						Relay_Prev_State = 1;
						Shutter_Calc_Time_To_Target(10);
					}
				}
				else if (Relay_State > 0)
				{
					Relay_State = 0;
					Relay_Changed = TRUE;
				}
			}
		}
	}
}

void Ext_SW_Task(void)
{
	if (!Shutter_Learning)
	{
		if (Ext_SW_Released && (Ext_SW_Pressed_Time_s < EXT_SW_LONG_PRESS_TIME_S))
		{
			Ext_SW_Pressed_Time_s = 0;
			Ext_SW_Released = FALSE;
			if (Relay_State == 0)
			{
				if (Relay_Prev_State == 1)
				{
					Relay_Prev_State = 2;
					Shutter_Calc_Time_To_Target(0);
				}
				else if (Relay_Prev_State == 2)
				{
					Relay_Prev_State = 1;
					Shutter_Calc_Time_To_Target(10);
				}
			}
			else if (Relay_State > 0)
			{
				Relay_State = 0;
				Relay_Changed = TRUE;
			}
		}
		else if (Ext_SW_Released)
		{
			Ext_SW_Released = FALSE;
			Ext_SW_Pressed_Time_s = 0;
		}
		else if (Ext_SW_Pressed && (Ext_SW_Pressed_Time_s >= EXT_SW_LONG_PRESS_TIME_S))
		{
			Shutter_Learning = TRUE;
		}
	} else
	{
		if (Ext_SW_Released)
		{
			Ext_SW_Released = FALSE;
			Relay_Channel_Set(1);
			_delay_ms(3000);
			Relay_Channel_Set(0);
			_delay_ms(3000);
			Relay_Channel_Set(1);
			while (Ext_SW_Pressed);
			_delay_ms(100);
			Relay_Channel_Set(0);
			while (Ext_SW_Released);
			_delay_ms(100);
			Shutter_Full_Motion_Time_s = 0;
			Relay_Channel_Set(2);
			while(Ext_SW_Pressed);
			Shutter_Learning = FALSE;
			Relay_Channel_Set(0);
			while (Ext_SW_Released);
			_delay_ms(100);
			Shutter_Position = 0;
			Shutter_Target_Position = 5;
		}
	}
	
}
void Timer_Task(void)
{
	
	if (Timer_10us_to_ms == 0)
	{
		Timer_10us_to_ms = 100;
		Timer_ms_to_s --;
		//1 ms tasks
		if (Relay_Change_Delay_1ms) Relay_Change_Delay_1ms--;
		if (Ext_SW_Debounce_Timeout_ms) Ext_SW_Debounce_Timeout_ms--;
	}
	if (Timer_ms_to_s == 0)
	{
		Timer_ms_to_s = 1000;
		if (Relay_State >0) Relay_On_Time_s++;
		if (Ext_SW_Pressed) Ext_SW_Pressed_Time_s++;
		if (Shutter_Learning) Shutter_Full_Motion_Time_s++;
		//1s tasks
	}
}

void Serial_Task(void)
{
	if (Serial_Rx_Counter == 8)
	{
		Serial_Rx_Counter = 0;
		if ((Serial_Rx_Buffer[0] == SERIAL_FUNCTION_ID) && (Serial_Rx_Buffer[1] == Serial_GroupID) && (Serial_Rx_Buffer[2]==Serial_DevID))
		{
			switch (Serial_Rx_Buffer[3])
			{
			case 1:
				if (Relay_State == 0)
				{
					if (Relay_Prev_State == 1)
					{
						Relay_State = 2;
						Relay_Prev_State = 2;
						Relay_Changed = TRUE;
					}
					else if (Relay_Prev_State == 2)
					{
						Relay_State = 1;
						Relay_Prev_State = 1;
						Relay_Changed = TRUE;
					}
				}
				else if (Relay_State > 0)
				{
					Relay_State = 0;
					Relay_Changed = TRUE;
				}
				Serial_Reply(0);
				break;
			case 2:
				Shutter_Calc_Time_To_Target(10);
				Serial_Reply(0);
				break;
			case 3:
				Shutter_Calc_Time_To_Target(0);
				Serial_Reply(0);
				break;
			case 4:
				Shutter_Calc_Time_To_Target(Serial_Rx_Buffer[4]);
				Serial_Reply(0);
				break;
			case 5:
				Serial_Reply(0);
				break;
			case 64:
				EV1527_Stored_Codes[0] = (Serial_Rx_Buffer[4] * 0x1000000) + (Serial_Rx_Buffer[5] *  0x10000 ) + (Serial_Rx_Buffer[6] * 0x100) + Serial_Rx_Buffer[7];
				Save_Codes();
				Serial_Reply(0);
				break;
			case 65:
				EV1527_Stored_Codes[1] = (Serial_Rx_Buffer[4] * 0x1000000) + (Serial_Rx_Buffer[5] *  0x10000 ) + (Serial_Rx_Buffer[6] * 0x100) + Serial_Rx_Buffer[7];
				Save_Codes();
				Serial_Reply(0);
				break;
			case 66:
				EV1527_Stored_Codes[2] = (Serial_Rx_Buffer[4] * 0x1000000) + (Serial_Rx_Buffer[5] *  0x10000 ) + (Serial_Rx_Buffer[6] * 0x100) + Serial_Rx_Buffer[7];
				Save_Codes();
				Serial_Reply(0);
				break;
			case 67:
				EV1527_Stored_Codes[3] = (Serial_Rx_Buffer[4] * 0x1000000) + (Serial_Rx_Buffer[5] *  0x10000 ) + (Serial_Rx_Buffer[6] * 0x100) + Serial_Rx_Buffer[7];
				Save_Codes();
				Serial_Reply(0);
				break;
			case 68:
				EV1527_Stored_Codes[4] = (Serial_Rx_Buffer[4] * 0x1000000) + (Serial_Rx_Buffer[5] *  0x10000 ) + (Serial_Rx_Buffer[6] * 0x100) + Serial_Rx_Buffer[7];
				Save_Codes();
				Serial_Reply(0);
				break;
			case 90:
				Shutter_Full_Motion_Time_s = Serial_Rx_Buffer[4];
				Save_Learned_Time();
				Serial_Reply(0);
				break;
			case 128:
				Serial_GroupID = Serial_Rx_Buffer[4];
				Serial_DevID = Serial_Rx_Buffer[5];
				Save_IDs();
				Serial_Reply(0);
				break;
			default:
				Serial_Reply(2);
				break;
			}
		}
	}
}

main()

{
	
	IO_Init();
	Load_Saved_Stuff();
	while (1)
	{
		Relay_Channel_Set(Relay_State);
		Timer_Task();
		EV1527_Receive_Check();
		Shutter_RF_Task(); 	// TODO change to calc targets
		Ext_SW_Task();		// TODO change to calc targets
		Serial_Task();
	}
	//GPIO_Init(GPIOA , GPIO_PIN_0, GPIO_MODE_IN_PU_IT);

}
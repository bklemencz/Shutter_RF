
#include "stm8s.h"
#include "EV1527.h"
#include "eeprom.h"
#include "delay.h"

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
Code learning : External switch 10s - Remote code long

SERIAL PROTOCOL ( 7 bytes, if byte not used send 0 7th always '\r'):

Byte
1 - Device Function ('S' Shutter)
2 - ID (0-100 ID, 101-254 Group ID, 255 Broadcast)
3 - Command/Setup/RF Store ('C'/'S'/'1...5')
3'C' - Command ID
4-	'S' - Start - Stop - Reverse
4-	'U' - Full UP
4-	'D' - Full Down
4-	'P' - Go To Position followed by 1 byte position
4-	'A' - ACK request for auto discovery

3'S' - Setup ID
4-	'T' - 90 - Store Shutter Travel time

3'1..5' - Store
3-	'1' - Store remote code 1 followed by 3 bytes
3-	'2' - Store remote code 2 followed by 3 bytes
3-	'3' - Store remote code 3 followed by 3 bytes
3-	'4' - Store remote code 4 followed by 3 bytes
3-	'5' - Store remote code 5 followed by 3 bytes
	

Reply: 
Byte
1 - ACK
	0 - OK 
	1 - NOK General
	2 - Unknown Command
*/

#define SERIAL_BAUD	57600

const uint8_t RELAY_DEADTIME_1ms = 10;			// relay contact release time 10ms
const uint8_t RELAY_ENDSTOP_TIMEOUT_S = 120;	// maximum relay energising time in seconds
const uint8_t EXT_SW_DEBOUNCE_MS = 100;			//external switch debounce time in ms
const uint8_t EXT_SW_LONG_PRESS_TIME_S = 5;
const uint8_t EXT_SW_LEARN_PRESS_TIME_S = 10;
const uint8_t SERIAL_FUNCTION_ID = 'S';
const uint8_t EV1527_Transmit_Repeat = 10;


uint32_t EV1527_Rec_Code;
uint32_t EV1527_Valid_Rec_Code;
uint8_t EV1527_Rec_Data;
uint8_t EV1527_Valid_Rec_Data;
uint8_t EV1527_Repeat_Count;
bool EV1527_Valid_Code;
bool EV1527_Valid_Data;
bool EV1527_Valid_Learn_Code;
volatile uint16_t EV1527_Hightime;
volatile uint16_t EV1527_LowTime;
volatile bool EV1527_BitReady;
uint32_t EV1527_Stored_Codes[5];				//storing a maximum of 5 codes
uint8_t EV1527_Last_Code_Position;
uint32_t EV1527_Transmit_Data;

extern volatile uint16_t Timer_last_edge_10us;
uint16_t Timer_ms_to_s;
volatile uint8_t Timer_10us_to_ms;

uint8_t Relay_Change_Delay_1ms;
uint8_t Relay_On_Time_s;
bool Relay_Changed;
uint8_t Relay_State,Relay_Prev_State;

uint8_t Shutter_Position;
uint8_t Shutter_Target_Position;
uint8_t Shutter_Time_To_Target_s;
uint8_t Shutter_Full_Motion_Time_s;
bool Shutter_Learning;
uint8_t Shutter_Learning_State;
bool Code_Learning;

volatile bool Ext_SW_Pressed;
volatile bool Ext_SW_Released;
volatile uint8_t Ext_SW_Debounce_Timeout_ms;
uint8_t Ext_SW_Pressed_Time_s;

volatile uint8_t Serial_Rx_Buffer[8];
volatile uint8_t Serial_Rx_Counter;
uint8_t Serial_GroupID;
uint8_t Serial_DevID;






void InitTIM4(void)
{
  TIM4->PSCR=0;				//1 frequency division, timer clock equals system clock = 16m

  TIM4->ARR=0XA0;			//10us reload value 0XA0

  TIM4->CNTR=0;				//It is necessary to clear down the counter
  TIM4->IER |= 1<<0;		//Enable tim4 update interrupt
  

  TIM4->SR1  |= 1<<0;		//Clear tim4 update interrupt flag

  TIM4->CR1 |= 1<<7;		//Allow reassembly to enable timer
  TIM4->CR1 |= 1<<0;		//Enabling tim4 counter
}

void Save_Codes(void)
{
	uint8_t i;
	EEPROM_Config();
	for(i=0;i<5;i++) 
	{
		if(EV1527_Stored_Codes[i] != 0) 
		{
			EEPROM_Program4Byte((uint16_t)((i*4)+5),EV1527_Stored_Codes[i]);
			_delay_ms(1);
		}
	}
	EEPROM_ProgramByte(4,EV1527_Last_Code_Position);
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
	EV1527_Last_Code_Position = EEPROM_ReadByte(4);
	for ( i=0; i<5; i++)
	{
		EV1527_Stored_Codes[i] = EEPROM_Read4Byte((i*4)+5);
	}
}

void IO_Init (void)
{
	CLK_DeInit();
                
    CLK_HSECmd(DISABLE);
    CLK_LSICmd(DISABLE);
    CLK_HSICmd(ENABLE);
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	CLK_SYSCLKConfig(CLK_PRESCALER_CPUDIV1);
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
	GPIO_Init(GPIOD,GPIO_PIN_1,GPIO_MODE_IN_PU_NO_IT);			//EXT_SW
	GPIO_Init(GPIOD,GPIO_PIN_4,GPIO_MODE_IN_PU_NO_IT);
	GPIO_Init(GPIOD,GPIO_PIN_2,GPIO_MODE_OUT_PP_LOW_FAST);	//Relay1
	GPIO_Init(GPIOD,GPIO_PIN_3,GPIO_MODE_OUT_PP_LOW_FAST);	//Relay2
	
	GPIO_Init(GPIOB,GPIO_PIN_5,GPIO_MODE_OUT_OD_HIZ_FAST);	//LED
	
	EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOA,EXTI_SENSITIVITY_RISE_FALL);
	//EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOD,EXTI_SENSITIVITY_FALL_ONLY);

	UART1_DeInit();
	
	UART1_Init(SERIAL_BAUD,UART1_WORDLENGTH_8D,UART1_STOPBITS_1,UART1_PARITY_NO,UART1_SYNCMODE_CLOCK_DISABLE,UART1_MODE_RX_ENABLE);
	UART1_ITConfig(UART1_IT_RXNE_OR,ENABLE);
	
	InitTIM4();
	_delay_ms(500);
	enableInterrupts();
}

// TIM4 10us interrupts, base system tick
void Shutter_Calc_Time_To_Target(uint8_t Pos_Target)
{
	uint8_t Shutter_One_Position_Time_s;
	Shutter_One_Position_Time_s = Shutter_Full_Motion_Time_s / 10;
	if (Pos_Target == Shutter_Position) Shutter_Time_To_Target_s = 0;
	else if (Pos_Target == 10)		//full up, go until timeout to deoffset
	{
		Shutter_Time_To_Target_s = (uint8_t) Shutter_Full_Motion_Time_s * 1.5;
		Shutter_Target_Position = 10;
		Relay_State = 1;
		Relay_Changed = TRUE;
	}
	else if (Pos_Target == 0)		//full down, go until timeout to deoffset
	{
		Shutter_Time_To_Target_s = (uint8_t) Shutter_Full_Motion_Time_s * 1.5;
		Shutter_Target_Position = 0;
		Relay_State = 2;
		Relay_Changed = TRUE;
	}
	else if (Pos_Target > Shutter_Position) 
	{
		Shutter_Time_To_Target_s = (Pos_Target - Shutter_Position) * Shutter_One_Position_Time_s;
		Shutter_Target_Position = Pos_Target;
		Relay_State = 1;
		Relay_Changed = TRUE;
	}
	else if (Pos_Target < Shutter_Position)
	{
		Shutter_Time_To_Target_s = (Shutter_Position - Pos_Target) * Shutter_One_Position_Time_s;
		Shutter_Target_Position = Pos_Target;
		Relay_State = 2;
		Relay_Changed = TRUE;
	}
}

//checks and sets the relay outputs with deadtime at changes
void Relay_Channel_Set(uint8_t Channel)
{
	uint8_t Temp;
	if (Relay_Changed)
	{
		if (Channel == 0) 
		{
			GPIO_WriteLow(GPIOD, GPIO_PIN_2);
			GPIO_WriteLow(GPIOD,GPIO_PIN_3);
			GPIO_WriteHigh(GPIOB,GPIO_PIN_5);
			Relay_Change_Delay_1ms = RELAY_DEADTIME_1ms;
			if ((Shutter_Time_To_Target_s!=0) && (Relay_On_Time_s < Shutter_Time_To_Target_s)) 			// if stopped before reaching target position, calculate new position
			{
				if (Relay_Prev_State == 1) 
				{
					Shutter_Position = Shutter_Position + (Relay_On_Time_s / (Shutter_Full_Motion_Time_s / 10));
					if (Shutter_Position>10) Shutter_Position = 10;
					Shutter_Target_Position = Shutter_Position;
					Shutter_Time_To_Target_s = 0;
					Save_Position();

				}
				if (Relay_Prev_State == 2) 
				{
					Temp = Relay_On_Time_s / (Shutter_Full_Motion_Time_s / 10);
					if (Temp >= Shutter_Position ) 
						Shutter_Position = 0;
					else
						Shutter_Position = Shutter_Position - Temp;
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
			GPIO_WriteLow(GPIOB,GPIO_PIN_5);
			Relay_Change_Delay_1ms = RELAY_DEADTIME_1ms;
		}
		else if ((Channel ==2) && (Relay_Change_Delay_1ms == 0))
		{
			GPIO_WriteLow(GPIOD, GPIO_PIN_2);
			GPIO_WriteHigh(GPIOD,GPIO_PIN_3);
			GPIO_WriteLow(GPIOB,GPIO_PIN_5);
			Relay_Change_Delay_1ms = RELAY_DEADTIME_1ms;

		}
		Relay_Changed = FALSE;
	}
	// if reached target position stop
	if((Relay_State > 0) && (Relay_On_Time_s >= Shutter_Time_To_Target_s))
	{
		Shutter_Position = Shutter_Target_Position;
		Shutter_Time_To_Target_s = 0;
		Relay_Prev_State = Relay_State;
		Relay_State = 0; Relay_Changed = TRUE;
		Relay_On_Time_s = 0;
		Save_Position();
	}
	//if relay reached end stop for a time, turn off relay
	if ((Relay_State > 0) && (Relay_On_Time_s >= RELAY_ENDSTOP_TIMEOUT_S))
	{
		if (Relay_State == 1) Shutter_Position = 10;
		if (Relay_State == 2) Shutter_Position = 0;
		Shutter_Time_To_Target_s = 0;
		Relay_Prev_State = Relay_State;
		Relay_State = 0; Relay_Changed = TRUE;
		Relay_On_Time_s = 0;
		Save_Position();
	} 
}

void Shutter_RF_Task(void)
{
	uint8_t i = 0;
	if (EV1527_Valid_Code)
	{
		for (i = 0; i < 5; i++)
		{
			if (EV1527_Valid_Rec_Code == EV1527_Stored_Codes[i])
			{
				if (EV1527_Valid_Data)
				{
					Shutter_Calc_Time_To_Target(EV1527_Valid_Rec_Data / 10);
					EV1527_Valid_Data = FALSE;
				}
				else
				{
					if (Relay_State == 0)
					{
						if (Relay_Prev_State == 1)
						{
							Relay_Prev_State = 2;
							Shutter_Calc_Time_To_Target(0);
						}
						else
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
		EV1527_Valid_Code = FALSE;
	}
}

void Ext_SW_Check(void)
{
	// External switch handling
	if (((GPIOD->IDR & GPIO_PIN_1) != 0) && (Ext_SW_Debounce_Timeout_ms == 0) && (Ext_SW_Pressed == TRUE))
	{
		Ext_SW_Released = TRUE;
		Ext_SW_Pressed = FALSE;
		Ext_SW_Debounce_Timeout_ms = EXT_SW_DEBOUNCE_MS;
	}
	if (((GPIOD->IDR & GPIO_PIN_1) == 0) && (Ext_SW_Debounce_Timeout_ms == 0) && (Ext_SW_Released == FALSE))
	{
		Ext_SW_Released = FALSE;
		Ext_SW_Pressed = TRUE;
		Ext_SW_Debounce_Timeout_ms = EXT_SW_DEBOUNCE_MS;
	}
}

void Ext_SW_Task(void)
{
	Ext_SW_Check();
	
	// Normal mode switch parsing
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
				else 
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
		else if (Ext_SW_Released && (Ext_SW_Pressed_Time_s >= EXT_SW_LEARN_PRESS_TIME_S))
		{
			Ext_SW_Pressed_Time_s = 0;
			Ext_SW_Released = FALSE;
			Code_Learning = TRUE;
			
		}
		else if (Ext_SW_Released && (Ext_SW_Pressed_Time_s >= EXT_SW_LONG_PRESS_TIME_S))
		{
			Shutter_Learning = TRUE;
			Shutter_Learning_State = 1;
			Ext_SW_Pressed_Time_s = 0;
			Ext_SW_Released = FALSE;
			
		}
	} 
	if (Shutter_Learning)
	{
		if (Shutter_Learning_State == 1)		//Show learning state and go up
		{
			Relay_Changed = TRUE;
			Relay_Change_Delay_1ms = 0;
			Relay_Channel_Set(1);

			_delay_ms(1000);
			Relay_Changed = TRUE;
			Relay_Change_Delay_1ms = 0;
			Relay_Channel_Set(0);

			_delay_ms(1000);
			Relay_Changed = TRUE;
			Relay_Change_Delay_1ms = 0;
			Relay_Channel_Set(1);
			Shutter_Learning_State = 2;
		}
		if ((Shutter_Learning_State == 2) && Ext_SW_Released) //Wait for switch to confirm top position and stop
		{
			Ext_SW_Released = FALSE;
			Relay_Changed = TRUE;
			Relay_Channel_Set(0);
			Shutter_Learning_State = 3;
		}
		if ((Shutter_Learning_State == 3) && Ext_SW_Released) //Wait for switch to start counting Time and go down
		{
		Ext_SW_Released = FALSE;
		Shutter_Full_Motion_Time_s = 0;
		Relay_Changed = TRUE;
		Relay_Channel_Set(2);
		Shutter_Learning_State = 4;
		}
		if ((Shutter_Learning_State == 4) && Ext_SW_Released) //Wait for switch to confirm bottom position and save time, exit Learning mode
		{
			Ext_SW_Released = FALSE;
			Shutter_Learning = FALSE;
			Shutter_Learning_State = 0;
			Relay_Changed = TRUE;
			Relay_Channel_Set(0);
			Save_Learned_Time();
			_delay_ms(1000);
			Shutter_Position = 0;							// set actual position
			Shutter_Calc_Time_To_Target(5);					// move halfway up
		}
	}
	if (Code_Learning)
	{
			uint8_t i;
			bool New_Code;
			if(EV1527_Valid_Learn_Code)
			{
				New_Code = TRUE;
				Code_Learning = FALSE;
				for(i=0;i<5;i++)
				{
					if (EV1527_Stored_Codes[i] == EV1527_Valid_Rec_Code) New_Code = FALSE;
				} 
				if (New_Code)
				{	
					EV1527_Stored_Codes[EV1527_Last_Code_Position] = EV1527_Valid_Rec_Code;
					EV1527_Last_Code_Position ++;
					if(EV1527_Last_Code_Position == 4) EV1527_Last_Code_Position = 0;
					Save_Codes();
				}
				EV1527_Valid_Learn_Code = FALSE;
				
			}
	}
	
}
void Timer_Task(void)
{
	
	//mS based Timer
	if (Timer_10us_to_ms >= 100)
	{
		Timer_10us_to_ms = 0;
		Timer_ms_to_s --;
		//1 ms tasks
		if (Relay_Change_Delay_1ms) Relay_Change_Delay_1ms--;
		if (Ext_SW_Debounce_Timeout_ms>0) 
		Ext_SW_Debounce_Timeout_ms--;

	}
	//Seconds based timer
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

	Serial_Rx_Counter = 0;
	if (Serial_Rx_Buffer[6] != '\r')
		return;

	if (Serial_Rx_Buffer[2] == '1')
	{
		EV1527_Stored_Codes[0] = (Serial_Rx_Buffer[3] * (uint32_t)0x10000) + (Serial_Rx_Buffer[4] * (uint32_t)0x100) + Serial_Rx_Buffer[5];
		Save_Codes();
	}
	else
	if (Serial_Rx_Buffer[2] == '2')
	{
		EV1527_Stored_Codes[1] = (Serial_Rx_Buffer[3] * (uint32_t)0x10000) + (Serial_Rx_Buffer[4] * (uint32_t)0x100) + Serial_Rx_Buffer[5];
		Save_Codes();
	}
	else
	if (Serial_Rx_Buffer[2] == '3')
	{
		EV1527_Stored_Codes[2] = (Serial_Rx_Buffer[3] * (uint32_t)0x10000) + (Serial_Rx_Buffer[4] * (uint32_t)0x100) + Serial_Rx_Buffer[5];
		Save_Codes();
	}
	else
	if (Serial_Rx_Buffer[2] == '4')
	{
		EV1527_Stored_Codes[3] = (Serial_Rx_Buffer[3] * (uint32_t)0x10000) + (Serial_Rx_Buffer[4] * (uint32_t)0x100) + Serial_Rx_Buffer[5];
		Save_Codes();
	}
	else
	if (Serial_Rx_Buffer[2] == '5')
	{
		EV1527_Stored_Codes[4] = (Serial_Rx_Buffer[3] * (uint32_t)0x10000) + (Serial_Rx_Buffer[4] * (uint32_t)0x100) + Serial_Rx_Buffer[5];
		Save_Codes();
	}
	else
	if (Serial_Rx_Buffer[2] == 'C')
	{
		switch (Serial_Rx_Buffer[3])
		{
		case 'S':
			if (Relay_State == 0)
			{
				if (Relay_Prev_State == 1)
				{
					Relay_Prev_State = 2;
					Shutter_Calc_Time_To_Target(0);
				}
				else
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
			break;
		case 'U':
			Shutter_Calc_Time_To_Target(10);
			break;
		case 'D':
			Shutter_Calc_Time_To_Target(0);
			break;
		case 'P':
			Shutter_Calc_Time_To_Target(Serial_Rx_Buffer[4]);
			break;
		default:
			break;
		}
	}
	else
	if (Serial_Rx_Buffer[2] == 'S')
	{
		switch (Serial_Rx_Buffer[3])
		{
		case 'T':
			Shutter_Full_Motion_Time_s = Serial_Rx_Buffer[4];
			Save_Learned_Time();
			break;
		case 'I':
			Serial_GroupID = Serial_Rx_Buffer[4];
			Serial_DevID = Serial_Rx_Buffer[5];
			Save_IDs();
			break;
		default:
			break;
		}
	}
}

void main(void)

{
	
	IO_Init();
	Load_Saved_Stuff();
	while (1)
	{
		Relay_Channel_Set(Relay_State);
		Timer_Task();
		EV1527_Receive_Parse();
		EV1527_Receive_Check();
		Shutter_RF_Task(); 
		Ext_SW_Task();		
		if ( Serial_Rx_Counter == 7) 
		{
			Serial_Task();
			UART1_ITConfig(UART1_IT_RXNE_OR, ENABLE);
		}
	}
}
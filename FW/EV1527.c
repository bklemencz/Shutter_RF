#include "EV1527.h"

const uint8_t EV1527_PulseTime_Max_10us = 58;
const uint8_t EV1527_PulseTime_Min_10us = 48;
const uint16_t EV1527_Pulse_Timeout_10us = EV1527_PulseTime_Max_10us * 35;
const uint16_t EV1527_Packet_Repeat_Timeout_10us = EV1527_PulseTime_Max_10us * 75;
const uint16_t EV1527_Sync_Low_Max_10us = EV1527_PulseTime_Max_10us * 31;
const uint16_t EV1527_Sync_Low_Min_10us = EV1527_PulseTime_Min_10us * 31;
const uint16_t EV1527_3_PulseTime_Max_10us = EV1527_PulseTime_Max_10us * 3;
const uint16_t EV1527_3_PulseTime_Min_10us = EV1527_PulseTime_Min_10us * 3;

extern uint32_t EV1527_Rec_Code;
extern uint8_t EV1527_Rec_Data;
extern uint8_t EV1527_Repeat_Count;
extern bool EV1527_Valid_Code;
extern bool EV1527_Valid_Data;

bool EV1527_sync_OK;
bool EV1527_bit_OK;
bool EV1527_packet_OK;
bool EV1527_last_edge_type;
uint16_t EV1527_last_edge_time_10us;
uint8_t EV1527_bit_count;
volatile uint16_t Timer_last_valid_packet_10us;
volatile uint16_t Timer_last_edge_10us;

void EV1527_Reset_Receive(void)
{
    EV1527_sync_OK = FALSE;
    EV1527_bit_OK = FALSE;
    EV1527_packet_OK = FALSE;
    EV1527_last_edge_type = 0;
    EV1527_bit_count = 0;
    
}

void EV1527_Receive_Parse(bool EV1527_edge_type, uint16_t EV1527_edge_time_10us)
{
    
    if (!EV1527_sync_OK)
    {
        if ((EV1527_last_edge_time_10us<EV1527_PulseTime_Max_10us) && (EV1527_last_edge_time_10us>EV1527_PulseTime_Min_10us) && !EV1527_last_edge_type
        && (EV1527_edge_time_10us<EV1527_Sync_Low_Max_10us) && (EV1527_last_edge_time_10us>EV1527_Sync_Low_Min_10us) && EV1527_edge_type)
        {
            EV1527_sync_OK = TRUE;
        }
        EV1527_last_edge_time_10us = EV1527_edge_time_10us;
        EV1527_last_edge_type = EV1527_edge_type;
    } 
    else if(EV1527_sync_OK)
    {
        // Bit 0
        if ((EV1527_last_edge_time_10us<EV1527_PulseTime_Max_10us) && (EV1527_last_edge_time_10us>EV1527_PulseTime_Min_10us) && !EV1527_last_edge_type
        && (EV1527_edge_time_10us<EV1527_3_PulseTime_Max_10us) && (EV1527_edge_time_10us>EV1527_3_PulseTime_Min_10us) && EV1527_edge_type)
        {
            if (EV1527_bit_count<24)
            {
                EV1527_Rec_Code << 1;
            }
            else if (EV1527_bit_count < 32)
            {
                EV1527_Rec_Data << 1;
            }
            EV1527_bit_count ++;
            if ((EV1527_bit_count==24) || (EV1527_bit_count ==32))
            { 
                Timer_last_valid_packet_10us = 0;
            }
        } 
        //bit 1
        else if ((EV1527_edge_time_10us<EV1527_PulseTime_Max_10us) && (EV1527_edge_time_10us>EV1527_PulseTime_Min_10us) && !EV1527_last_edge_type
        && (EV1527_last_edge_time_10us<EV1527_3_PulseTime_Max_10us) && (EV1527_last_edge_time_10us>EV1527_3_PulseTime_Min_10us) && EV1527_edge_type)
        {
            if (EV1527_bit_count<24)
            {
                EV1527_Rec_Code << 1;
                EV1527_Rec_Code ++;
            }
            else if (EV1527_bit_count < 32)
            {
                EV1527_Rec_Data << 1;
                EV1527_Rec_Data++;
            }
            EV1527_bit_count ++;
            if ((EV1527_bit_count==24) || (EV1527_bit_count ==32))
            { 
                Timer_last_valid_packet_10us = 0;
            }
        } 
    }
}

void EV1527_Receive_Check(void)
{
    if (Timer_last_edge_10us > EV1527_Pulse_Timeout_10us)
    {
        if (EV1527_bit_count == 24) 
        {
            EV1527_Valid_Code = TRUE;
            EV1527_Valid_Data = FALSE;
            if (Timer_last_valid_packet_10us > EV1527_Packet_Repeat_Timeout_10us) 
            {
                EV1527_Repeat_Count = 0;
            } else
            {
                EV1527_Repeat_Count ++;
            }
            
        }
        else if (EV1527_bit_count == 32) 
        {
            EV1527_Valid_Code = TRUE;
            EV1527_Valid_Data = TRUE;
            if (Timer_last_valid_packet_10us > EV1527_Packet_Repeat_Timeout_10us) 
            {
                EV1527_Repeat_Count = 0;
            } else
            {
                EV1527_Repeat_Count ++;
            }
        } 
        EV1527_Reset_Receive();
    } 
}  
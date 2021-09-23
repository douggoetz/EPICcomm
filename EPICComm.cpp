/*
 *  EPICComm.cpp
 *  Author:  Lars Kalnajs
 *  Created: September 2019
 *  
 *  This file implements an Arduino library (C++ class) that implements the communication
 *  between the PIB and PU. The class inherits its protocol from the SerialComm
 *  class which has been modified to the LoRaComm class to allow communications through the 
 *  LoRa.h library.   LoRa uses packetized communications, this library converts the Stream class that
 *  SerComm expects into a buffer using LoopbackStream, which can then be sent to the LoRa object (TX)
 *  and filled from the LoRa object (RX). 
 */

#include "EPICComm.h"

EPICComm::EPICComm(Stream * serial_port)
    : LoRaComm(serial_port)
{
}

// For use with default SPI and pins
bool EPICComm::Begin(long frequency) 
{
    if (!LoRa.begin(frequency))
        return false;
    return true;
}

// If you need to use different pins (ie EPIC Board)
bool EPICComm::Begin(long frequency,int SSPin, int ResetPin, int InteruptPin)
{
    SPI.begin();
    LoRa.setPins(SSPin, ResetPin,InteruptPin);
    if (!LoRa.begin(frequency))
        return false;
    return true;
}

// If you need to use different pins and a different SPI (ie Profiler)
bool EPICComm::Begin(long frequency,int SSPin, int ResetPin, int InteruptPin, SPIClass& spi)
{
    LoRa.setPins(SSPin, ResetPin,InteruptPin);
    LoRa.setSPI(spi);
    if (!LoRa.begin(frequency))
        return false;
    return true;
}

void EPICComm::SetModulation(int SpreadingFactor, long BandWidth)
{
    LoRa.setSpreadingFactor(SpreadingFactor);
    LoRa.setSignalBandwidth(BandWidth);
}

void EPICComm::SetTXPower(int Power)
{
    LoRa.setTxPower(Power);

}

// MonDo -> EPIC (with params) ---------------------------

bool EPICComm::TX_SetHeaters(float HeaterT, float BoostT)
{
    if (!Add_float(HeaterT)) return false;
    if (!Add_float(BoostT)) return false;
   
    TX_ASCII(EFU_SET_HEATERS);

    return true;
}

bool EPICComm::RX_SetHeaters(float * HeaterT, float * BoostT)
{
    float temp1, temp2;

    if (!Get_float(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    *HeaterT = temp1;
    *BoostT = temp2;
    return true;
}

bool EPICComm::TX_SetDataRate(int32_t DataRate)
{
    if (!Add_int32(DataRate)) return false;
   
    TX_ASCII(EFU_SET_DATA_RATE);

    return true;
}

bool EPICComm::RX_SetDataRate(int32_t * DataRate)
{
    int32_t temp1;

    if (!Get_int32(&temp1)) return false;
   
    *DataRate = temp1;

    return true;
}

bool EPICComm::TX_SetTxRate(int32_t TxRate)
{
    if (!Add_int32(TxRate)) return false;
   
    TX_ASCII(EFU_SET_TX_RATE);

    return true;
}

bool EPICComm::RX_SetTxRate(int32_t * TxRate)
{
    int32_t temp1;

    if (!Get_int32(&temp1)) return false;
   
    *TxRate = temp1;

    return true;
}


bool EPICComm::TX_Data(uint32_t EFUTime, float Lat, float Lon, uint16_t Alt, uint16_t TSEN_T, uint32_t TSEN_P, uint32_t TSEN_TP)

{

    if (!Add_uint32(EFUTime)) return false;
    if (!Add_float(Lat)) return false;
    if (!Add_float(Lon)) return false;
    if (!Add_uint16(Alt)) return false;
    if (!Add_uint16(TSEN_T)) return false;
    if (!Add_uint32(TSEN_P)) return false;
    if (!Add_uint32(TSEN_TP)) return false;
    
    TX_ASCII(EFU_TSEN);

    return true;
}


bool EPICComm::RX_Data(uint32_t * EFUTime, float * Lat, float * Lon, uint16_t * Alt, uint16_t * TSEN_T, uint32_t * TSEN_P, uint32_t * TSEN_TP)
{
    uint32_t temp1, temp6, temp7;
    float temp2, temp3;//, temp8;
    uint16_t temp4, temp5, temp9, temp10;
    int16_t temp8;
    uint8_t temp11;

    if (!Get_uint32(&temp1)) return false;
    if (!Get_float(&temp2)) return false;
    if (!Get_float(&temp3)) return false;
    if (!Get_uint16(&temp4)) return false;
    if (!Get_uint16(&temp5)) return false;
    if (!Get_uint32(&temp6)) return false;
    if (!Get_uint32(&temp7)) return false;
    // if (!Get_float(&temp8)) return false;
    // if (!Get_uint16(&temp9)) return false;
    // if (!Get_uint16(&temp10)) return false;
    // if (!Get_uint8(&temp11)) return false;

    *EFUTime = temp1;
    *Lat = temp2;
    *Lon = temp3;
    *Alt = temp4;
    *TSEN_T = temp5;
    *TSEN_P = temp6;
    *TSEN_TP = temp7;
    // *T_Battery = temp8;
    // *V_Battery = temp9;
    // *V_DCDC = temp10;
    // *HeaterStat = temp11;
    return true;
}

bool EPICComm::TX_HK(uint32_t EFUTime, uint16_t V_Battery, uint16_t V_DCDC, uint16_t V_3v3, float T_Battery, float T_PCB, uint8_t HeaterStat)
{

    if (!Add_uint32(EFUTime)) return false;
    if (!Add_uint16(V_Battery)) return false;
    if (!Add_uint16(V_DCDC)) return false;
    if (!Add_uint16(V_3v3)) return false;
    if (!Add_float(T_Battery)) return false;
    if (!Add_float(T_PCB)) return false;
    if (!Add_uint8(HeaterStat)) return false;

    TX_ASCII(EFU_HK);

    return true;

}

bool EPICComm::RX_HK(uint32_t * EFUTime, uint16_t * V_Battery, uint16_t * V_DCDC, uint16_t * V_3v3, float * T_Battery, float * T_PCB, uint8_t * HeaterStat)
{

    uint32_t temp1;
    uint16_t temp2, temp3, temp4;
    float temp5, temp6;
    uint8_t temp7;

    if (!Get_uint32(&temp1)) return false;
    if (!Get_uint16(&temp2)) return false;
    if (!Get_uint16(&temp3)) return false;
    if (!Get_uint16(&temp4)) return false;
    if (!Get_float(&temp5)) return false;
    if (!Get_float(&temp6)) return false;
    if (!Get_uint8(&temp7)) return false;

    *EFUTime = temp1;
    *V_Battery = temp2;
    *V_DCDC = temp3;
    *V_3v3 = temp4;
    *T_Battery = temp5;
    *T_PCB = temp6;
    *HeaterStat = temp7;
    return true;

}

bool EPICComm::TX_ResetInst()
{

    TX_ASCII(EFU_RESET);
    return true;

}

//bool EPICComm::RX_ResetInst()
//{

  //  return true;
//}


// // -- EFU to PIB error string

bool EPICComm::TX_Error(const char * error)
{
    if (Add_string(error)) return false;

    TX_ASCII(EFU_ERROR);

    return true;
}

bool EPICComm::RX_Error(char * error, uint8_t buffer_size)
{
    return Get_string(error, buffer_size);
}


// // -- Ground to EPIC Drift Valve

bool EPICComm::TX_SetValvePos(uint8_t OpenPercent)
{

    if (!Add_uint8(OpenPercent)) return false;
   
    TX_ASCII(DRIFT_SET_VALVE_POS);

    return true;
}

bool EPICComm::RX_SetValvePos(uint8_t * OpenPercent)

{   

    uint8_t temp1;

    if (!Get_uint8(&temp1)) return false;
   
    *OpenPercent = temp1;

    return true;


}

bool EPICComm::TX_SetMode(uint8_t ZephMode){

    if (!Add_uint8(ZephMode)) return false;
   
    TX_ASCII(ZEPH_MODE);

    return true;


}
bool EPICComm::RX_SetMode(uint8_t * ZephMode){

    uint16_t temp1;

    if (!Get_uint16(&temp1)) return false;
   
    *ZephMode = temp1;

    return true;


}

bool EPICComm::TX_ReelOut(uint16_t outrevs){

    if (!Add_uint16(outrevs)) return false;
   
    TX_ASCII(ZEPH_REEL_OUT);

    return true;


}

bool EPICComm::RX_ReelOut(uint16_t * outrevs){

    uint16_t temp1;

    if (!Get_uint16(&temp1)) return false;
   
    *outrevs = temp1;

    return true;


}

bool EPICComm::TX_ReelIn(uint16_t inrevs){

    if (!Add_uint16(inrevs)) return false;
   
    TX_ASCII(ZEPH_REEL_IN);

    return true;


}

bool EPICComm::RX_ReelIn(uint16_t *inrevs){

    uint16_t temp1;

    if (!Get_uint16(&temp1)) return false;
   
    *inrevs = temp1;

    return true;


}

bool EPICComm::ResetFTR(){

    TX_ASCII(ZEPH_RESETFTR);

}

bool EPICComm::CancelMotion(){

    TX_ASCII(ZEPH_CANCELMOT);

}

bool EPICComm::TX_SetTXPower(uint8_t txpw){

    if (!Add_uint8(txpw)) return false;
   
    TX_ASCII(TX_PWR);

    return true;


}


bool EPICComm::RX_SetTXPower(uint8_t * txpw){

     uint8_t temp1;

    if (!Get_uint8(&temp1)) return false;
   
    *txpw = temp1;

    return true;

    
}



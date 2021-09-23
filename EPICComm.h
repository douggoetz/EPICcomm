/*
 *  EPICComm.h
 *  Author:  Lars Kalnajs
 *  Created: September 2019
 *  
 *  This file declares an Arduino library (C++ class) that implements the communication
 *  between the PIB and the PU. The class inherits its protocol from the SerialComm
 *  class.
 */

#ifndef EPICComm_H
#define EPICComm_H

#include "LoRaComm.h"

/* Note for use with Teensy 3.X, the following lines in LoRa.cpp need to be modified:
Line 365:
    SPI.notUsingInterrupt((IRQ_NUMBER_t)digitalPinToInterrupt(_dio0));
Line 383: 
  SPI.notUsingInterrupt((IRQ_NUMBER_t)digitalPinToInterrupt(_dio0));
*/
#include <LoRa.h>
#include <SPI.h>     

enum PUMessages_t : uint8_t {
    EFU_NO_MESSAGE = 0,

    // MonDo -> EFU (no params)
    EFU_SEND_STATUS, //1
    EFU_RESET, //2

    // MonDo -> EFU (with params)
    EFU_SET_HEATERS, //3
    EFU_SET_DATA_RATE, //4
    EFU_SET_TX_RATE, //5

    // EFU -> MonDo (no params)
    EFU_DATA_RECORD,  //6 binary transfer

    // EFU -> MonDo (with params)
    EFU_STATUS, //7
    EFU_ERROR, //8

    //Ground -> EPIC (Drift Valve)
    DRIFT_SET_VALVE_POS, //9

    ZEPH_MODE, //10
    ZEPH_REEL_OUT, //11
    ZEPH_REEL_IN, //12
    ZEPH_RESETFTR, //13

    TX_PWR, //

    ZEPH_CANCELMOT, //15

    EFU_TSEN, //16
    EFU_HK, //17
    
};


class EPICComm : public LoRaComm {
public:
    EPICComm(Stream * serial_port);
    ~EPICComm() { };

    //Setup 
    bool Begin(long frequency);
    bool Begin(long frequency,int SSPin, int ResetPin, int InteruptPin);
    bool Begin(long frequency,int SSPin, int ResetPin, int InteruptPin, SPIClass& spi);
    void SetModulation(int SpreadingFactor, long BandWidth);
    void SetTXPower(int Power);



    // MonDo -> EPIC (with params) -----------------------
    bool TX_SetHeaters(float HeaterT, float BoostT); //Set the heater temperature (parameter not state)
    bool RX_SetHeaters(float * HeaterT, float * BoostT);
    bool TX_SetDataRate(int32_t DataRate);
    bool RX_SetDataRate(int32_t * DataRate);
    bool TX_SetTxRate(int32_t TxRate);
    bool RX_SetTxRate(int32_t * TXRate);
    bool TX_ResetInst();
    bool RX_ResetInst();




    // EPIC -> MonDo (with params) -----------------------

    bool TX_Data(uint32_t EFUTime, float Lat, float Lon, uint16_t Alt, uint16_t TSEN_T, uint32_t TSEN_P, uint32_t TSEN_TP);
    bool RX_Data(uint32_t * EFUTime, float * Lat, float * Lon, uint16_t * Alt, uint16_t * TSEN_T, uint32_t * TSEN_P, uint32_t * TSEN_TP);
    bool TX_HK(uint32_t EFUTime, uint16_t V_Battery, uint16_t V_DCDC, uint16_t V_3v3, float T_Battery, float T_PCB, uint8_t HeaterStat);
    bool RX_HK(uint32_t * EFUTime, uint16_t * V_Battery, uint16_t * V_DCDC, uint16_t * V_3v3, float * T_Battery, float * T_PCB, uint8_t * HeaterStat);
    bool TX_Error(const char * error);
    bool RX_Error(char * error, uint8_t buffer_size);

    //Ground -> EPIC (Drift Valve) ------------------------
    bool TX_SetValvePos(uint8_t OpenPercent); //percent open for valve (0 = full close, 100 = full open)
    bool RX_SetValvePos(uint8_t * OpenPercent);

    //Ground -> EPIC (Zephyr) ------------------------------
    bool TX_SetMode(uint8_t ZephMode);
    bool RX_SetMode(uint8_t * ZephMode);
    bool TX_ReelOut(uint16_t outrevs);
    bool RX_ReelOut(uint16_t * outrevs);
    bool TX_ReelIn(uint16_t inrevs);
    bool RX_ReelIn(uint16_t *inrevs);
    bool ResetFTR();

    bool TX_SetTXPower(uint8_t txpw);
    bool RX_SetTXPower(uint8_t * txpw); 

    bool CancelMotion();

};

#endif /* EPICComm_H */

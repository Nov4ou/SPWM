/*
 * ina238.h
 *
 *  Created on: Mar 22, 2024
 *      Author: nov4ou
 */

#ifndef APP_INA238_INA238_H_
#define APP_INA238_INA238_H_

#include "F2806x_Device.h"          // F2806x Headerfile
#include "F2806x_Examples.h"        // F2806x Examples Headerfile
#include "iic.h"

#define DEVICE_ADDRESS        0x41
//#define DEVICE_ADDRESS        0x40

#define CONFIG_REG            0x00
#define ADC_CONFIG            0x01
#define SHUNT_VOLTAGE_REG     0x04
#define BUS_VOLTAGE_REG       0x05
#define SHUNT_CALIBRATION_REG 0x02
#define MANUFACTURER_ID_REG   0x3E
#define TEMP_REG              0x06


#define CONFIG_RST            0x01
#define CONFIG_CONVDLY_0      0x00
#define CONFIG_CONVDLY_2      0x01
#define CONFIG_CONVDLY_510    0xFF
#define ADCRANGE_163_84       0x00
#define ADCRANGE_40_96        0x01

//Uint16 reset, config_convdly, adc_range;


void INA238_Init();
unsigned char INA238_ReadOneByte(Uint16 ReadAddr);
Uint16 INA238_ReadTwoByte(Uint16 ReadAddr);
void INA238_WriteBytes(Uint8 *pData, Uint16 Size);
void INA238_WriteLenByte(Uint16 WriteAddr,Uint32 DataToWrite,unsigned char Len);

void INA238_Read_Manufacturer_ID();
void INA238_Reset();
void INA238_Read_Shunt_Voltage();
void INA238_Read_Bus_Voltage();
void INA238_Read_Temperature();

#endif /* APP_INA238_INA238_H_ */

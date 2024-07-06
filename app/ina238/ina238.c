#include "ina238.h"
#include <string.h>

Uint16 rawRecValue = 0;
float shuntVoltage = 0.0;
float busVoltage = 0.0;
float temperature = 0.0;
unsigned char data[3];
Uint8 recData[2];
unsigned char tempid[2] = {0, 0};
Uint8 id[2];

void INA238_Init()
{
  IICA_Init();

  // Set config register as 0x0050;
  // 00 00000000 0 1 0000
  data[0] = CONFIG_REG;
  data[1] = 0x00;
  data[2] = 0x10;
  INA238_WriteBytes(data, sizeof(data));


  // Set ADC configuration register as 0xF491
  // 1111 000 000 000 000
  data[0] = ADC_CONFIG;
  data[1] = 0xF0;
  data[2] = 0x02;
  INA238_WriteBytes(data, sizeof(data));

//  data[0] = SHUNT_CALIBRATION_REG;
//  data[1] = 0x0B;
//  data[2] = 0xBA;
//  INA238_WriteBytes(data, sizeof(data));
}


void INA238_WriteBytes(unsigned char *pData, Uint16 Size)
{
    data[0] = pData[0];
    data[1] = pData[1];
    data[2] = pData[2];

    IIC_Start();
    IIC_Send_Byte((DEVICE_ADDRESS << 1) + 0);
    IIC_Wait_Ack();

    IIC_Send_Byte(data[0]);
    IIC_Wait_Ack();
    IIC_Send_Byte(data[1]);
    IIC_Wait_Ack();
    IIC_Send_Byte(data[2]);
    IIC_Wait_Ack();
    IIC_Stop();
    DELAY_US(10*1000);
}


unsigned char INA238_ReadOneByte(Uint16 ReadAddr)
{
    unsigned char temp=0;
    IIC_Start();

    IIC_Send_Byte((DEVICE_ADDRESS << 1) + 0 +((ReadAddr/256)<<1));   //����������ַ0XA0,д����

    IIC_Wait_Ack();
    IIC_Send_Byte(ReadAddr%256);   //���͵͵�ַ
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((DEVICE_ADDRESS << 1) + 0 + 1);           //�������ģʽ
    IIC_Wait_Ack();
    temp=IIC_Read_Byte(0);
    IIC_Stop();//����һ��ֹͣ����
    return temp;
}



Uint16 INA238_ReadTwoByte(Uint16 ReadAddr)
{
    Uint8 temp[2];
    Int16 tempvalue;
    IIC_Start();

    IIC_Send_Byte((DEVICE_ADDRESS << 1) + 0 +((ReadAddr/256)<<1));   //����������ַ0XA0,д����

    IIC_Wait_Ack();
    IIC_Send_Byte(ReadAddr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((DEVICE_ADDRESS << 1) + 1);
    IIC_Wait_Ack();
    temp[0] = IIC_Read_Byte(1);
    temp[1] = IIC_Read_Byte(0);
    IIC_Stop();
    tempvalue = (Int16) ((temp[0] << 8) | temp[1]);
    return tempvalue;
}




/*******************************************************************************
* �� �� ��         : INA238_WriteOneByte
* ��������         : ��INA238ָ����ַд��һ������
* ��    ��         : WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ
                     DataToWrite:Ҫд�������
* ��    ��         : ��
*******************************************************************************/
void INA238_WriteOneByte(Uint16 WriteAddr,unsigned char DataToWrite)
{
    IIC_Start();


        IIC_Send_Byte(0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д����

    IIC_Wait_Ack();
    IIC_Send_Byte(WriteAddr%256);   //���͵͵�ַ
    IIC_Wait_Ack();
    IIC_Send_Byte(DataToWrite);     //�����ֽ�
    IIC_Wait_Ack();
    IIC_Stop();//����һ��ֹͣ����
    DELAY_US(10*1000);
}

void INA238_Read_Shunt_Voltage()
{
    rawRecValue = INA238_ReadTwoByte(SHUNT_VOLTAGE_REG);
    shuntVoltage = rawRecValue * 1.25 / 1000;
//    memset(recData, 0, sizeof(recData));
}

void INA238_Read_Bus_Voltage()
{
    rawRecValue = INA238_ReadTwoByte(BUS_VOLTAGE_REG);
    busVoltage = rawRecValue * 3.125 / 1000;
//    memset(recData, 0, sizeof(recData));
}

void INA238_Read_Manufacturer_ID()
{
    rawRecValue = INA238_ReadTwoByte(MANUFACTURER_ID_REG);
    id[0] = (rawRecValue >> 8) & 0xFF;
    id[1] = rawRecValue & 0xFF;
//    memset(recData, 0, sizeof(recData));
}

void INA238_Read_Temperature()
{
    rawRecValue = INA238_ReadTwoByte(TEMP_REG);
    rawRecValue = rawRecValue >> 4;
    temperature = rawRecValue * 125.0 / 1000;
}

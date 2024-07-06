/*
 * iic.h
 *
 *  Created on: 2018-2-5
 *      Author: Administrator
 */

#ifndef IIC_H_
#define IIC_H_


#include "F2806x_Device.h"          // F2806x Headerfile
#include "F2806x_Examples.h"        // F2806x Examples Headerfile


#define IIC_SCL_SETH    (GpioDataRegs.GPASET.bit.GPIO29=1)
#define IIC_SCL_SETL    (GpioDataRegs.GPACLEAR.bit.GPIO29=1)

#define IIC_SDA_SETH    (GpioDataRegs.GPASET.bit.GPIO28=1)
#define IIC_SDA_SETL    (GpioDataRegs.GPACLEAR.bit.GPIO28=1)

#define READ_SDA        (GpioDataRegs.GPADAT.bit.GPIO28)



//IIC���в�������
void IICA_Init(void);                //��ʼ��IIC��IO��
void IIC_Start(void);               //����IIC��ʼ�ź�
void IIC_Stop(void);                //����IICֹͣ�ź�
void IIC_Send_Byte(unsigned char txd);          //IIC����һ���ֽ�
unsigned char IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
unsigned char IIC_Wait_Ack(void);               //IIC�ȴ�ACK�ź�
void IIC_Ack(void);                 //IIC����ACK�ź�
void IIC_NAck(void);                //IIC������ACK�ź�


#endif /* IIC_H_ */

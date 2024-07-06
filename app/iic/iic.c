/*
 * iic.c
 *
 *  Created on: 2018-2-5
 *      Author: Administrator
 */

#include "iic.h"


/*******************************************************************************
* �� �� ��         : IIC_Init
* ��������		   : IIC��ʼ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IICA_Init(void)
{
	EALLOW;
//	SysCtrlRegs.PCLKCR3.bit.GPIOINENCLK = 1;// ����GPIOʱ��
	GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;	  	//����
	GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;   	// ����˿�
	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 0;  	// IO��
	GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3; 	// ��ͬ��

	GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;	  	//����
	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;   	// ����˿�
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;  	// IO��
	GpioCtrlRegs.GPAQSEL2.bit.GPIO29 = 3;   // ��ͬ��
	EDIS;
}

/*******************************************************************************
* �� �� ��         : SDA_OUT
* ��������		   : SDA�������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void SDA_OUT(void)
{
	EALLOW;
	GpioCtrlRegs.GPADIR.bit.GPIO28=1;       //Output. SDA
	EDIS;
}

/*******************************************************************************
* �� �� ��         : SDA_IN
* ��������		   : SDA��������
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void SDA_IN(void)
{
	EALLOW;
	GpioCtrlRegs.GPADIR.bit.GPIO28=0;       //Input, SDA
	EDIS;
}

/*******************************************************************************
* �� �� ��         : IIC_Start
* ��������		   : ����IIC��ʼ�ź�
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA_SETH;
	IIC_SCL_SETH;
	DELAY_US(5);
	IIC_SDA_SETL;//START:when CLK is high,DATA change form high to low
	DELAY_US(6);
	IIC_SCL_SETL;//ǯסI2C���ߣ�׼�����ͻ��������
}

/*******************************************************************************
* �� �� ��         : IIC_Stop
* ��������		   : ����IICֹͣ�ź�
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_SETL;
	IIC_SDA_SETL;//STOP:when CLK is high DATA change form low to high
	IIC_SCL_SETH;
 	DELAY_US(6);
 	IIC_SDA_SETH;//����I2C���߽����ź�
	DELAY_US(6);
}

/*******************************************************************************
* �� �� ��         : IIC_Wait_Ack
* ��������		   : �ȴ�Ӧ���źŵ���
* ��    ��         : ��
* ��    ��         : 1������Ӧ��ʧ��
        			 0������Ӧ��ɹ�
*******************************************************************************/
unsigned char IIC_Wait_Ack(void)
{
	unsigned char tempTime=0;

	IIC_SDA_SETH;
	DELAY_US(1);
	SDA_IN();      //SDA����Ϊ����
	IIC_SCL_SETH;
	DELAY_US(1);
	while(READ_SDA)
	{
		tempTime++;
		if(tempTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_SETL;//ʱ�����0
	return 0;
}

/*******************************************************************************
* �� �� ��         : IIC_Ack
* ��������		   : ����ACKӦ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL_SETL;
	SDA_OUT();
	IIC_SDA_SETL;
	DELAY_US(2);
	IIC_SCL_SETH;
	DELAY_US(5);
	IIC_SCL_SETL;
}

/*******************************************************************************
* �� �� ��         : IIC_NAck
* ��������		   : ����NACK��Ӧ��
* ��    ��         : ��
* ��    ��         : ��
*******************************************************************************/
void IIC_NAck(void)
{
	IIC_SCL_SETL;
	SDA_OUT();
	IIC_SDA_SETH;
	DELAY_US(2);
	IIC_SCL_SETH;
	DELAY_US(5);
	IIC_SCL_SETL;
}

/*******************************************************************************
* �� �� ��         : IIC_Send_Byte
* ��������		   : IIC����һ���ֽ�
* ��    ��         : txd������һ���ֽ�
* ��    ��         : ��
*******************************************************************************/
void IIC_Send_Byte(unsigned char txd)
{
	unsigned char t;
	SDA_OUT();
	IIC_SCL_SETL;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {
        if((txd&0x80)>0) //0x80  1000 0000
        	IIC_SDA_SETH;
		else
			IIC_SDA_SETL;
        txd<<=1;
        DELAY_US(2);   //��TEA5767��������ʱ���Ǳ����
        IIC_SCL_SETH;
		DELAY_US(2);
		IIC_SCL_SETL;
		DELAY_US(2);
    }
}

/*******************************************************************************
* �� �� ��         : IIC_Read_Byte
* ��������		   : IIC��һ���ֽ�
* ��    ��         : ack=1ʱ������ACK��ack=0������nACK
* ��    ��         : Ӧ����Ӧ��
*******************************************************************************/
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
    	IIC_SCL_SETL;
    	DELAY_US(2);
        IIC_SCL_SETH;
        receive<<=1;
        if(READ_SDA)receive++;
        DELAY_US(1);
    }
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK
    return receive;
}


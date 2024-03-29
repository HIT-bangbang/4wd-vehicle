/**			                                                
  * @内  容  MPU6050 操作
  ******************************************************************************
  * @说  明
  *
  * 1.MPU6050三轴加速度传感器、三轴陀螺仪
	* 2.IIC通信采用IO口模拟方式
  *
  ******************************************************************************
  */

#include "mpu6050.h" 
#include "sys.h"
#include "delay.h"

//IO方向设置
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)8<<4;}
#define SDA_OUT() {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)3<<4;}

//IO操作函数	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //输入SDA 

static void IIC_Init(void);

/**
  * @简  述  MPU6050写寄存器。
  * @参  数  无	  
  * @返回值  无
  */
static void MPU6050_WriteRegister(uint8_t reg_address, uint8_t data)
{
	MPU6050_I2C_Write(MPU6050_ADDR,reg_address,1,&data);
}

/**
  * @简  述  初始化三轴加速度和三轴陀螺仪（MPU6050）。
  * @参  数  无	  
  * @返回值  无
  */
static void MPU6050_ReadRegister(uint8_t reg_address, uint8_t *pdata, uint16_t len)
{
	MPU6050_I2C_Read(MPU6050_ADDR,reg_address,len,pdata);
}


/**
  * @简  述  MPU6050传感器初始化
  * @参  数  无	  
  * @返回值  无
  */
void AX_MPU6050_Init(void)
{	
	IIC_Init();	//初始化I2C2
	
	//配置MPU6050寄存器  
  MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x00);     //解除休眠状态

	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,0x18);      //陀螺仪量程 默认2000deg/s
  MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,0x00);     //加速计量程 默认2g	
	
	MPU6050_WriteRegister(MPU6050_INTBP_CFG_REG,0x80);      //INT引脚低电平有效
	
	MPU6050_WriteRegister(MPU6050_PWR_MGMT1_REG,0x01);      //设置CLKSEL,PLL X轴为参考
	MPU6050_WriteRegister(MPU6050_PWR_MGMT2_REG,0x00); 	    //加速度与陀螺仪都工作
	
	Delayms(100);  //等待传感器稳定
}


/**
  * @简  述  MPU6050设置加速度量程 
  * @参  数  range： 加速度量程2g、4g、8g、16g
  *          可设置的加速度量程ACC_RANGE_2G、ACC_RANGE_4G、ACC_RANGE_8G、ACC_RANGE_16G
  * @返回值  无
  */
void AX_MPU6050_SetAccRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_ACCEL_CFG_REG,range<<3);
}


/**
  * @简  述  MPU6050设置陀螺仪量程
  * @参  数  range 陀螺仪量程250°/S、500°/S、1000°/S、2000°/S
  *          可设置的陀螺仪量程GYRO_RANGE_250、GYRO_RANGE_500、GYRO_RANGE_1000、GYRO_RANGE_2000	
  * @返回值  无
  */
void AX_MPU6050_SetGyroRange(uint8_t range)
{
	MPU6050_WriteRegister(MPU6050_GYRO_CFG_REG,range<<3);
}

/**
  * @简  述  MPU6050设置陀螺仪采样率
  * @参  数  smplrate 陀螺仪采样率，范围10~1000Hz	  
  * @返回值  无
  */
void AX_MPU6050_SetGyroSmplRate(uint16_t smplrate)
{	
	if(smplrate>1000)
		smplrate = 1000;
	if(smplrate<10)
		smplrate = 10;
	
	MPU6050_WriteRegister(MPU6050_SAMPLE_RATE_REG,(uint8_t)(1000/smplrate -1));	
}

/**
  * @简  述  MPU6050设置低通滤波器带宽
  * @参  数  bandwidth 低通滤波器带宽
  *          可设置的带宽： DLPF_ACC184_GYRO188、DLPF_ACC94_GYRO98、DLPF_ACC44_GYRO42、
  *                        DLPF_ACC21_GYRO20、DLPF_ACC10_GYRO10、DLPF_ACC5_GYRO5
  * @返回值  无
  */
void AX_MPU6050_SetDLPF(uint8_t bandwidth)
{
	MPU6050_WriteRegister(MPU6050_CFG_REG,bandwidth);
}

/**
  * @简  述  MPU6050获取传感器温度值
  * @参  数  无	  
  * @返回值  传感器温度值。
  */
float AX_MPU6050_GetTempValue(void)
{	
	uint8_t buf[2];
	int16_t tmp;

	MPU6050_ReadRegister(MPU6050_TEMP_OUTH_REG,buf,2);

	tmp = (buf[0]<<8)| buf[1];
	
	return ( 36.53f + ((double)tmp/340.0f) );	
}

/**
  * @简  述  MPU6050获取X轴加速度寄存器输出值
  * @参  数  无	  
  * @返回值  X轴加速度寄存器数据。
  */
int16_t AX_MPU6050_GetAccData_X(void)
{
	uint8_t buf[2];
	
	MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);	
}

/**
  * @简  述  MPU6050获取Y轴加速度寄存器输出值
  * @参  数  无	  
  * @返回值  Y轴加速度寄存器数据。
  */
int16_t AX_MPU6050_GetAccData_Y(void)
{	
	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_ACCEL_YOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);	
}

/**
  * @简  述  MPU6050获取Z轴加速度寄存器输出值
  * @参  数  无	  
  * @返回值  Z轴加速度寄存器数据。
  */
int16_t AX_MPU6050_GetAccData_Z(void)
{	
	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_ACCEL_ZOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);		
}

/**
  * @简  述  MPU6050获取三轴加速度寄存器输出值
  * @参  数  pbuf：读取的数据缓冲区指针 
  * @返回值  无
  */
void AX_MPU6050_GetAccData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	MPU6050_ReadRegister(MPU6050_ACCEL_XOUTH_REG,buf,6);
	
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}


/**
  * @简  述  MPU6050获取X轴陀螺仪寄存器输出值
  * @参  数  无	  
  * @返回值  X轴陀螺仪寄存器数据。
  */
int16_t AX_MPU6050_GetGyroData_X(void)
{

	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_GYRO_XOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);		
}

/**
  * @简  述  MPU6050获取Y轴陀螺仪寄存器输出值
  * @参  数  无	  
  * @返回值  Y轴陀螺仪寄存器数据。
  */
int16_t AX_MPU6050_GetGyroData_Y(void)
{	
	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_GYRO_YOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);
}

/**
  * @简  述  MPU6050获取Z轴陀螺仪寄存器输出值
  * @参  数  无	  
  * @返回值  Z轴陀螺仪寄存器数据。
  */
int16_t AX_MPU6050_GetGyroData_Z(void)
{	
	uint8_t buf[2];

	MPU6050_ReadRegister(MPU6050_GYRO_ZOUTH_REG,buf,2);

	return ((buf[0]<<8) | buf[1]);
}

/**
  * @简  述  MPU6050获取三轴轴陀螺仪寄存器输出值
  * @参  数  pbuf：读取的数据缓冲区指针 	  
  * @返回值  无
  */
void AX_MPU6050_GetGyroData(int16_t *pbuf)
{	
	uint8_t buf[6];
	
	MPU6050_ReadRegister(MPU6050_GYRO_XOUTH_REG,buf,6);
	
    pbuf[0] = (buf[0] << 8) | buf[1];
    pbuf[1] = (buf[2] << 8) | buf[3];
    pbuf[2] = (buf[4] << 8) | buf[5];	
}										


//--------------------------I2C 初始化操作函数-----------------------------------------

/**************************实现函数********************************************
*函数原型:		void IIC_Init(void)
*功　　能:		初始化I2C对应的接口引脚。
*******************************************************************************/
static  void IIC_Init(void)
{			

	GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );	//使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
    IIC_SCL=1;
    IIC_SDA=1;
	
}

/**************************实现函数********************************************
*函数原型:		void IIC_Start(void)
*功　　能:		产生IIC起始信号
*******************************************************************************/
static  int IIC_Start(void)
{
	SDA_OUT();     //sda线输出
	IIC_SDA=1;
	if(!READ_SDA)return 0;	
	IIC_SCL=1;
	Delayus(1);
 	IIC_SDA=0;//START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return 0;
	Delayus(1);
	IIC_SCL=0;//钳住I2C总线，准备发送或接收数据 
	return 1;
}

/**************************实现函数********************************************
*函数原型:		void IIC_Stop(void)
*功　　能:	    //产生IIC停止信号
*******************************************************************************/	  
static  void IIC_Stop(void)
{
	SDA_OUT();//sda线输出
	IIC_SCL=0;
	IIC_SDA=0;//STOP:when CLK is high DATA change form low to high
 	Delayus(1);
	IIC_SCL=1; 
	IIC_SDA=1;//发送I2C总线结束信号
	Delayus(1);							   	
}

/**************************实现函数********************************************
*函数原型:		u8 IIC_Wait_Ack(void)
*功　　能:	    等待应答信号到来 
//返回值：1，接收应答失败
//        0，接收应答成功
*******************************************************************************/
static  int IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  
	IIC_SDA=1;
	Delayus(1);	   
	IIC_SCL=1;
	Delayus(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			IIC_Stop();
			return 0;
		}
	  Delayus(1);
	}
	IIC_SCL=0;//时钟输出0 	   
	return 1;  
} 

/**************************实现函数********************************************
*函数原型:		void IIC_Ack(void)
*功　　能:	    产生ACK应答
*******************************************************************************/
static  void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	Delayus(1);
	IIC_SCL=1;
	Delayus(1);
	IIC_SCL=0;
}
	
/**************************实现函数********************************************
*函数原型:		void IIC_NAck(void)
*功　　能:	    产生NACK应答
*******************************************************************************/	    
static  void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	Delayus(1);
	IIC_SCL=1;
	Delayus(1);
	IIC_SCL=0;
}
/**************************实现函数********************************************
*函数原型:		void IIC_Send_Byte(u8 txd)
*功　　能:	    IIC发送一个字节
*******************************************************************************/		  
static  void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		Delayus(1);   
		IIC_SCL=1;
		Delayus(1); 
		IIC_SCL=0;	
		Delayus(1);
    }	 
} 	 
  
/**************************实现函数********************************************
*函数原型:		u8 IIC_Read_Byte(unsigned char ack)
*功　　能:	    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK 
*******************************************************************************/  
static  u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        Delayus(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		Delayus(2); 
    }					 
    if (ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck();//发送nACK  
    return receive;
}

/**************************实现函数********************************************
*函数原型:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:		
*******************************************************************************/
uint8_t MPU6050_I2C_Write(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, const uint8_t *data)
{
		int i;
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(dev_addr << 1 );
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
		for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (!IIC_Wait_Ack()) {
            IIC_Stop();
            return 0;
        }
    }
    IIC_Stop();
    return 0;
}
/**************************实现函数********************************************
*函数原型:		bool i2cWrite(uint8_t addr, uint8_t reg, uint8_t data)
*功　　能:		
*******************************************************************************/
uint8_t MPU6050_I2C_Read(uint8_t dev_addr, uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    if (!IIC_Start())
        return 1;
    IIC_Send_Byte(dev_addr << 1);
    if (!IIC_Wait_Ack()) {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg_addr);
    IIC_Wait_Ack();
    IIC_Start();
    IIC_Send_Byte((dev_addr << 1)+1);
    IIC_Wait_Ack();
    while (len) {
        if (len == 1)
            *data = IIC_Read_Byte(0);
        else
            *data = IIC_Read_Byte(1);
        data++;
        len--;
    }
    IIC_Stop();
    return 0;
}


/******************* (C) 版权 2018 XTARK **************************************/

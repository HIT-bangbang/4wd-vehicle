/**
  ******************************************************************************
  * @file    main.c
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart.h"

#include <stdio.h>
#include <math.h>

#include "sys.h"    //系统设置
#include "delay.h"  //软件延时
#include "motor.h"    //直流电机调速控制
#include "encoder.h"  //编码器控制
#include "servo.h"    //舵机控制
#include "mpu6050.h"  //IMU加速度陀螺仪测量
#include "mpu6050_dmp.h"  //DMP功能函数
#include "pid.h"        //PID控制
#include "kinematics.h" //运动学解析
#include "tim.h"      //定时器

#define ENCODER_MID_VALUE  30000  //编码器中间计数值

int16_t ax_encoder[4];	//编码器累加值
int16_t ax_encoder_delta[4];	//编码器变化值
int16_t ax_encoder_delta_target[4] = {0};  //编码器目标变化值
int16_t robot_odom[6] = {0}; //里程计数据，绝对值和变化值，x y yaw dx dy dyaw
int16_t ax_motor_pwm[2];  //电机PWM
uint16_t ax_bat_vol;  //电池电压
int16_t mpu_data[10];  //陀螺仪，加速度，姿态角

int16_t robot_target_speed[3] = {0};  //机器人目标速度 X Y Yaw   m/s*1000
int16_t robot_params[2] = {1000,0};  //机器人参数


//主要函数声明
void GetImuData(void);  //读取MPU6050数据
void MoveCtl(void);  //机器人运动控制函数  
void SendData(void);  //机器人发送数据到树莓派



int main(void)
{	
	uint8_t cnt = 1;  //定时器计数
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //设置中断分组
	
	//机器人初始化
	DELAY_Init();  
	AX_SERVO_S12_Init();	
	AX_MOTOR_Init(10);
	AX_JTAG_Set(JTAG_SWD_DISABLE);     
	AX_JTAG_Set(SWD_ENABLE);
	
	USART1_Config();
	USART2_Config();
	
	AX_ENCODER_Init(ENCODER_MID_VALUE*2);    //初始化两路编码
	AX_ENCODER_A_SetCounter(ENCODER_MID_VALUE); 
	AX_ENCODER_B_SetCounter(ENCODER_MID_VALUE); 
	
	
	//模块初始化及配置
	AX_MPU6050_Init();    //MPU6050初始化  
	AX_MPU6050_DMP_Init();	//DMP初始化
	TIM6_Init(10000);  //启动定时器中断，定时时长10ms

  while(1)
	{	
		//100HZ控制频率
		if(TIM6_CheckIrqStatus())
		{		
			//机器人获取PMU6050数据
			GetImuData();
			
			//50HZ执行频率
      if(cnt%2 == 0)
			{
				//机器人运动控制
				MoveCtl();
	//			printf("当前角度为：%d\n",mpu_data[6]);
				//机器人发送数据到树莓派
				SendData();
			}
			
			//计数器累加
			cnt++;
		}
	}
			
}

/**
  * @简  述  机器人获取MPU6050 加速度陀螺仪姿态数据
  * @参  数  无
  * @返回值  无
  */
void GetImuData(void)
{
	  //变量定义，舵机角度
    AX_MPU6050_DMP_GetData(mpu_data);
}


/**
  * @简  述  机器人运动控制函数
  * @参  数  无
  * @返回值  无
  */
void MoveCtl(void)
{
	  //变量定义，舵机角度
		static int16_t servo;  
	  
	  //获取编码器变化值
		ax_encoder_delta[0] = (AX_ENCODER_A_GetCounter()-ENCODER_MID_VALUE);
		ax_encoder_delta[1] = -(AX_ENCODER_B_GetCounter()-ENCODER_MID_VALUE);
		printf("编码器1变化值:%d \r\n",ax_encoder_delta[0]);
		//printf("编码器2变化值:%d \r\n",ax_encoder_delta[1]);
	
		printf("A轮子速度：%f \r\n",(float)ax_encoder_delta[0]/780/0.02);
		//printf("B轮子速度：%f\r\n",(float)ax_encoder_delta[1]/780/0.02);


	  //编码器重置为中间值
		AX_ENCODER_A_SetCounter(ENCODER_MID_VALUE); 
		AX_ENCODER_B_SetCounter(ENCODER_MID_VALUE); 
		
	  //计算编码器累加值
		ax_encoder[0] = ax_encoder[0] - ax_encoder_delta[0];
		ax_encoder[1] = ax_encoder[1] - ax_encoder_delta[1];
		
	  //运动学解析
		AX_Kinematics_Forward(ax_encoder,robot_odom);  //正向运动学解析，编码器——>里程计
		AX_Kinematics_Inverse(robot_target_speed, ax_encoder_delta_target);  //逆向运动学解析，线角速度——>编码器
//		printf("目标线速度:%d \r\n",robot_target_speed[0]);
		printf("目标编码器1:%d \r\n",ax_encoder_delta_target[0]);
//		printf("目标编码器2:%d \r\n",ax_encoder_delta_target[1]);

		
	  //电机PID速度控制
		ax_motor_pwm[0] = AX_PID_MotorVelocityCtlA(ax_encoder_delta_target[0], ax_encoder_delta[0]);   
		ax_motor_pwm[1] = AX_PID_MotorVelocityCtlB(ax_encoder_delta_target[1], ax_encoder_delta[1]);   
		AX_MOTOR_A_SetSpeed(-ax_motor_pwm[0]);                 
		AX_MOTOR_B_SetSpeed(-ax_motor_pwm[1]);  
		printf("设置速度1：%d\r\n",ax_motor_pwm[0]);
//		printf("设置pwm1：%d\r\n",ax_motor_pwm[1]);

	  //舵机角度控制
		servo = 900+ax_encoder_delta_target[2];
		AX_SERVO_S1_SetAngle(servo);
}

/**
  * @简  述  机器人发送数据到树莓派
  * @参  数  无
  * @返回值  无
  */
void SendData(void)
{
	    //串口发送数据
			static uint8_t comdata[41]; 			
	
			//陀螺仪角速度 = (ax_gyro/32768) * 2000 ?s
			comdata[0] = (u8)( mpu_data[0] >> 8 );  
			comdata[1] = (u8)( mpu_data[0] );
			comdata[2] = (u8)( mpu_data[1] >> 8 );
			comdata[3] = (u8)( mpu_data[1] );
			comdata[4] = (u8)( mpu_data[2] >> 8 );
			comdata[5] = (u8)( mpu_data[2] );
			
			//加速度 = (ax_acc/32768) * 2G  
			comdata[6] = (u8)( mpu_data[3] >> 8 );
			comdata[7] = (u8)( mpu_data[3] );
			comdata[8] = (u8)( mpu_data[4] >> 8 );
			comdata[9] = (u8)( mpu_data[4] );
			comdata[10] = (u8)( mpu_data[5] >> 8 );
			comdata[11] = (u8)( mpu_data[5] );
			
			//姿态角度 = (ax_angle/100)
			comdata[12] = (u8)( mpu_data[6] >> 8 ); 
			comdata[13] = (u8)( mpu_data[6] );
			comdata[14] = (u8)( mpu_data[7] >> 8 );
			comdata[15] = (u8)( mpu_data[7] );
			comdata[16] = (u8)( mpu_data[8] >> 8 );
			comdata[17] = (u8)( mpu_data[8] );
			
			//里程计坐标 x(m) y(m) yaw(rad)  odom_frame
			comdata[18] = (u8)( robot_odom[0] >> 8 );
			comdata[19] = (u8)( robot_odom[0] );
			comdata[20] = (u8)( robot_odom[1] >> 8 );
			comdata[21] = (u8)( robot_odom[1] );
			comdata[22] = (u8)( robot_odom[2] >> 8 );
			comdata[23] = (u8)( robot_odom[2] );
			
			//里程计坐标变化量  d_x(m) d_y(m) d_yaw(rad)  base_frame
			comdata[24] = (u8)( robot_odom[3] >> 8 );
			comdata[25] = (u8)( robot_odom[3] );
			comdata[26] = (u8)( robot_odom[4] >> 8 );
			comdata[27] = (u8)( robot_odom[4] );
			comdata[28] = (u8)( robot_odom[5] >> 8 );
			comdata[29] = (u8)( robot_odom[5] );
			//printf("x速度:%d \r\n",robot_odom[3]);

		
		  //编码器当前值和目标值
			comdata[30] = (u8)( ax_encoder_delta[0] >> 8 );  
			comdata[31] = (u8)( ax_encoder_delta[0] );
			comdata[32] = (u8)( ax_encoder_delta[1] >> 8 );
			comdata[33] = (u8)( ax_encoder_delta[1] );
			comdata[34] = (u8)( ax_encoder_delta_target[0] >> 8 );  
			comdata[35] = (u8)( ax_encoder_delta_target[0] );
			comdata[36] = (u8)( ax_encoder_delta_target[1] >> 8 );
			comdata[37] = (u8)( ax_encoder_delta_target[1] );
			
			//编码器
//			ax_bat_vol = -160;
			comdata[38] = (u8)( ax_bat_vol >> 8 );
			comdata[39] = (u8)( ax_bat_vol );
				
			//发送串口数据
			UART_SendPacket(comdata, 40, 0x06);
}


/*********************************************END OF FILE**********************/

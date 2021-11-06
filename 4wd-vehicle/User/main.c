/**
  ******************************************************************************
  * @file    main.c
  ******************************************************************************
  */ 
 
 
#include "stm32f10x.h"
#include "bsp_usart.h"

#include <stdio.h>
#include <math.h>

#include "sys.h"    //ϵͳ����
#include "delay.h"  //�����ʱ
#include "motor.h"    //ֱ��������ٿ���
#include "encoder.h"  //����������
#include "servo.h"    //�������
#include "mpu6050.h"  //IMU���ٶ������ǲ���
#include "mpu6050_dmp.h"  //DMP���ܺ���
#include "pid.h"        //PID����
#include "kinematics.h" //�˶�ѧ����
#include "tim.h"      //��ʱ��

#define ENCODER_MID_VALUE  30000  //�������м����ֵ

int16_t ax_encoder[4];	//�������ۼ�ֵ
int16_t ax_encoder_delta[4];	//�������仯ֵ
int16_t ax_encoder_delta_target[4] = {0};  //������Ŀ��仯ֵ
int16_t robot_odom[6] = {0}; //��̼����ݣ�����ֵ�ͱ仯ֵ��x y yaw dx dy dyaw
int16_t ax_motor_pwm[2];  //���PWM
uint16_t ax_bat_vol;  //��ص�ѹ
int16_t mpu_data[10];  //�����ǣ����ٶȣ���̬��

int16_t robot_target_speed[3] = {0};  //������Ŀ���ٶ� X Y Yaw   m/s*1000
int16_t robot_params[2] = {1000,0};  //�����˲���


//��Ҫ��������
void GetImuData(void);  //��ȡMPU6050����
void MoveCtl(void);  //�������˶����ƺ���  
void SendData(void);  //�����˷������ݵ���ݮ��



int main(void)
{	
	uint8_t cnt = 1;  //��ʱ������
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //�����жϷ���
	
	//�����˳�ʼ��
	DELAY_Init();  
	AX_SERVO_S12_Init();	
	AX_MOTOR_Init(10);
	AX_JTAG_Set(JTAG_SWD_DISABLE);     
	AX_JTAG_Set(SWD_ENABLE);
	
	USART1_Config();
	USART2_Config();
	
	AX_ENCODER_Init(ENCODER_MID_VALUE*2);    //��ʼ����·����
	AX_ENCODER_A_SetCounter(ENCODER_MID_VALUE); 
	AX_ENCODER_B_SetCounter(ENCODER_MID_VALUE); 
	
	
	//ģ���ʼ��������
	AX_MPU6050_Init();    //MPU6050��ʼ��  
	AX_MPU6050_DMP_Init();	//DMP��ʼ��
	TIM6_Init(10000);  //������ʱ���жϣ���ʱʱ��10ms

  while(1)
	{	
		//100HZ����Ƶ��
		if(TIM6_CheckIrqStatus())
		{		
			//�����˻�ȡPMU6050����
			GetImuData();
			
			//50HZִ��Ƶ��
      if(cnt%2 == 0)
			{
				//�������˶�����
				MoveCtl();
	//			printf("��ǰ�Ƕ�Ϊ��%d\n",mpu_data[6]);
				//�����˷������ݵ���ݮ��
				SendData();
			}
			
			//�������ۼ�
			cnt++;
		}
	}
			
}

/**
  * @��  ��  �����˻�ȡMPU6050 ���ٶ���������̬����
  * @��  ��  ��
  * @����ֵ  ��
  */
void GetImuData(void)
{
	  //�������壬����Ƕ�
    AX_MPU6050_DMP_GetData(mpu_data);
}


/**
  * @��  ��  �������˶����ƺ���
  * @��  ��  ��
  * @����ֵ  ��
  */
void MoveCtl(void)
{
	  //�������壬����Ƕ�
		static int16_t servo;  
	  
	  //��ȡ�������仯ֵ
		ax_encoder_delta[0] = (AX_ENCODER_A_GetCounter()-ENCODER_MID_VALUE);
		ax_encoder_delta[1] = -(AX_ENCODER_B_GetCounter()-ENCODER_MID_VALUE);
		printf("������1�仯ֵ:%d \r\n",ax_encoder_delta[0]);
		//printf("������2�仯ֵ:%d \r\n",ax_encoder_delta[1]);
	
		printf("A�����ٶȣ�%f \r\n",(float)ax_encoder_delta[0]/780/0.02);
		//printf("B�����ٶȣ�%f\r\n",(float)ax_encoder_delta[1]/780/0.02);


	  //����������Ϊ�м�ֵ
		AX_ENCODER_A_SetCounter(ENCODER_MID_VALUE); 
		AX_ENCODER_B_SetCounter(ENCODER_MID_VALUE); 
		
	  //����������ۼ�ֵ
		ax_encoder[0] = ax_encoder[0] - ax_encoder_delta[0];
		ax_encoder[1] = ax_encoder[1] - ax_encoder_delta[1];
		
	  //�˶�ѧ����
		AX_Kinematics_Forward(ax_encoder,robot_odom);  //�����˶�ѧ����������������>��̼�
		AX_Kinematics_Inverse(robot_target_speed, ax_encoder_delta_target);  //�����˶�ѧ�������߽��ٶȡ���>������
//		printf("Ŀ�����ٶ�:%d \r\n",robot_target_speed[0]);
		printf("Ŀ�������1:%d \r\n",ax_encoder_delta_target[0]);
//		printf("Ŀ�������2:%d \r\n",ax_encoder_delta_target[1]);

		
	  //���PID�ٶȿ���
		ax_motor_pwm[0] = AX_PID_MotorVelocityCtlA(ax_encoder_delta_target[0], ax_encoder_delta[0]);   
		ax_motor_pwm[1] = AX_PID_MotorVelocityCtlB(ax_encoder_delta_target[1], ax_encoder_delta[1]);   
		AX_MOTOR_A_SetSpeed(-ax_motor_pwm[0]);                 
		AX_MOTOR_B_SetSpeed(-ax_motor_pwm[1]);  
		printf("�����ٶ�1��%d\r\n",ax_motor_pwm[0]);
//		printf("����pwm1��%d\r\n",ax_motor_pwm[1]);

	  //����Ƕȿ���
		servo = 900+ax_encoder_delta_target[2];
		AX_SERVO_S1_SetAngle(servo);
}

/**
  * @��  ��  �����˷������ݵ���ݮ��
  * @��  ��  ��
  * @����ֵ  ��
  */
void SendData(void)
{
	    //���ڷ�������
			static uint8_t comdata[41]; 			
	
			//�����ǽ��ٶ� = (ax_gyro/32768) * 2000 ?s
			comdata[0] = (u8)( mpu_data[0] >> 8 );  
			comdata[1] = (u8)( mpu_data[0] );
			comdata[2] = (u8)( mpu_data[1] >> 8 );
			comdata[3] = (u8)( mpu_data[1] );
			comdata[4] = (u8)( mpu_data[2] >> 8 );
			comdata[5] = (u8)( mpu_data[2] );
			
			//���ٶ� = (ax_acc/32768) * 2G  
			comdata[6] = (u8)( mpu_data[3] >> 8 );
			comdata[7] = (u8)( mpu_data[3] );
			comdata[8] = (u8)( mpu_data[4] >> 8 );
			comdata[9] = (u8)( mpu_data[4] );
			comdata[10] = (u8)( mpu_data[5] >> 8 );
			comdata[11] = (u8)( mpu_data[5] );
			
			//��̬�Ƕ� = (ax_angle/100)
			comdata[12] = (u8)( mpu_data[6] >> 8 ); 
			comdata[13] = (u8)( mpu_data[6] );
			comdata[14] = (u8)( mpu_data[7] >> 8 );
			comdata[15] = (u8)( mpu_data[7] );
			comdata[16] = (u8)( mpu_data[8] >> 8 );
			comdata[17] = (u8)( mpu_data[8] );
			
			//��̼����� x(m) y(m) yaw(rad)  odom_frame
			comdata[18] = (u8)( robot_odom[0] >> 8 );
			comdata[19] = (u8)( robot_odom[0] );
			comdata[20] = (u8)( robot_odom[1] >> 8 );
			comdata[21] = (u8)( robot_odom[1] );
			comdata[22] = (u8)( robot_odom[2] >> 8 );
			comdata[23] = (u8)( robot_odom[2] );
			
			//��̼�����仯��  d_x(m) d_y(m) d_yaw(rad)  base_frame
			comdata[24] = (u8)( robot_odom[3] >> 8 );
			comdata[25] = (u8)( robot_odom[3] );
			comdata[26] = (u8)( robot_odom[4] >> 8 );
			comdata[27] = (u8)( robot_odom[4] );
			comdata[28] = (u8)( robot_odom[5] >> 8 );
			comdata[29] = (u8)( robot_odom[5] );
			//printf("x�ٶ�:%d \r\n",robot_odom[3]);

		
		  //��������ǰֵ��Ŀ��ֵ
			comdata[30] = (u8)( ax_encoder_delta[0] >> 8 );  
			comdata[31] = (u8)( ax_encoder_delta[0] );
			comdata[32] = (u8)( ax_encoder_delta[1] >> 8 );
			comdata[33] = (u8)( ax_encoder_delta[1] );
			comdata[34] = (u8)( ax_encoder_delta_target[0] >> 8 );  
			comdata[35] = (u8)( ax_encoder_delta_target[0] );
			comdata[36] = (u8)( ax_encoder_delta_target[1] >> 8 );
			comdata[37] = (u8)( ax_encoder_delta_target[1] );
			
			//������
//			ax_bat_vol = -160;
			comdata[38] = (u8)( ax_bat_vol >> 8 );
			comdata[39] = (u8)( ax_bat_vol );
				
			//���ʹ�������
			UART_SendPacket(comdata, 40, 0x06);
}


/*********************************************END OF FILE**********************/

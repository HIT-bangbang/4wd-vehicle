/**			                                                    

  ******************************************************************************

  * @��  ��  ���PID����
  *
  ******************************************************************************
  */
#include "pid.h"
//#include "delay.h"
#include "bsp_usart.h"

#define PID_SCALE  1  //PID����ϵ��
#define PID_INTEGRAL_UP 200000  //��������

int16_t ax_motor_kp=30;  //���ת��PID-P
int16_t ax_motor_ki=1;    //���ת��PID-I
int16_t ax_motor_kd=20;  //���ת��PID-D

/**
  * @��  ��  ���A PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ ,��Χ����250��
  *          spd_current: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current)
{
	printf("pid: %d-%d-%d",ax_motor_kp,ax_motor_ki,ax_motor_kd);
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral ;

	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//����ƫ���ۼ�ֵ
	bias_integral += bias;
	
	//�����ֱ���
	if(bias_integral>PID_INTEGRAL_UP) bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP) bias_integral = -PID_INTEGRAL_UP;
	
	//PID���������PWMֵ
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
  
	//����PWM����ֵ
	return motor_pwm_out;
}	

/**
  * @��  ��  ���B PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//����ƫ���ۼ�ֵ
	bias_integral += bias;
	
  //�����ֱ���
	if(bias_integral>PID_INTEGRAL_UP) bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP) bias_integral = -PID_INTEGRAL_UP;
	
	//PID���������PWMֵ
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;

	//����PWM����ֵ
	return motor_pwm_out;
}



/******************* (C) ��Ȩ 2019 XTARK **************************************/

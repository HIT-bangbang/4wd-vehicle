/**			                                                    

  ******************************************************************************

  * @内  容  电机PID控制
  *
  ******************************************************************************
  */
#include "pid.h"
//#include "delay.h"
#include "bsp_usart.h"

#define PID_SCALE  1  //PID缩放系数
#define PID_INTEGRAL_UP 200000  //积分上限

int16_t ax_motor_kp=30;  //电机转速PID-P
int16_t ax_motor_ki=1;    //电机转速PID-I
int16_t ax_motor_kd=20;  //电机转速PID-D

/**
  * @简  述  电机A PID控制函数
  * @参  数  spd_target:编码器速度目标值 ,范围（±250）
  *          spd_current: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current)
{
	printf("pid: %d-%d-%d",ax_motor_kp,ax_motor_ki,ax_motor_kd);
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral ;

	//获得偏差值
	bias = spd_target - spd_current;
	
	//计算偏差累加值
	bias_integral += bias;
	
	//抗积分饱和
	if(bias_integral>PID_INTEGRAL_UP) bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP) bias_integral = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
  
	//返回PWM控制值
	return motor_pwm_out;
}	

/**
  * @简  述  电机B PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//获得偏差值
	bias = spd_target - spd_current;
	
	//计算偏差累加值
	bias_integral += bias;
	
  //抗积分饱和
	if(bias_integral>PID_INTEGRAL_UP) bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP) bias_integral = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;

	//返回PWM控制值
	return motor_pwm_out;
}



/******************* (C) 版权 2019 XTARK **************************************/

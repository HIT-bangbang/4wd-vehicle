
//机器人运动学解析

#include "kinematics.h"
#include <stdio.h>
#include <math.h>


//变量定义
int32_t  current_count[2] = {0};
float    ticks_per_meter = 3548;//轮子每转一圈前进的距离，注意是轮子，即电机出轴而不是编码器转一圈。这个数据在串口通讯中断中（中断函数调用AX_Kinematics_Init）会被修改。
double   linear_correction_factor = 1.0;
int32_t  wheel_mult[2] = {0};
int16_t  servo_bias = 0;

extern float robot_linear_acc;
extern float robot_angular_acc;
extern int16_t robot_odom[6];
extern int16_t robot_target_speed[3];

/**
  * @简  述  机器人运动参数设置
  * @参  数  无
  * @返回值  无
  */
void AX_Kinematics_Init(int16_t* robot_params)
{
	linear_correction_factor = (float)robot_params[0]/1000;
    servo_bias = -robot_params[1];
 
	robot_odom[0]  = 0;
	robot_odom[1]  = 0;
	robot_odom[2]  = 0;

	ticks_per_meter    = (float)ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926*linear_correction_factor);		
}

/**
  * @简  述  逆向运动学解析，底盘三轴速度->轮子速度
  * @参  数  input:  robot_target_speed[]  机器人三轴速度 单位为：m/s*1000
  *          output：ax_encoder_delta_target[] 目前应该达到的编码器数值和舵机角度值
  * @返回值  无
  */
void AX_Kinematics_Inverse(int16_t* input, int16_t* output)
{
	float x_speed   = ((float)input[0])/1000;
	float turn_angular = -((float)input[2])/1000;
	static float wheel_velocity[2] = {0};
	static float servo_angle = 0;
	
	//计算舵机转角，弧度制转换为角度制
	servo_angle = (turn_angular*180/3.1415);
  
	//舵机转角限幅
	if(servo_angle>SERVO_MAX)
		servo_angle = SERVO_MAX;
	else if(servo_angle < -SERVO_MAX)
		servo_angle = -SERVO_MAX;
	
	//线速度限幅
	if(x_speed > SPEED_MAX)
		x_speed = SPEED_MAX;
	else if(x_speed < -SPEED_MAX)
		x_speed = -SPEED_MAX;
	
	//计算后轮电子差速，得出后轮的两轮速度，单位m/s
	wheel_velocity[0] = x_speed*(1+ACKMAN_WHEEL_TRACK*tan((float)servo_angle/180*3.1415)/2/ACKMAN_WHEEL_DISTANCE);
	wheel_velocity[1] = x_speed*(1-ACKMAN_WHEEL_TRACK*tan((float)servo_angle/180*3.1415)/2/ACKMAN_WHEEL_DISTANCE);
	
	//转化为实际电机/舵机控制值，该值为目前应该达到的编码器数值
	output[0] = (int16_t)(wheel_velocity[0] * ticks_per_meter/PID_RATE);
	output[1] = (int16_t)(wheel_velocity[1] * ticks_per_meter/PID_RATE);
	output[2] = servo_angle*10 - servo_bias;
}

/**
  * @简  述  正向运动学解析，轮子编码值->底盘三轴里程计坐标
  * @参  数  input: ax_encoder[]  编码器累加值
  *          output: robot_odom[] 里程计以及三轴速度 x y yaw  Vx  Vy缺省  Ωz
  * @返回值  无
  */
void AX_Kinematics_Forward(int16_t* input, int16_t* output)
{
		static double delta_count[2];  
		static double delta_v_ave[3];
		static int16_t recv_count[2];
	
		recv_count[0] = -input[0];
		recv_count[1] = -input[1];
	
		//编码器计数溢出处理
		//编码器累加值(input)有可能在转过很多圈之后得到溢出int32_t的数据(65536)
		//后续改为中断的方式记录圈数？
		for(int i=0;i<2;i++)
		{
			if(recv_count[i] < ENCODER_LOW_WRAP && current_count[i] > ENCODER_HIGH_WRAP)
				wheel_mult[i]++;
			else if(recv_count[i] > ENCODER_HIGH_WRAP && current_count[i] < ENCODER_LOW_WRAP)
				wheel_mult[i]--;
			else
				wheel_mult[i]=0;
		}
		
		//将编码器数值转化为左右两轮分别前进的距离，单位m
		for(int i=0;i<2;i++)
		{	
			delta_count[i] = (double)(recv_count[i] + wheel_mult[i]*(ENCODER_MAX-ENCODER_MIN)-current_count[i])/ticks_per_meter;
			current_count[i] = recv_count[i];
		}
		
		//计算底盘x轴变化距离m与Yaw轴朝向变化rad
		delta_v_ave[0] = (delta_count[0]+delta_count[1])/2.0;  
		delta_v_ave[1] = delta_v_ave[0]*tan(ANGLE_CAL((double)(TIM2->CCR1+(int16_t)(servo_bias*1.111)))) / ACKMAN_WHEEL_DISTANCE;
		
		//积分计算里程计坐标系(odom_frame)下的机器人X,Y,Yaw轴坐标，单位m*1000
		output[0] += (int16_t)((delta_v_ave[0]*cos(((double)output[2]/1000))*1000));
		output[1] += (int16_t)((delta_v_ave[0]*sin(((double)output[2]/1000))*1000));
		output[2] += (int16_t)(delta_v_ave[1]*1000);
		
    //Yaw轴坐标变化范围控制-2Π -> 2Π
		if(output[2] > PI*1000)
			output[2] -= 2*PI*1000;
		else if(output[2] < -PI*1000)
			output[2] += 2*PI*1000;
		
		//发送机器人X轴Yaw轴速度反馈,单位（m/控制周期）*1000
		output[3] = (int16_t)(delta_v_ave[0]*1000);
		output[4] = 0;
		output[5] = (int16_t)(delta_v_ave[1]*1000);
}


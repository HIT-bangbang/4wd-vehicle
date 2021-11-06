
//�������˶�ѧ����

#include "kinematics.h"
#include <stdio.h>
#include <math.h>


//��������
int32_t  current_count[2] = {0};
float    ticks_per_meter = 3548;//����ÿתһȦǰ���ľ��룬ע�������ӣ��������������Ǳ�����תһȦ����������ڴ���ͨѶ�ж��У��жϺ�������AX_Kinematics_Init���ᱻ�޸ġ�
double   linear_correction_factor = 1.0;
int32_t  wheel_mult[2] = {0};
int16_t  servo_bias = 0;

extern float robot_linear_acc;
extern float robot_angular_acc;
extern int16_t robot_odom[6];
extern int16_t robot_target_speed[3];

/**
  * @��  ��  �������˶���������
  * @��  ��  ��
  * @����ֵ  ��
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
  * @��  ��  �����˶�ѧ���������������ٶ�->�����ٶ�
  * @��  ��  input:  robot_target_speed[]  �����������ٶ� ��λΪ��m/s*1000
  *          output��ax_encoder_delta_target[] ĿǰӦ�ôﵽ�ı�������ֵ�Ͷ���Ƕ�ֵ
  * @����ֵ  ��
  */
void AX_Kinematics_Inverse(int16_t* input, int16_t* output)
{
	float x_speed   = ((float)input[0])/1000;
	float turn_angular = -((float)input[2])/1000;
	static float wheel_velocity[2] = {0};
	static float servo_angle = 0;
	
	//������ת�ǣ�������ת��Ϊ�Ƕ���
	servo_angle = (turn_angular*180/3.1415);
  
	//���ת���޷�
	if(servo_angle>SERVO_MAX)
		servo_angle = SERVO_MAX;
	else if(servo_angle < -SERVO_MAX)
		servo_angle = -SERVO_MAX;
	
	//���ٶ��޷�
	if(x_speed > SPEED_MAX)
		x_speed = SPEED_MAX;
	else if(x_speed < -SPEED_MAX)
		x_speed = -SPEED_MAX;
	
	//������ֵ��Ӳ��٣��ó����ֵ������ٶȣ���λm/s
	wheel_velocity[0] = x_speed*(1+ACKMAN_WHEEL_TRACK*tan((float)servo_angle/180*3.1415)/2/ACKMAN_WHEEL_DISTANCE);
	wheel_velocity[1] = x_speed*(1-ACKMAN_WHEEL_TRACK*tan((float)servo_angle/180*3.1415)/2/ACKMAN_WHEEL_DISTANCE);
	
	//ת��Ϊʵ�ʵ��/�������ֵ����ֵΪĿǰӦ�ôﵽ�ı�������ֵ
	output[0] = (int16_t)(wheel_velocity[0] * ticks_per_meter/PID_RATE);
	output[1] = (int16_t)(wheel_velocity[1] * ticks_per_meter/PID_RATE);
	output[2] = servo_angle*10 - servo_bias;
}

/**
  * @��  ��  �����˶�ѧ���������ӱ���ֵ->����������̼�����
  * @��  ��  input: ax_encoder[]  �������ۼ�ֵ
  *          output: robot_odom[] ��̼��Լ������ٶ� x y yaw  Vx  Vyȱʡ  ��z
  * @����ֵ  ��
  */
void AX_Kinematics_Forward(int16_t* input, int16_t* output)
{
		static double delta_count[2];  
		static double delta_v_ave[3];
		static int16_t recv_count[2];
	
		recv_count[0] = -input[0];
		recv_count[1] = -input[1];
	
		//�����������������
		//�������ۼ�ֵ(input)�п�����ת���ܶ�Ȧ֮��õ����int32_t������(65536)
		//������Ϊ�жϵķ�ʽ��¼Ȧ����
		for(int i=0;i<2;i++)
		{
			if(recv_count[i] < ENCODER_LOW_WRAP && current_count[i] > ENCODER_HIGH_WRAP)
				wheel_mult[i]++;
			else if(recv_count[i] > ENCODER_HIGH_WRAP && current_count[i] < ENCODER_LOW_WRAP)
				wheel_mult[i]--;
			else
				wheel_mult[i]=0;
		}
		
		//����������ֵת��Ϊ�������ֱַ�ǰ���ľ��룬��λm
		for(int i=0;i<2;i++)
		{	
			delta_count[i] = (double)(recv_count[i] + wheel_mult[i]*(ENCODER_MAX-ENCODER_MIN)-current_count[i])/ticks_per_meter;
			current_count[i] = recv_count[i];
		}
		
		//�������x��仯����m��Yaw�ᳯ��仯rad
		delta_v_ave[0] = (delta_count[0]+delta_count[1])/2.0;  
		delta_v_ave[1] = delta_v_ave[0]*tan(ANGLE_CAL((double)(TIM2->CCR1+(int16_t)(servo_bias*1.111)))) / ACKMAN_WHEEL_DISTANCE;
		
		//���ּ�����̼�����ϵ(odom_frame)�µĻ�����X,Y,Yaw�����꣬��λm*1000
		output[0] += (int16_t)((delta_v_ave[0]*cos(((double)output[2]/1000))*1000));
		output[1] += (int16_t)((delta_v_ave[0]*sin(((double)output[2]/1000))*1000));
		output[2] += (int16_t)(delta_v_ave[1]*1000);
		
    //Yaw������仯��Χ����-2�� -> 2��
		if(output[2] > PI*1000)
			output[2] -= 2*PI*1000;
		else if(output[2] < -PI*1000)
			output[2] += 2*PI*1000;
		
		//���ͻ�����X��Yaw���ٶȷ���,��λ��m/�������ڣ�*1000
		output[3] = (int16_t)(delta_v_ave[0]*1000);
		output[4] = 0;
		output[5] = (int16_t)(delta_v_ave[1]*1000);
}


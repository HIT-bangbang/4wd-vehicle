/**			                                                    

  ******************************************************************************
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  2019-7-26
  * @��  ��  �������˶�ѧ����
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _KINEMATICS_H
#define _KINEMATICS_H


/* Includes ------------------------------------------------------------------*/	

#include "stm32f10x.h"

#define ENCODER_MAX 32767
#define ENCODER_MIN -32768 
#define SERVO_MAX   35           //rad �������ת�� 50��
#define SPEED_MAX   2            //m/s ���ٶ����   2m/s
#define ACKMAN_WHEEL_DISTANCE   0.342   //������ǰ���־�
#define ACKMAN_WHEEL_TRACK      0.165   //�����������־�
#define WHEEL_DIAMETER          0.07   //�������־�
#define ENCODER_RESOLUTION      780     //�������ֱ��ʣ�ע�����ı�Ƶ֮���
#define PID_RATE                50      //PIDƵ�ʣ����ֵ�Ϳ���Ƶ�ʶ�Ӧ
#define ROBOT_LINEAR_SPEED_LIMIT 5000   //���������ٶ���ֵ m/s*1000
#define ROBOT_ANGULAR_SPEED_LIMIT 5000  //�����˽��ٶ���ֵ rad/s*1000

#define ENCODER_LOW_WRAP  ((ENCODER_MAX - ENCODER_MIN)*0.3+ENCODER_MIN)
#define ENCODER_HIGH_WRAP ((ENCODER_MAX - ENCODER_MIN)*0.7+ENCODER_MIN)
#define PI 3.1415926

#define ANGLE_CAL(X) (PI-((X)-1500)/2000*PI)

//ROBOT���ܺ���
void AX_Kinematics_Init(int16_t* robot_params);  //������ʼ��
void AX_Kinematics_Forward(int16_t* input, int16_t* output); //����(ForwardKinematics)�����ӱ���ֵ->����������̼�����
void AX_Kinematics_Inverse(int16_t* input, int16_t* output); //���(InverseKinematics)�����������ٶ�->�����ٶ�

#endif

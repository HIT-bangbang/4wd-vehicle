/**			                                                    
		   ____                    _____ _____  _____        XTARK@���˴���
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP ��ݮ�� ר��ROS�����˿�����                                   
									 
  ****************************************************************************** 
  *           
  * ��Ȩ���У� XTARK@���˴���  ��Ȩ���У�����ؾ�
  * ������վ�� www.xtark.cn
  * �Ա����̣� https://shop246676508.taobao.com  
  * ����ý�壺 www.cnblogs.com/xtark�����ͣ�
  *          
  ******************************************************************************
  * @��  ��  ax_tim.h
  * @��  ��  Musk Han@XTARK
  * @��  ��  V1.0
  * @��  ��  2018-10-26
  * @��  ��  TIM��ʱ����ʱ
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _TIM_H
#define _TIM_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

//OpenCRP�����ӿں���
void TIM6_Init(uint16_t cnt_us);  //TIM6��ʼ��
void TIM6_Cmd(FunctionalState NewState); //TIM6��ʱ�������ر�
uint8_t TIM6_CheckIrqStatus(void);//��־λȷ��

#endif

/******************* (C) ��Ȩ 2018 XTARK **************************************/
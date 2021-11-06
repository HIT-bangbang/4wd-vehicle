/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTI
  
  AL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x_it.h"
#include "bsp_usart.h"
#include "kinematics.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/


static uint8_t uart2_rx_con=0;  //接收计数器
static uint8_t uart2_rx_checksum; //帧头部分校验和
static uint8_t uart2_rx_buf[60];  //接收缓冲，数据内容小于等于32Byte
//外部变量
extern int16_t ax_motor_kp;
extern int16_t ax_motor_ki;
extern int16_t ax_motor_kd;
extern int16_t robot_target_speed[3];  // X Y Yaw
extern int16_t robot_params[2];


/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

// 串口2中断服务函数，用来接收数据并数据解析，同时对速度进行限制
void USART2_IRQHandler(void)                	
{
	uint8_t Res;
	
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)  //接收中断
	{
		  //printf("Get Data!\r\n");
			Res =USART_ReceiveData(USART2);	
		
			if(uart2_rx_con < 3)    //==接收帧头 + 长度
			{
				if(uart2_rx_con == 0)  //接收帧头1 0xAA
				{
					if(Res == 0xAA)
					{
						uart2_rx_buf[0] = Res;
						uart2_rx_con = 1;					
					}
					else
					{
						uart2_rx_con = 0;
					}
				}else if(uart2_rx_con == 1) //接收帧头2 0x55
				{
					if(Res == 0x55)
					{
						uart2_rx_buf[1] = Res;
						uart2_rx_con = 2;
					}
					else
					{
						uart2_rx_con = 0;						
					}				
				}
				else  //接收数据长度
				{
					uart2_rx_buf[2] = Res;
					uart2_rx_con = 3;
					uart2_rx_checksum = (0xAA+0x55) + Res;	//计算校验和
				}
			}
			else    //==接收数据
			{
				if(uart2_rx_con < (uart2_rx_buf[2]-1) )
				{
					uart2_rx_buf[uart2_rx_con] = Res;
					uart2_rx_con++;
					uart2_rx_checksum = uart2_rx_checksum + Res;					
				}
				else    //判断最后1位
				{
					//数据校验
					if( Res == uart2_rx_checksum )  //校验正确
					{	
						//=====此处进行数据解析=========
						//运动控制帧
						if(uart2_rx_buf[3] == 0x11)
						{
							robot_target_speed[0] = (int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]);
							robot_target_speed[1] = (int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]);
							robot_target_speed[2] = (int16_t)((uart2_rx_buf[8]<<8) | uart2_rx_buf[9]);
						  
							//速度限制
							if(robot_target_speed[0] > ROBOT_LINEAR_SPEED_LIMIT)    robot_target_speed[0] = ROBOT_LINEAR_SPEED_LIMIT;
							if(robot_target_speed[0] < (-ROBOT_LINEAR_SPEED_LIMIT)) robot_target_speed[0] = (-ROBOT_LINEAR_SPEED_LIMIT);
							if(robot_target_speed[1] > ROBOT_LINEAR_SPEED_LIMIT)    robot_target_speed[1] = ROBOT_LINEAR_SPEED_LIMIT;
							if(robot_target_speed[1] < (-ROBOT_LINEAR_SPEED_LIMIT)) robot_target_speed[1] = (-ROBOT_LINEAR_SPEED_LIMIT);
							if(robot_target_speed[2] > ROBOT_ANGULAR_SPEED_LIMIT)    robot_target_speed[2] = ROBOT_ANGULAR_SPEED_LIMIT;
							if(robot_target_speed[2] < (-ROBOT_ANGULAR_SPEED_LIMIT)) robot_target_speed[2] = (-ROBOT_ANGULAR_SPEED_LIMIT);
//							printf("get data\r\n");
							printf("目标x速度为,%d \r\n",robot_target_speed[0]);
//							printf("y速度为,%d",robot_target_speed[1]);
//							printf("z速度为,%d",robot_target_speed[2]);

						}
						else
						{
							//PID参数帧
							if(uart2_rx_buf[3] == 0x12)
							{
								ax_motor_kp = (int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]);
								ax_motor_ki = (int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]);
								ax_motor_kd = (int16_t)((uart2_rx_buf[8]<<8) | uart2_rx_buf[9]);
							}
							
							//机器人参数
							if(uart2_rx_buf[3] == 0x13)
							{
								robot_params[0] = (int16_t)((uart2_rx_buf[4]<<8) | uart2_rx_buf[5]);
								robot_params[1] = (int16_t)((uart2_rx_buf[6]<<8) | uart2_rx_buf[7]);
								
								AX_Kinematics_Init(robot_params);
							}
						}
						
						//接收完成，恢复初始状态
						uart2_rx_con = 0;					
					}	
					
				}
			}
			
      USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	} 
}


/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/

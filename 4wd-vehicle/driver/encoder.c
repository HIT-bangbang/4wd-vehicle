/**			                                                    

  * @brief   ���������
  *
  ******************************************************************************
  * @˵  ��
  *
  ******************************************************************************
  */

#include "encoder.h" 

/**
  * @��  ��  ������CD��ʼ��
  * @��  ��  cycle����������
  * @����ֵ  ��
  */
void AX_ENCODER_Init(uint16_t cycle)
{ 
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;   

	//GPIO����ʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�100MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3 , ENABLE); //���������ӳ�书�ܺ���

	//Timer configuration in Encoder mode
	TIM_DeInit(TIM3);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM3, &TIM_ICInitStructure);

	//Reset counter
	TIM3->CNT = 0;

	TIM_Cmd(TIM3, ENABLE);  

	//����IO��Ϊ���ù���-��ʱ��ͨ��
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//TIMʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//Timer configuration in Encoder mode 
	TIM_DeInit(TIM4);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);

	TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
	TIM_TimeBaseStructure.TIM_Period = cycle;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_ICFilter = 6;
	TIM_ICInit(TIM4, &TIM_ICInitStructure);

	//Reset counter
	TIM4->CNT = 0;

	TIM_Cmd(TIM4, ENABLE);     	
}

/**
  * @��  ��  ������CD��ȡ��������ֵ
  * @��  ��  ��
  * @����ֵ  ��������ǰֵ
  */
uint16_t AX_ENCODER_A_GetCounter(void)
{
	return (TIM_GetCounter(TIM3)); 
}

/**
  * @��  ��  ������CD���ü�������ֵ
  * @��  ��  count  ��������ֵ
  * @����ֵ  ��
  */
void AX_ENCODER_A_SetCounter(uint16_t count)
{
	TIM3->CNT = count;
}

/**
  * @��  ��  ��������ȡ��������ֵ
  * @��  ��  ��
  * @����ֵ  ��������ǰֵ
  */
uint16_t AX_ENCODER_B_GetCounter(void)
{
	return (TIM_GetCounter(TIM4)); 
}

/**
  * @��  ��  ���������ü�������ֵ
  * @��  ��  count  ��������ֵ
  * @����ֵ  ��
  */
void AX_ENCODER_B_SetCounter(uint16_t count)
{
	TIM4->CNT = count;
}

/******************* (C) ��Ȩ 2019 XTARK **************************************/

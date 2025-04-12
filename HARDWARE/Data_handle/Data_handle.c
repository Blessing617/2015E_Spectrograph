#include "Data_handle.h"

#define FRE 	1024000
#define sample 	5

uint16_t ADC_GetValue[sample] = {0};

extern 	uint8_t 	adc_flag;
extern  uint16_t 	num;
extern	float		Fre;
extern	float		Start_Fre;
extern	float		Stop_Fre;
extern  float		Step;

void Fre_Control(uint32_t Fre)
{
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	uint32_t MaxData;
	uint16_t div = 1;
	while ((SystemCoreClock / Fre / div) > 65535)
	{
		div++;
	}
	MaxData = SystemCoreClock / Fre / div - 1;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = div - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = MaxData;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Init(void)
{
	Fre_Control(FRE);
	HAL_TIM_Base_Start(&htim3);
	//HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_GetValue, 1024);
}

void ADC_Get(void)
{
	adc_flag = 0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_GetValue, sample);
}

void Data_handle(void)
{
	uint16_t i;
	uint16_t data_average = 0;
	uint16_t line_num = 0;
	
	static uint16_t data_max = 0;
	static uint16_t data_max_position = 0;
	static uint16_t data [400] = {0};
	
	int posX1=0, posY1=0, posX2=0, posY2=0;	
	
	for (i = 0; i < sample; i++)
	{
		data_average += ADC_GetValue[i];
//		printf("adc = %d\r\n",ADC_GetValue[i]);
	}
	data_average /= sample;
	if(data_average < 190 )
		data_average = 0;
	data[num] = data_average;

	if(data_max < data[num])
	{
		data_max = data[num];
		data_max_position = num;
	}
	num++;
	if(num >= (Stop_Fre-Start_Fre)/Step)
	{ 
		data[0] = 0;
		//OLED_ShowFloat(0,4,num,2);
		for(i = 0;i<num;i++)
		{
			if((float)data[i] > (float)data_max*0.02f)
				line_num++;	
			
			posX1 = 2*i;
			posX2 = posX1 + 2;
			posY1 = 271 - (data[i] 	 * (200.0f / 4095.0f));
			posY2 = 271 - (data[i+1] * (200.0f / 4095.0f));
			printf("line %d,%d,%d,%d,%d\xff\xff\xff", posX1, posY1, posX2, posY2, 63488);	
		}
		OLED_ShowFloat(0,4,data[data_max_position],2);
		printf("page2.t3.txt=\"%.1f MHz\"\xff\xff\xff",Start_Fre+ 0.1f * data_max_position - 10.8f);
		printf("page2.t4.txt=\"%d个\"\xff\xff\xff",line_num-1);
		
		data_max = 0;
		data_max_position = 0; 
	}
	
//	printf("平均值 = %d\r\n",data_average);
//	printf("num = %d\r\n",++num);
}

/*
 * DHT11.c
 *
 *  Created on: Jun 13, 2025
 *      Author: nihal
 */

#include "dht11.h"

extern TIM_HandleTypeDef htim2;

void delay_us (uint32_t time)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim2))<time);
}

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//#define DHT11_PORT GPIOA
//#define DHT11_PIN GPIO_PIN_1

void DHT11_Start (void)
{
	Set_Pin_Output(DHT11_PORT, DHT11_PIN);
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 0);
	delay_us(20000);
	HAL_GPIO_WritePin (DHT11_PORT, DHT11_PIN, 1);
	delay_us (20);
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);
}

uint8_t DHT11_Check_Response (void)
{
	Set_Pin_Input(DHT11_PORT, DHT11_PIN);
	uint8_t Response = 0;
	delay_us (40);
	if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
	{
		delay_us (80);
		if ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN))) Response = 1;
		else Response = -1;
	}
	while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
	return Response;
}

uint8_t DHT11_Read (void)
{
	uint8_t i,j;
	for (j=0;j<8;j++)
	{
		while (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
		delay_us (40);
		if (!(HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)))
			i&= ~(1<<(7-j));
		else i|= (1<<(7-j));
		while ((HAL_GPIO_ReadPin (DHT11_PORT, DHT11_PIN)));
	}
	return i;
}


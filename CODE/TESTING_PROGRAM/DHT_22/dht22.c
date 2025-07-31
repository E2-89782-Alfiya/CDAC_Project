/*
 * dht22.c
 *
 *  Created on: Jun 12, 2025
 *      Author: nihal
 */

#include"dht22.h"

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

//#define DHT22_PORT GPIOA
//#define DHT22_PIN GPIO_PIN_1

void DHT22_Start(void)
{
  Set_Pin_Output(DHT22_PORT, DHT22_PIN);
  HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET);
  delay(1200); // 1.2ms low
  HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);
  delay(20);   // 20us high
  Set_Pin_Input(DHT22_PORT, DHT22_PIN);
}

uint8_t DHT22_Check_Response(void)
{
  uint8_t Response = 0;
  delay(40);
  if (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)))
  {
    delay(80);
    if ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))) Response = 1;
    else Response = -1;
  }
  while ((HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)));
  return Response;
}

uint8_t DHT22_Read(void)
{
  uint8_t i, j;
  uint8_t data = 0;

  for (j = 0; j < 8; j++)
  {
    while (!(HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))); // wait for high
    delay(40);
    if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN))
      data |= (1 << (7 - j)); // set bit
    else
      data &= ~(1 << (7 - j)); // clear bit
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN)); // wait for low
  }

  return data;
}

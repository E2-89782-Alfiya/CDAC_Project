/*
 * dht22.h
 *
 *  Created on: Jun 12, 2025
 *      Author: nihal
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include"stm32f407xx.h"
#include"stm32f4xx.h"

#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_1

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) ;
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) ;
void DHT22_Start(void) ;
uint8_t DHT22_Check_Response(void) ;
uint8_t DHT22_Read(void) ;


#endif /* INC_DHT22_H_ */

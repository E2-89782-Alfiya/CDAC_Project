/*
 * dht22.h
 *
 *  Created on: Jun 13, 2025
 *      Author: nihal
 */

#ifndef INC_DHT22_H_
#define INC_DHT22_H_

#include<stm32f4xx.h>

#define DHT22_PORT   GPIOD
#define DHT22_PIN    GPIO_PIN_8

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) ;
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) ;
void DHT22_Start (void) ;
uint8_t DHT22_Check_Response (void) ;
uint8_t DHT22_Read (void) ;

int Read_DHT22(float *temperature, float *humidity) ;

#endif /* INC_DHT22_H_ */

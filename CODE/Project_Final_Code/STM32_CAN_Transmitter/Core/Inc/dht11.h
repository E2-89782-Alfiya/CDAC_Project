/*
 * dht11.h
 *
 *  Created on: Jun 13, 2025
 *      Author: nihal
 */

#ifndef INC_DHT11_H_
#define INC_DHT11_H_

#include<stm32f4xx.h>

#define DHT11_PORT   GPIOD
#define DHT11_PIN    GPIO_PIN_8

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) ;
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) ;
void DHT11_Start (void) ;
uint8_t DHT11_Check_Response (void) ;
uint8_t DHT11_Read (void) ;

int Read_DHT11(float *temperature, float *humidity) ;

#endif /* INC_DHT11_H_ */

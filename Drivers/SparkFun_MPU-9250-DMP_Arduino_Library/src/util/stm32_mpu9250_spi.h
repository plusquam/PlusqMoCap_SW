/*
 * stm32_mpu9250_spi.h
 *
 *  Created on: 15 lip 2019
 *      Author: plusq
 */

#ifndef SPARKFUN_MPU_9250_DMP_ARDUINO_LIBRARY_SRC_UTIL_STM32_MPU9250_SPI_H_
#define SPARKFUN_MPU_9250_DMP_ARDUINO_LIBRARY_SRC_UTIL_STM32_MPU9250_SPI_H_

#include <stdio.h>
#include "stm32wbxx_hal.h"

#define SPI 1

#ifdef SPI
#define SPI_SPEEDUP_FOR_SENSOR_DATA	0

void set_spi_handler(SPI_HandleTypeDef *handler);
extern void set_CS_portpin(GPIO_TypeDef *port, uint16_t pin);
extern void get_CS_portpin(GPIO_TypeDef *port, uint16_t *pin);
extern uint8_t spi_write( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data );
extern uint8_t spi_read( unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data );
#define i2c_write(a, b, c, d) spi_write(a, b, c, d)
#define i2c_read(a, b, c, d)  spi_read(a, b, c, d)
#else
#include "arduino_mpu9250_i2c.h"
#define i2c_write(a, b, c, d) arduino_i2c_write(a, b, c, d)
#define i2c_read(a, b, c, d)  arduino_i2c_read(a, b, c, d)
#endif


#endif /* SPARKFUN_MPU_9250_DMP_ARDUINO_LIBRARY_SRC_UTIL_STM32_MPU9250_SPI_H_ */

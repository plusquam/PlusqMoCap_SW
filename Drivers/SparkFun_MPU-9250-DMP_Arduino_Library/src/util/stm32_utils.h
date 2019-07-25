/*
 * stm32_utils.h
 *
 *  Created on: Jul 19, 2019
 *      Author: plusq
 */

#ifndef SPARKFUN_MPU_9250_DMP_ARDUINO_LIBRARY_SRC_UTIL_STM32_UTILS_H_
#define SPARKFUN_MPU_9250_DMP_ARDUINO_LIBRARY_SRC_UTIL_STM32_UTILS_H_

#include "stm32wbxx_hal.h"
#include <stdio.h>

#define min(a,b) ( (a)>=(b) ? (b) : (a) )

#define delay_ms  HAL_Delay

int getMsWrapper(unsigned long *count);
#define get_ms    getMsWrapper

#define log_i     printf
#define log_e     printf

#endif /* SPARKFUN_MPU_9250_DMP_ARDUINO_LIBRARY_SRC_UTIL_STM32_UTILS_H_ */
/*
 * app_mpu.h
 *
 *  Created on: 10 wrz 2019
 *      Author: plusq
 */

#pragma once

#include "main.h"

// Symbols declarations
#define MPU_DMP_DATA_ENABLE	0 // Set to 0 for raw sensor data, 1 for DMP data
#if MPU_DMP_DATA_ENABLE
#define MPU_SAMPLE_RATE 200 //[Hz]
#else
#define MPU_SAMPLE_RATE 100 //[Hz]
#endif

#define MPU_SENSORS_SET (INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS)

#define NUMBER_OF_SENSORS		3
#define ENABLE_SENSORS_SYNCH	0
#define PRINT_FULL_DATA			0

// Variable declarations
extern UART_HandleTypeDef 	huart1;

extern volatile uint8_t 	mpuDataToBeSend[75];
extern volatile uint8_t		mpuDataLength;
extern volatile bool		runMeasurement;
extern volatile bool		runCalibration;


// Funtion declarations
#if PRINT_FULL_DATA
void printIMUData(uint8_t sensor_number);
#endif
void InitialMpuProcedure(void);
void ReadMpuDataCallback(void);
void SetupMPUSensors(void);
void MeasurementLoop(void);
void PerformCalibration(void);
bool test_whoAmI(void);

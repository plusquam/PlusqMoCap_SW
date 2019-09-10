/*
 * app_mpu.cpp
 *
 *  Created on: 10 wrz 2019
 *      Author: plusq
 */

#include "app_mpu.h"

extern "C" {
#include "MPU9250_RegisterMap.h"
#include "stm32_mpu9250_spi.h"
#include "stm32_utils.h"
#include "app_conf.h"
#include "scheduler.h"
}
#include <string.h>

typedef struct
{
	uint8_t			number;
	GPIO_TypeDef 	*port;
	uint16_t 		pin;
} SpiSlaveHandler_t;

///////////////////// GLOBAL VARIABLES ///////////////////////////////
static SpiSlaveHandler_t spiSlavesArray[] =
{
	[0] = {.number = 0, .port = SPI1_CS_0_GPIO_Port, .pin = SPI1_CS_0_Pin},
	[1] = {.number = 1, .port = SPI1_CS_1_GPIO_Port, .pin = SPI1_CS_1_Pin},
	[2] = {.number = 2, .port = SPI1_CS_2_GPIO_Port, .pin = SPI1_CS_2_Pin}
};

static constexpr uint8_t SLAVES_MASK(void)
{
	return (1 << 0) | (1 << 1) | (1 << 2); // Slaves: 0 | 1 | 2
}

static constexpr uint16_t MPU_SAMPLE_INTERVAL_MS(void)
{
	return 1000 / MPU_SAMPLE_RATE;
}

#ifndef NUMBER_OF_SENSORS
#define NUMBER_OF_SENSORS (sizeof(spiSlavesArray)/sizeof(SpiSlaveHandler_t))
#endif

static MPU9250_DMP 		IMUs[NUMBER_OF_SENSORS];

volatile uint8_t 	mpuDataToBeSend[75];
volatile uint8_t	mpuDataLength = 0u;

volatile bool				runMeasurement 		= true;
volatile bool				runCalibration 		= false;
static volatile bool		firstMeasurementLoop = true;
static volatile uint16_t	timestampInterval 	= 0u;
static volatile bool		isMpuMeasureReady 	= false;
static volatile uint8_t		readySensorsMask 	= 0u;
static volatile bool		callbackCalled 		= false;


///////////////////// FUNCTIONS ///////////////////////////////
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{
	if(isMpuMeasureReady)
	{
		switch (GPIO_Pin)
		{
			case MPU_INT_0_Pin:
				readySensorsMask |= (1 << 0); // sensor 0 ready
				break;
			case MPU_INT_1_Pin:
				readySensorsMask |= (1 << 1); // sensor 1 ready
				break;
			case MPU_INT_2_Pin:
				readySensorsMask |= (1 << 2); // sensor 2 ready
				break;
			case MPU_INT_3_Pin:
				readySensorsMask |= (1 << 3); // sensor 3 ready
				break;
		}

		if((!callbackCalled) && (GPIO_Pin == MPU_INT_0_Pin)) {
			static volatile uint32_t	previousTimestamp = 0u;
			uint32_t currTimestamp = HAL_GetTick();

			if(firstMeasurementLoop) {
				timestampInterval = 0u;
				firstMeasurementLoop = false;
			}
			else {
				if(previousTimestamp < currTimestamp) {
					timestampInterval = (uint16_t)(currTimestamp - previousTimestamp);
				}
				else {
					// Timer overflow
					timestampInterval = (uint16_t)(currTimestamp + (~((uint32_t)0u) - previousTimestamp));
				}
			}
			previousTimestamp = currTimestamp;
			SCH_SetTask(1<<CFG_TASK_MPU9250_INT_ID, CFG_SCH_PRIO_0);
		}

	}
	return;
}



void SetupMPUSensors(void)
{
	////////////////// SETUP /////////////////////////////
	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
		set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);

		// Call IMUs[i].begin() to verify communication and initialize
		if (IMUs[i].resetDevice() != INV_SUCCESS)
		{
			while (1)
			{
				printf("Unable to reset MPU-9250 nr %d\n", i);
				HAL_Delay(10);
			}
		}
	}

	HAL_Delay(100);

	////////////////// SETUP /////////////////////////////
	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
		set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);

		// Call IMUs[i].begin() to verify communication and initialize
		if (IMUs[i].begin() != INV_SUCCESS)
		{
			while (1)
			{
				printf("Unable to communicate with MPU-9250\n");
				printf("Check connections, and try again.\n");
				HAL_Delay(5000);
			}
		}

#if MPU_DMP_DATA_ENABLE
		// DMP_FEATURE_LP_QUAT can also be used. It uses the
		// accelerometer in low-power mode to estimate quat's.
		// DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
		/*(	DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
			DMP_FEATURE_GYRO_CAL, // Use gyro calibration
			MPU_SAMPLE_RATE) // Set DMP FIFO rate to 200 Hz */
		if ( IMUs[i].dmpBegin( DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL, MPU_SAMPLE_RATE) != INV_SUCCESS )
			while (1)
			{
				printf("Unable to setup DMP in MPU-9250 nr %d\n", i);
				printf("Check connections, and try again.\n");
				HAL_Delay(1000);
			}
#else
		// Enable all sensors, and set sample rates to 4Hz.
		// (Slow so we can see the interrupt work.)
		IMUs[i].setSensors(MPU_SENSORS_SET);
		IMUs[i].setSampleRate(MPU_SAMPLE_RATE); // Set accel/gyro sample rate to 100 Hz

#if MPU_SENSORS_SET & INV_XYZ_COMPASS
		IMUs[i].setCompassSampleRate(MPU_SAMPLE_RATE); // Set mag rate to 100 Hz
#endif

		// Use enableInterrupt() to configure the MPU-9250's
		// interrupt output as a "data ready" indicator.
//		IMUs[i].enableInterrupt(1); Not yet

		// The interrupt level can either be active-high or low.
		// Configure as active-low, since we'll be using the pin's
		// internal pull-up resistor.
		// Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
		IMUs[i].setIntLevel(INT_ACTIVE_LOW);

		// The interrupt can be set to latch until data has
		// been read, or to work as a 50us pulse.
		// Use latching method -- we'll read from the sensor
		// as soon as we see the pin go LOW.
		// Options are INT_LATCHED or INT_50US_PULSE
		IMUs[i].setIntLatched(INT_50US_PULSE);
#endif

		////////////////// Disable interrupts /////////////////////////////
		if ( IMUs[i].enableInterrupt(0) != INV_SUCCESS)
		while (1)
		{
			printf("Unable to disable interrupt.\n");
			printf("Check connections, and try again.\n");
			HAL_Delay(1000);
		}
	}

	printf("Setup done.\n");
}



#if PRINT_FULL_DATA
void printIMUData(uint8_t sensor_number)
{
#if MPU_DMP_DATA_ENABLE
	char str1[10], str2[10];
	// After calling dmpUpdateFifo() the ax, gx, mx, etc. values
	// are all updated.
	// Quaternion values are, by default, stored in Q30 long
	// format. calcQuat turns them into a float between -1 and 1
	float q0 = IMUs[sensor_number].calcQuat(IMUs[sensor_number].qw);
	float q1 = IMUs[sensor_number].calcQuat(IMUs[sensor_number].qx);
	float q2 = IMUs[sensor_number].calcQuat(IMUs[sensor_number].qy);
	float q3 = IMUs[sensor_number].calcQuat(IMUs[sensor_number].qz);

	// Quaternions
	ftoa(q0, str1);
	ftoa(q1, str2);
	printf("Sensor: %d\n", sensor_number);
	printf("Q: %s, %s", str1, str2);
	ftoa(q2, str1);
	ftoa(q3, str2);
	printf(", %s, %s\n", str1, str2);

	// Euler angles
//	ftoa(IMUs[sensor_number].roll, str1);
//	ftoa(IMUs[sensor_number].pitch, str2);
//	printf("R/P/Y: %s, %s", str1, str2);
//	ftoa(IMUs[sensor_number].yaw, str1);
//	printf(", %s\n", str1);
#else
	char str1[10], str2[10], str3[10];
	// After calling update() the ax, ay, az, gx, gy, gz, mx,
	// my, mz, time, and/or temerature class variables are all
	// updated. Access them by placing the object. in front:

	printf("Sensor: %d\n", sensor_number);

	// Use the calcAccel, calcGyro, and calcMag functions to
	// convert the raw sensor readings (signed 16-bit values)
	// to their respective units.
	float data1 = IMUs[sensor_number].calcAccel(IMUs[sensor_number].ax);
	float data2 = IMUs[sensor_number].calcAccel(IMUs[sensor_number].ay);
	float data3 = IMUs[sensor_number].calcAccel(IMUs[sensor_number].az);

	// Accel
	ftoa(data1, str1);
	ftoa(data2, str2);
	ftoa(data3, str3);
	printf("A: %s, %s, %s g\n", str1, str2, str3);

	// Gyro
	data1 = IMUs[sensor_number].calcGyro(IMUs[sensor_number].gx);
	data2 = IMUs[sensor_number].calcGyro(IMUs[sensor_number].gy);
	data3 = IMUs[sensor_number].calcGyro(IMUs[sensor_number].gz);
	ftoa(data1, str1);
	ftoa(data2, str2);
	ftoa(data3, str3);
	printf("G: %s, %s, %s dps\n", str1, str2, str3);

	// Mag
	data1 = IMUs[sensor_number].calcMag(IMUs[sensor_number].mx);
	data2 = IMUs[sensor_number].calcMag(IMUs[sensor_number].my);
	data3 = IMUs[sensor_number].calcMag(IMUs[sensor_number].mz);
	ftoa(data1, str1);
	ftoa(data2, str2);
	ftoa(data3, str3);
	printf("M: %s, %s, %s uT\n", str1, str2, str3);
#endif

	// Time
	printf("Time: %lu ms\n", IMUs[sensor_number].time);
}
#endif



bool test_whoAmI()
{
	uint8_t readData[5] = {0};
	if(spi_read(0u, MPU9250_WHO_AM_I, 4u, readData)) {
		printf("Check error\n");
		return false;
	}
	else if(readData[0] == 0x71) {
		return true;
	}
	else {
		printf("ID matching error\n");
		return false;
	}
}



void HAL_Delay(uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;

  /* Add a freq to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
    wait += (uint32_t)(uwTickFreq);
  }

  while ((HAL_GetTick() - tickstart) < wait)
  {
	  // Run all tasks
	  	SCH_Run(SCH_DEFAULT);
  }
}



void MeasurementLoop(void)
{
	////////////////// MEASUREMENT START //////////////////////
	firstMeasurementLoop = true;

	printf("Measurement start.\n");

	////////////////// SENSORS SETUP //////////////////////////
	SetupMPUSensors();

	#if MPU_DMP_DATA_ENABLE
	////////////////// RESET FIFO /////////////////////////////
	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
	set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);
	IMUs[i].resetFifo();
	}
	printf("Fifo reset done.\n");
	#endif

#if ENABLE_SENSORS_SYNCH
	////////////////// ENABLE MPU9250 TRIGGER /////////////////////////
	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
		set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);

		mpu_set_fsync_configuration();
		mpu_set_slave4_interrupt();
	}

	// Enable PWM signal for MPU triggering
	MX_TIM1_Init();
#endif

	////////////////// ENABLE MPU9250 INTERRUPT /////////////////////////
	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
		set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);
		IMUs[i].enableInterrupt(1);
	}

	uint32_t primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
	__disable_irq();          /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
	isMpuMeasureReady = true;
	__set_PRIMASK(primask_bit); /**< Restore PRIMASK bit*/

	////////////////// MEASUREMENT LOOP START ///////////////////////////
	while(runMeasurement)
	{
		SCH_Run(SCH_DEFAULT);
	}

#if ENABLE_SENSORS_SYNCH
	// Disable PWM signal for MPU triggering
	HAL_TIM_Base_MspDeInit(&htim1);
#endif

	primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
	__disable_irq();          /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
	isMpuMeasureReady = false;
	__set_PRIMASK(primask_bit); /**< Restore PRIMASK bit*/

	printf("Measurement stopped.\n\n");

	////////////////// SENSORS RESET //////////////////////////
	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
		////////////////// DISABLE MPU9250 INTERRUPT /////////////////////////
		set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);
		IMUs[i].enableInterrupt(0);
		IMUs[i].resetDevice();
	}
	printf("Sensors reset.\n");

	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}



void PerformCalibration(void)
{
	constexpr int numberOfSamples = 200;
	runCalibration = false;
	printf("Calibration begin...\n");
	////////////////// SENSORS SETUP //////////////////////////
	SetupMPUSensors();

	int64_t mean_acc_x[NUMBER_OF_SENSORS] = {0}, mean_acc_y[NUMBER_OF_SENSORS] = {0}, mean_acc_z[NUMBER_OF_SENSORS] = {0};
	int64_t mean_gyro_x[NUMBER_OF_SENSORS] = {0}, mean_gyro_y[NUMBER_OF_SENSORS] = {0}, mean_gyro_z[NUMBER_OF_SENSORS] = {0};

	////////////////// CALIBRATION //////////////////////////
	for(uint8_t sample = 0; sample < numberOfSamples; ++sample) {
		for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)	{
			set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);

			while(!IMUs[i].dataReady())
			{
				delay_ms(1);
			}

			if( IMUs[i].allDataUpdate(NULL, 0) == INV_SUCCESS ) {
				mean_acc_x[i] += IMUs[i].ax;
				mean_acc_y[i] += IMUs[i].ay;
				mean_acc_z[i] += IMUs[i].az;

				mean_gyro_x[i] += IMUs[i].gx;
				mean_gyro_y[i] += IMUs[i].gy;
				mean_gyro_z[i] += IMUs[i].gz;
			}
		}

		if(sample % 10 == 0)
			printf("Sample %d done.\n", sample);
	}

	printf("\nCalculating mean values...\n");
	mpuDataToBeSend[0] = 'B';

	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)	{
		mean_acc_x[i] /= numberOfSamples;
		mean_acc_y[i] /= numberOfSamples;
		mean_acc_z[i] /= numberOfSamples;
		mean_acc_z[i] -= IMUs[i].getAccelSens();

		mean_gyro_x[i] /= numberOfSamples;
		mean_gyro_y[i] /= numberOfSamples;
		mean_gyro_z[i] /= numberOfSamples;

		uint8_t offset = i * 12u + 1;
		mpuDataToBeSend[offset] = (uint8_t)(mean_acc_x[i] >> 8);
		mpuDataToBeSend[offset + 1] = (uint8_t)mean_acc_x[i];
		mpuDataToBeSend[offset + 2] = (uint8_t)(mean_acc_y[i] >> 8);
		mpuDataToBeSend[offset + 3] = (uint8_t)mean_acc_y[i];
		mpuDataToBeSend[offset + 4] = (uint8_t)(mean_acc_z[i] >> 8);
		mpuDataToBeSend[offset + 5] = (uint8_t)mean_acc_z[i];

		offset += 6;
		mpuDataToBeSend[offset] = (uint8_t)(mean_gyro_x[i] >> 8);
		mpuDataToBeSend[offset + 1] = (uint8_t)mean_gyro_x[i];
		mpuDataToBeSend[offset + 2] = (uint8_t)(mean_gyro_y[i] >> 8);
		mpuDataToBeSend[offset + 3] = (uint8_t)mean_gyro_y[i];
		mpuDataToBeSend[offset + 4] = (uint8_t)(mean_gyro_z[i] >> 8);
		mpuDataToBeSend[offset + 5] = (uint8_t)mean_gyro_z[i];
	}
	mpuDataLength = NUMBER_OF_SENSORS * 12u + 1;

	// Send calibration data
	SCH_SetTask(1<<CFG_TASK_MPU_DATA_READY_ID, CFG_SCH_PRIO_1);

	printf("\nCalibration done.\n");
}



static constexpr uint8_t MEASUREMENT_TIMEOUT(void)
{
	return (uint8_t)MPU_SAMPLE_INTERVAL_MS() * 2;
}


void ReadMpuDataCallback(void)
{
	uint32_t primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
	__disable_irq();          /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
	if(isMpuMeasureReady)
	{
		callbackCalled = true;
		// Get timestamp
		mpuDataToBeSend[1] = (uint8_t)(timestampInterval >> 8);
		mpuDataToBeSend[2] = (uint8_t)timestampInterval;
		IMUs[0].time = timestampInterval;

		__set_PRIMASK(primask_bit); /**< Restore PRIMASK bit*/

#if PRINT_FULL_DATA
		static uint8_t numOfIters = 0u;
		static constexpr uint8_t numOfItersToPrint = MPU_SAMPLE_RATE/3; // value set for 1Hz printf refresh rate
#endif

		uint8_t error_result_mask = 0u;

#if !MPU_DMP_DATA_ENABLE
		uint8_t		sensorsCompletedMask = 0u;
		uint32_t	startTime = HAL_GetTick();

		while(	(sensorsCompletedMask != SLAVES_MASK()) &&
				(HAL_GetTick() < startTime + MEASUREMENT_TIMEOUT()) )
		{
#endif
			for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
			{

#if !MPU_DMP_DATA_ENABLE
				uint8_t sensorShiftedNumber = 1 << i;
				// if sensor already read or sensor not ready -> skip sensor
				if((sensorShiftedNumber & sensorsCompletedMask) || !(sensorShiftedNumber & readySensorsMask))
					continue;
#endif
				set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);

#if MPU_DMP_DATA_ENABLE
				// Check for new data in the FIFO
				if (i == 0 )
					while(!IMUs[i].fifoAvailable())
					{
						delay_ms(1);
					}

				// Use dmpUpdateFifo to update the ax, gx, mx, etc. values
				if ( IMUs[i].dmpUpdateFifo() == INV_SUCCESS)
				{
					// computeEulerAngles can be used -- after updating the
					// quaternion values -- to estimate roll, pitch, and yaw
					//IMUs[i].computeEulerAngles();
				}
				else
				{
					printf("DMP update fifo read error!\n");
				}
#else

				// Update of IMU sensor data
#if MPU_SENSORS_SET & INV_XYZ_COMPASS
				if(IMUs[i].allDataUpdate((uint8_t*)mpuDataToBeSend, i * 18 + 3) != INV_SUCCESS)
#else
				if(IMUs[i].allDataUpdate((uint8_t*)mpuDataToBeSend, i * 12 + 3) != INV_SUCCESS)
#endif
				{
					printf("IMU nr %d data read error!\n", i);
					error_result_mask |= sensorShiftedNumber;
				}

				// Add sensor to the completed mask and clear sensor ready bit
				sensorsCompletedMask |= sensorShiftedNumber;

				primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
				__disable_irq();
				readySensorsMask &= ~sensorShiftedNumber;
				__set_PRIMASK(primask_bit); /**< Restore PRIMASK bit*/

#endif
			}
#if !MPU_DMP_DATA_ENABLE
		}

		if(sensorsCompletedMask != SLAVES_MASK()) {
			printf("Sensor mask doesn't match!\n");
			error_result_mask = 0xff;
		}
#endif

		if(!error_result_mask) {
			mpuDataToBeSend[0] = 'S';
#if MPU_SENSORS_SET & INV_XYZ_COMPASS
			mpuDataLength = NUMBER_OF_SENSORS * 18 + 3;
#else
			mpuDataLength = NUMBER_OF_SENSORS * 12 + 3;
#endif
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
		}
		else {
			mpuDataToBeSend[0] = 'E';
			mpuDataLength = 3u;
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
		}

		// Send measurements
		SCH_SetTask(1<<CFG_TASK_MPU_DATA_READY_ID, CFG_SCH_PRIO_1);

#if PRINT_FULL_DATA
		numOfIters++;

		if(numOfIters >= numOfItersToPrint) {
			for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
			{
				printIMUData(i);
			}
			numOfIters = 0u;
			printf("\n");
		}
#endif

		primask_bit = __get_PRIMASK();  /**< backup PRIMASK bit */
		__disable_irq();          /**< Disable all interrupts by setting PRIMASK bit on Cortex*/
		callbackCalled = false;
	}

	__set_PRIMASK(primask_bit); /**< Restore PRIMASK bit*/
}



void InitialMpuProcedure(void)
{
	///////////////// SPI FOR MPU9250 SET ///////////////////////
	set_spi_handler(&hspi1);

	///////////////// SENSOR CHECK ///////////////////////
	for(uint8_t i = 0u; i < NUMBER_OF_SENSORS; i++)
	{
		set_CS_portpin(spiSlavesArray[i].port, spiSlavesArray[i].pin);
		while(!test_whoAmI())
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
			// Sensor check fail
			printf("Sensor nr %d check fail! Try again.\n", i);
			delay_ms(500);
		}

		IMUs[i].resetDevice();
	}

	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
	printf("Sensors check passed.\n");
}




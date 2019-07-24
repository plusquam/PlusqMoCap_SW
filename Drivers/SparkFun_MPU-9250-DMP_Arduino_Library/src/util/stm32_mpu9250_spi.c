/*
 * stm32_mpu9250_spi.c
 *
 *  Created on: 15 lip 2019
 *      Author: plusq
 */
#include "stm32_mpu9250_spi.h"
#include "MPU9250_RegisterMap.h"
#include "stm32_utils.h"
#include "main.h"

#define CORE_FREQ_MHZ 64u

static void delay_us(uint8_t microseconds)
{
	if(microseconds)
	{
		uint16_t counter;
		const uint16_t limit = microseconds * CORE_FREQ_MHZ / 7u; // approximate number of cpu cycles per one for iteration
		for( counter = 0u; counter < limit; ++counter)
		{
			;
		}
	}
}

/**
 * Send a byte-array of data through SPI bus (blocking)
 * @param I2Caddress (NOT USED) Here for compatibility with I2C HAL implementation
 * @param regAddress Address of a first register in MPU to start writing into
 * @param data Buffer of data to send
 * @param length Length of data to send
 * @return 0 to verify that function didn't hang somewhere
 */
//uint8_t HAL_MPU_WriteBytes(uint8_t I2Caddress, uint8_t regAddress,
//                           uint16_t length, uint8_t *data)
//{
//    uint16_t i;
//    uint32_t dummy[1];
//
//    regAddress = regAddress & 0x7F; //  MSB = 0 for writing operation
//    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x00);
//    HAL_DelayUS(1);
//    SSIDataPut(SSI2_BASE, regAddress);
//
//    for (i = 0; i < length; i++)
//        SSIDataPut(SSI2_BASE, data[i]);
//
//    while(SSIBusy(SSI2_BASE));
//    MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0xFF);
//
//    while (SSIDataGetNonBlocking(SSI2_BASE, &dummy[0]));
//
//    return 0;
//}

static inline uint8_t spi_write_register(uint8_t reg_addr, uint8_t * data, uint8_t length)
{
	uint8_t returnVal = 0u;
	reg_addr &= 0x7F; //  MSB = 0 for writing operation

	HAL_GPIO_WritePin(SPI1_CS_0_GPIO_Port, SPI1_CS_0_Pin, GPIO_PIN_RESET);
	delay_us(1);

	returnVal |= HAL_SPI_Transmit(&hspi1, &reg_addr, 1u, 2u);
	if(!returnVal)
		returnVal |= HAL_SPI_Transmit(&hspi1, data, length, 1u * length);

	HAL_GPIO_WritePin(SPI1_CS_0_GPIO_Port, SPI1_CS_0_Pin, GPIO_PIN_SET);

	if(returnVal)
		printf("SPI write error!");

	return returnVal;
}

static inline uint8_t spi_read_register(uint8_t reg_addr, uint8_t * data, uint8_t length)
{
	uint8_t returnVal = 0u;
	if(length < 30)
	{
		reg_addr |= 0x80; //  MSB = 1 for reading operation

		HAL_GPIO_WritePin(SPI1_CS_0_GPIO_Port, SPI1_CS_0_Pin, GPIO_PIN_RESET);
		uint8_t dummy_buffer[30] = {0};
		delay_us(1);
		returnVal |= HAL_SPI_Transmit(&hspi1, &reg_addr, 1u, 2u);
		if(!returnVal)
			returnVal |= HAL_SPI_TransmitReceive(&hspi1, dummy_buffer, data, length, 1u * length);

		HAL_GPIO_WritePin(SPI1_CS_0_GPIO_Port, SPI1_CS_0_Pin, GPIO_PIN_SET);
	}
	else
	{
		printf("Error: SPI read length exceeded: %d", length);
		returnVal = 1u;
	}

	if(returnVal)
		printf("SPI read error!");

	return returnVal;
}

uint8_t spi_write( uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data )
{
	uint8_t returnVal = 0u;

    if( slave_addr == 0x0C )
    {
        // We use slave 4 because the Motion Library only uses slaves 0 and 1
        uint8_t byte;
        byte = slave_addr;
        returnVal |= spi_write_register( MPU9250_I2C_SLV4_ADDR, &byte, 1 );

        for( uint8_t i = 0; i < length; i++ )
        {
            byte = reg_addr + i;
            returnVal |= spi_write_register( MPU9250_I2C_SLV4_REG, &byte, 1 );
            returnVal |= spi_write_register( MPU9250_I2C_SLV4_DO, &(data[i]), 1 );
            byte = 0x80;
            returnVal |= spi_write_register( MPU9250_I2C_SLV4_CTRL, &byte, 1 );
            HAL_Delay(20u);
        }
    }
    else
    {
    	returnVal |= spi_write_register( reg_addr, (uint8_t *)data, length );
    }

    return returnVal;
}

uint8_t spi_read( uint8_t slave_addr, uint8_t reg_addr, uint8_t length, uint8_t *data )
{
	uint8_t returnVal = 0u;

    if( slave_addr == 0x0C )
    {
        // We use slave 4 because the Motion Library only uses slaves 0 and 1
        uint8_t byte;
        byte = slave_addr | 0x80;
        returnVal |= spi_write_register( MPU9250_I2C_SLV4_ADDR, &byte, 1 );

        for( uint8_t i = 0; i < length; i++ )
        {
            byte = reg_addr + i;
            returnVal |= spi_write_register( MPU9250_I2C_SLV4_REG, &byte, 1 );
//            spi_write_register( MPU9250_I2C_SLV4_DO, &(data[i]), 1 );
            byte = 0x80;
            returnVal |= spi_write_register( MPU9250_I2C_SLV4_CTRL, &byte, 1 );
            HAL_Delay(20u);
            returnVal |= spi_read_register( MPU9250_I2C_SLV4_DI, &(data[i]), 1 );
        }
    }
    else
    {
    	returnVal |= spi_read_register( reg_addr, (uint8_t *)data, length );
    }

    return returnVal;
}

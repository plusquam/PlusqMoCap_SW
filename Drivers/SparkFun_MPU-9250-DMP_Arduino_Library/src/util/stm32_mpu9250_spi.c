/*
 * stm32_mpu9250_spi.c
 *
 *  Created on: 15 lip 2019
 *      Author: plusq
 */
#include "stm32_mpu9250_spi.h"

#ifdef SPI
#include "MPU9250_RegisterMap.h"
#include "stm32_utils.h"

static SPI_HandleTypeDef *spi_handler;
static uint16_t current_CS_pin;
static GPIO_TypeDef *current_CS_port = NULL;

void set_spi_handler(SPI_HandleTypeDef *handler)
{
	spi_handler = handler;
}

void set_CS_portpin(GPIO_TypeDef *port, uint16_t pin)
{
	if(current_CS_port != NULL) HAL_GPIO_WritePin(current_CS_port, current_CS_pin, GPIO_PIN_SET);
	current_CS_port = port;
	current_CS_pin = pin;
}

void get_CS_portpin(GPIO_TypeDef *port, uint16_t *pin)
{
	port = current_CS_port;
	*pin = current_CS_pin;
}

static inline uint8_t spi_write_register(uint8_t reg_addr, uint8_t * data, uint8_t length)
{
	uint8_t returnVal = 0u;
	reg_addr &= 0x7F; //  MSB = 0 for writing operation

	HAL_GPIO_WritePin(current_CS_port, current_CS_pin, GPIO_PIN_RESET);
	delay_us(3);

	returnVal |= HAL_SPI_Transmit(spi_handler, &reg_addr, 1u, 2u);
	if(!returnVal)
		returnVal |= HAL_SPI_Transmit(spi_handler, data, length, length + 1u);

	HAL_GPIO_WritePin(current_CS_port, current_CS_pin, GPIO_PIN_SET);

	if(returnVal == 3u)
			printf("SPI write timeout!\n");
	else if(returnVal)
		printf("SPI write error!\n");

	return returnVal;
}

static inline uint8_t spi_read_register(uint8_t reg_addr, uint8_t * data, uint8_t length)
{
	uint8_t returnVal = 0u;

#if SPI_SPEEDUP_FOR_SENSOR_DATA
	uint32_t CR1_old = spi_handler->Instance->CR1;
	// Set higher SPI clock freq. for data registers
	if((reg_addr >= MPU9250_ACCEL_XOUT_H) && ((reg_addr + length - 1) <= MPU9250_EXT_SENS_DATA_23)) {
		__IO uint32_t CR1_new = (CR1_old & ~(0x00000007 << 3u)) | SPI_BAUDRATEPRESCALER_8;
		WRITE_REG(spi_handler->Instance->CR1, CR1_new);
	}
#endif

	reg_addr |= 0x80; //  MSB = 1 for reading operation

	HAL_GPIO_WritePin(current_CS_port, current_CS_pin, GPIO_PIN_RESET);
	uint8_t dummy_buffer[30] = {0};
	delay_us(3);

	returnVal |= HAL_SPI_Transmit(spi_handler, &reg_addr, 1u, 2u);
	if(!returnVal) {
		if(length <= 30)	{
			returnVal |= HAL_SPI_TransmitReceive(spi_handler, dummy_buffer, data, length, length + 1u);
		}
		else {
			uint8_t temp_length = 30;
			do
			{
				returnVal |= HAL_SPI_TransmitReceive(spi_handler, dummy_buffer, data, temp_length, temp_length + 1u);

				if(!returnVal) {
					length -= temp_length;
					if(length >= 30)
						temp_length = 30;
					else
						temp_length = length % 30;
				}
				else
					break;

			} while(length > 0);
		}
	}
	else
		printf("SPI read init error!\n");

	HAL_GPIO_WritePin(current_CS_port, current_CS_pin, GPIO_PIN_SET);

#if SPI_SPEEDUP_FOR_SENSOR_DATA
	// Restore SPI clock freq. for data registers
	if((reg_addr >= MPU9250_ACCEL_XOUT_H) && ((reg_addr + length - 1) <= MPU9250_EXT_SENS_DATA_23)) {
		WRITE_REG(spi_handler->Instance->CR1, CR1_old);
	}
#endif

	if(returnVal == 3u)
		printf("SPI read timeout!\n");
	else if(returnVal)
		printf("SPI read error!\n");

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
            byte = 0x80;
            returnVal |= spi_write_register( MPU9250_I2C_SLV4_CTRL, &byte, 1 );
            HAL_Delay(20u);
            returnVal |= spi_read_register( MPU9250_I2C_SLV4_DI, &(data[i]), 1 );
        }
    }
    else
    {
    	returnVal |= spi_read_register( reg_addr, data, length );
    }

    return returnVal;
}
#endif

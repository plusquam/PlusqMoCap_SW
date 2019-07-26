/*
 * stm32_utils.c
 *
 *  Created on: Jul 23, 2019
 *      Author: plusq
 */

#include "stm32_utils.h"
#include "main.h"

int getMsWrapper(unsigned long *count)
{
	*count = HAL_GetTick();
	return 0;
}

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

#include <stdlib.h>
#include <math.h>
// prints a number with 2 digits following the decimal place
// creates the string backwards, before printing it character-by-character from
// the end to the start
//
// Usage: myPrintf(270.458)
//  Output: 270.45
void ftoa(float fVal, char *string)
{
    int16_t dVal;
    uint16_t dec;
	int8_t i;
    char backString[20];

    fVal += 0.0005;   // added after a comment from Matt McNabb, see below.

    dVal = fVal;
    dVal = abs(dVal);

    {
    	float tempDec = (float)fabs(fVal) * 1000.0f;
    	dec = (uint16_t)((uint32_t)(tempDec) % 1000);
    }

    backString[0] = (dec % 10) + '0';
    dec /= 10;
    backString[1] = (dec % 10) + '0';
    backString[2] = (dec / 10) + '0';
    backString[3] = '.';

    i = 4;
    if(dVal == 0)
    {
    	backString[i] = '0';
    	i++;
    }
    else
		while (dVal > 0)
		{
			backString[i] = (dVal % 10) + '0';
			dVal /= 10;
			i++;
		}

    if(fVal < 0.0f)
    {
    	backString[i] = '-';
    	i++;
    }

    --i;
    uint8_t k;
    for(k = 0u; i >= 0; k++, i--)
    {
    	string[k] = backString[i];
    }

    string[k] = '\0';
}

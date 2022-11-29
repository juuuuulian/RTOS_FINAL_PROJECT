/*
 * utility.c
 *
 *  Created on: Oct 6, 2022
 *      Author: julian
 */

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "shell.h"

char *strncpy(char *destination, const char *source, int num) {
    char *ret = destination;

    while (*source && num-- > 0) {
        *destination++ = *source++;
    }

    while (num-- > 0) {
        *destination++ = '\0';
    }

    return ret;
}

void itoa_h(uint32_t num)
{
    char table[16] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
    char buff[9];
    uint32_t indx = 0;
    uint8_t offset = 28;
    uint8_t i;
    for( i = 0; i < 8; i++)
    {
        indx = (num & (0xF << offset));
        indx = indx >> offset;
        buff[i] = table[indx];
        offset -= 4;
    }
    buff[8] = '\0';

    putsUart0("0x");
    putsUart0(buff);
    putsUart0("\n");

}

void itoa_s(uint32_t num)
{
    uint8_t len = 0;
    uint32_t numx = num;
    uint32_t divisor = 1;
    uint8_t i;
    char buff[11];
    while(numx != 0)
    {
        len++;
        numx = numx / 10;
        divisor *= 10;
    }
    divisor /= 10;
    for (i = 0; i < len; i++)
    {
        numx = num / divisor;
        num -= (numx * divisor);
        buff[i] = numx + '0';
        divisor /= 10;
    }
    buff[len] = '\0';
    putsUart0(buff);
    putsUart0("\n");

}
/*
 * reentrant atoi function
 * (*p) - '0' is subtracting value of char '0' from char pointed to be p
 * this turns it into a number
 */
uint32_t r_atoi(char *ptr) {
    int value = 0;
    while (*ptr) {
        value = (value * 10) + (*ptr) - '0';
        ptr++;
     }
     return value;
}

/*
 * Compares two strings and return <0, 0, or > 0
 * CASE INSENSITIVE
 */
int strCmp (const char *str1, const char *str2)
{
    while ( *str1 && ( (*str1 == *str2)  || (*str1+32 == *str2) || (*str1-32 == *str2) ))
    {
        str1++;
        str2++;
    }
    return *(const unsigned char*)str1 - *(const unsigned char*)str2;
}




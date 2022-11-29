/*
 * shell.h
 *
 *  Created on: Sep 27, 2022
 *      Author: julian
 */

#ifndef SHELL_H_
#define SHELL_H_

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration: - Stack size

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdbool.h>
#include <stdint.h>

#include "clock.h"
#include "uart0.h"
#include "gpio.h"

// UI information
#define MAX_CHARS 80
#define MAX_FIELDS 5

// LEDS
#define _RED_LED PORTF,1

// Struct for holding parsed data from user
typedef struct _USER_DATA
{
    char buffer[16];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
    char buf[16];
} USER_DATA;

extern void putcUart0(char c);
extern void putsUart0(char* str);
extern char getcUart0(void);

//void initShell(void);

void getsUart0 (USER_DATA *data);
void parseFields (USER_DATA *data);
char *getFieldString (USER_DATA *data, uint8_t fieldNumber);
int32_t getFieldInteger (USER_DATA *data, uint8_t fieldNumber);
bool isCommand (USER_DATA *data, const char strCommand[], uint8_t minArguements);

//void ps(void);
//void ipcs(void);
//void kill(uint32_t pid, char *str);
//void pmap(uint32_t pid, char *str);
//void preempt(bool toggle);
//void sched(bool toggle);
//void pidof(const char name[]);


#endif /* SHELL_H_ */

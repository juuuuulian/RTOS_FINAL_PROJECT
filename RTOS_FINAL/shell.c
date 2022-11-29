/*
 * main.c
 *
 *  Created on: Aug 30, 2022
 *      Author: julian schneider
 */

#include "shell.h"
#include "tm4c123gh6pm.h"
#include "utility.h"

// UI information
#define MAX_CHARS 80
#define MAX_FIELDS 5

// LEDS
#define _RED_LED PORTF,1

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

/*
   Function to receive chars from the UI, processing special chars such as backspace
   and writing the resultant string into the buffer
   Backspace = 8, DEL = 127
*/

void getsUart0(USER_DATA *data)
{
    char c;
    uint8_t count = 0;

    while (true)
    {
        c = getcUart0();                                 // get a char and put in buffer
        if ((c == 8 || c == 127) && (count > 0))         // remove backspace char and check if backspace is the first char
        {
            count--;
        }
        else if (c == 13 || c == 10)                     // check if <enter key> was pressed
        {
            data->buffer[count] = '\0';
            break;
        }
        else if (c >= 32)                                // check if <space> or any printable char is pressed
        {
            data->buffer[count++] = c;

            if (count == MAX_CHARS)                      // program will exit if max char are input
            {
                data->buffer[count] = '\0';
                break;
            }
        }
    }
}

void parseFields (USER_DATA *data)
{
    // alpha A : 65 , Z : 90 , a : 97 , z : 122
    // numeric 0 : 48 , 9 : 57
    // everything else is a delimiter

    char previous = 'd';

    data->fieldCount = 0;

    uint8_t count = 0;
    uint8_t index = 0;

    while (data->buffer[count] != '\0')
    {
        char c = data->buffer[count];

        // exit the loop if we already have our max fields
        if ( data->fieldCount == MAX_FIELDS )
        {
            break;
        }

        // check if it's an alpha
        //&& ( (previous == 'd') || (previous == 'n') )
        else if ( ( (c >= 65 && c <= 90) || (c >= 97 && c <= 122) ) )
        {
            if (previous == 'a')
            {
                count++;
                continue;
            }
            data->fieldCount++;
            data->fieldType[index] = 'a';
            data->fieldPosition[index++] = count;
            previous = 'a';
        }

        //check if its numeric
        // || (previous == 'a') && ( (previous == 'd') )
        else if ( (c >= 48 && c <= 57)|| (c == 45) )
        {
            if (previous == 'n')
            {
                count++;
                continue;
            }
            data->fieldCount++;
            data->fieldType[index] = 'n';
            data->fieldPosition[index++] = count;
            previous = 'n';
        }

        // otherwise, it's a delimiter
        else
        {
            previous = 'd';
            data->buffer[count] = '\0';
        }

        count++;
    }
}

/*
 *  Returns the value of a field requested if the field
 *  is in range or NULL otherwise.
 *  returns the address of
 */
char *getFieldString (USER_DATA *data, uint8_t fieldNumber)
{
    if (fieldNumber <= data->fieldCount)
    {
        return &data->buffer[data->fieldPosition[fieldNumber]];
    }
    else
    {
        return '\0';
    }
}

/*
 * Function to return the integer value of the field if the
 * field number is in range and the field type is numeric or 0 otherwise.
 */
int32_t getFieldInteger (USER_DATA *data, uint8_t fieldNumber)
{
    if ( (fieldNumber <= data->fieldCount) && (data->fieldType[fieldNumber] == 'n') )
    {
        return r_atoi( &data->buffer[ data->fieldPosition[ fieldNumber ] ] );
    }
    else
    {
        return 0;
    }
}

 /*
  * Returns true if the command matches the first field
  * and the number of arguments (excluding the command field) is greater
  * than or equal to the requested number of minimum arguments.
  */
bool isCommand (USER_DATA *data, const char strCommand[], uint8_t minArguements)
{
    uint8_t fieldNums = data->fieldCount;

    if (fieldNums-1 >= minArguements && ( strCmp(data->buffer, strCommand) == 0))
    {
        return true;
    }
    else
    {
        return false;
    }
}
/*
 * Displays the process (thread) status
 */
//void ps(void)
//{
//    putsUart0("PS Called\n");
//}

/*
 * Displays the inter-process (thread) communication status
 */
//void ipcs(void)
//{
//    putsUart0("IPCS called\n");
//}
//
///*
// * Kills the process (thread) with the matching PID
// */
//void kill(uint32_t pid, char *str)
//{
//    putsUart0(str);
//    putsUart0(" killed\n");
//}
//
///*
// * Displays memory usage by the process (thread) with matching PID
// */
//void pmap(uint32_t pid, char *str)
//{
//    putsUart0("Memory usage by ");
//    putsUart0(str);
//    putcUart0('\n');
//}
//
///*
// * Turns preemption on or off
// */
//void preempt(bool toggle)
//{
//    toggle ? putsUart0("Preempt on\n") : putsUart0("Preempt off\n");
//}
//
///*
// * Selected priority or round-robin scheduling
// */
//void sched(bool toggle)
//{
//    toggle ? putsUart0("Schedule -> Priority\n") : putsUart0("Schedule -> Round-Robin\n");
//}
//
///*
// * Displays the PID of the process (thread)
// */
//void pidof(const char name[])
//{
//    uint8_t i;
//    for (i = 0; name[i] != '\0'; i++)
//    {
//        putcUart0(name[i]);
//    }
//    putsUart0(" Launched\n");
//}

/*
 * Runs the selected program in the background
 */
//
//void initShell(void)
//{
//    // Initialize system clock to 40 MHz
//    initSystemClockTo40Mhz();
//    enablePort(PORTF);
//    selectPinPushPullOutput(_RED_LED);
//    initUart0();
//    // Setup UART0 baud rate
//    setUart0BaudRate(115200, 40e6);
//
//    // structure for parsing user input
//    USER_DATA data;
//    uint32_t pid;
//    bool preemptToggle;
//    bool schedToggle;
//    char *name;
//
//    putsUart0("> ");
//
//    while(1)
//    {
//        if (kbhitUart0())
//        {
//            getsUart0(&data);
//            parseFields(&data);
//
//            // command evaluation
//            if (isCommand(&data, "reboot", 0))
//            {
//                putsUart0("Rebooting...\n");
//                //waitMicrosecond(10000); If not rebooting, uncomment this line
//                NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
//            }
//
//            else if (isCommand(&data, "ps", 0))
//            {
//                ps();
//            }
//
//            else if (isCommand(&data, "ipcs", 0))
//            {
//                ipcs();
//            }
//
//            else if (isCommand(&data, "kill", 0))
//            {
//                pid = getFieldInteger(&data, 1);
//                kill(pid, getFieldString(&data, 1));
//            }
//
//            else if (isCommand(&data, "pmap", 0))
//            {
//                pid = getFieldInteger(&data, 1);
//                pmap(pid, getFieldString(&data, 1));
//            }
//
//            else if (isCommand(&data, "preempt", 0))
//            {
//                if (strCmp(getFieldString(&data, 1) , "on") == 0)
//                {
//                    preemptToggle = true;
//                }
//                else
//                {
//                    preemptToggle = false;
//                }
//                preempt(preemptToggle);
//            }
//
//            else if (isCommand(&data, "sched", 0))
//            {
//                if (strCmp(getFieldString(&data, 1) , "prio") == 0)
//                {
//                    schedToggle = true;
//                }
//                else if (strCmp(getFieldString(&data, 1) , "rr") == 0)
//                {
//                    schedToggle = false;
//                }
//                sched(schedToggle);
//            }
//
//            else if (isCommand(&data, "pidof", 0))
//            {
//                name = getFieldString(&data, 1);
//                pidof(name);
//            }
//
//            else if (isCommand(&data, "run", 0))
//            {
//                name = getFieldString(&data, 1);
//                setPinValue(_RED_LED, 1);
//            }
//            putsUart0("> ");
//            getcUart0();    // Clears the buffer
//        }
//    }
//}

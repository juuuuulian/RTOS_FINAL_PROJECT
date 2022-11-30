// RTOS Framework - Fall 2022

// Student Name: Julian Schneider
// Please do not change any function name in this code or the thread priorities

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 6 Pushbuttons and 5 LEDs, UART
// LEDs on these pins:
// Blue:   PF2 (on-board)
// Red:    PE0 (lengthy and general)
// Orange: PA2 (idle)
// Yellow: PA3 (oneshot and general)
// Green:  PA4 (flash4hz)
// PBs on these pins
// PB0:    PD6 (set red, toggle yellow)
// PB1:    PD7 (clear red, post flash_request semaphore)
// PB2:    PC4 (restart flash4hz)
// PB3:    PC5 (stop flash4hz, uncoop)
// PB4:    PC6 (lengthy priority increase)
// PB5:    PC7 (errant)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1
// Memory Protection Unit (MPU):
//   Region to allow peripheral access (RW) or a general all memory access (RW)
//   Region to allow flash access (XRW)
//   Regions to allow 32 1KiB SRAM access (RW or none)
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "clock.h"
#include "gpio.h"
#include "uart0.h"
#include "wait.h"
// TODO: Add header files here for your strings functions, ...
#include "utility.h"
#include "shell.h"

#define BLUE_LED   PORTF,2 // on-board blue LED
#define RED_LED    PORTE,0 // off-board red LED
#define ORANGE_LED PORTA,2 // off-board orange LED
#define YELLOW_LED PORTA,3 // off-board yellow LED
#define GREEN_LED  PORTA,4 // off-board green LED
#define PB_0 PORTD,6
#define PB_1 PORTD,7
#define PB_2 PORTC,4
#define PB_3 PORTC,5
#define PB_4 PORTC,6
#define PB_5 PORTC,7

#define MAX_PRIORITY_LEVEL 7

#define YIELD_SVC 7
#define SLEEP_SVC 9
#define WAIT_SVC 11
#define POST_SVC 13
#define MALLOC_SVC 15
#define REBOOT_SVC 17
#define IPCS_SVC 19
#define PS_SVC 21
#define CHANGE_SCHEDULE_SVC 23
#define SET_THREAD_PRIORITY_SVC 25
#define PID_SVC 27
#define PMAP_SVC 29
#define PIDOF_SVC 31
#define KILL_SVC 33
#define CHANGE_PREEMPTION_SVC 35

extern void svcYield(void);
extern void svcSleep(void);
extern void svcWait(void);
extern void svcPost(void);
extern void svcMalloc(void);
extern void svcReboot(void);
extern void svcIPCS(void);
extern void svcPS(void);
extern void svcChangeSchedule(void);
extern void svcSetThreadPriority(void);
extern void svcPID(void);
extern void svcPMAP(void);
extern void svcPidOf(void);
extern void svcKill(void);
extern void svcChangePreemption(void);

void accessWindow(uint32_t baseAdd, uint32_t size_in_bytes);
uint32_t calculateSubregionMask(uint32_t baseAdd, uint32_t size_in_bytes);

extern uint32_t __STACK_TOP;
extern void setASPBit(void);
extern void setPSP(uint32_t* stack);
extern uint32_t* getPSP(void);
extern uint32_t* getMSP(void);
extern void setPrivilegeOff(void);
extern void pushContext(void);
extern void popContext(void);
extern void thatDummyStack(void*);

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

bool preemptionFlag = 1;
bool priorityFlag = 1;
bool ping = 1;
uint64_t totalTime = 0;

uint32_t *brk = (uint32_t *) 0x20008000;        // Moving the brk to the top of heap
uint32_t *heap = (uint32_t *) 0x20001400;        // heap at 20001400 which gives room in SRAM for OS

// function pointer
typedef void (*_fn)();

// semaphore
#define MAX_SEMAPHORES 5
#define MAX_QUEUE_SIZE 5

typedef struct _semaphore
{
    uint16_t count;     // number of available resources
    uint16_t queueSize; // how many tasks waiting
    uint32_t processQueue[MAX_QUEUE_SIZE]; // store task index here
} semaphore;
semaphore semaphores[MAX_SEMAPHORES];

#define keyPressed 1
#define keyReleased 2
#define flashReq 3
#define resource 4

// task
#define STATE_INVALID    0 // no task
#define STATE_UNRUN      1 // task has never been run
#define STATE_READY      2 // has run, can resume at any time
#define STATE_DELAYED    3 // has run, but now awaiting timer
#define STATE_BLOCKED    4 // has run, but now blocked by semaphore

#define MAX_TASKS 12       // maximum number of valid tasks
uint8_t taskCurrent = 0;   // index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks

// REQUIRED: add store and management for the memory used by the thread stacks
//           thread stacks must start on 1 kiB boundaries so mpu can work correctly

struct _tcb
{
    uint8_t state;                 // see STATE_ values above
    void *pid;                     // used to uniquely identify thread
    void *spInit;                  // original top of stack
    void *sp;                      // current stack pointer
    int8_t priority;               // 0=highest to 7=lowest
    uint8_t initPriority;          // initial priority value
    uint32_t ticks;                // ticks until sleep complete
    uint32_t srd;                  // MPU subregion disable bits (one per 1 KiB)
    char name[16];                 // name of task used in ps command
    uint64_t time;
    uint32_t size;
    void *semaphore;               // pointer to the semaphore that is blocking the thread
} tcb[MAX_TASKS];

typedef struct _PROGRAM_STATUS
{
    uint8_t state;
    uint16_t pid;
    uint8_t priority;
    char processName[16];
    uint64_t time;
    uint64_t totalTimeSpent;
    uint32_t address;
    uint32_t size;
} programStatus;

typedef struct _PROGRAM_MAP
{
    uint16_t pid;
    char processName[16];
    uint32_t address;
    uint32_t size;
    uint8_t stackOrHeap;
} programMap;

//-----------------------------------------------------------------------------
// Memory Manager and MPU Functions
//-----------------------------------------------------------------------------

// TODO: add your malloc code here and update the SRD bits for the current thread

void *alloc(uint32_t size_in_bytes)
{
    void *p = brk;
    size_in_bytes = (((size_in_bytes - 1)/1024) + 1) * (1024);
    brk = (uint32_t)brk - (uint32_t)(size_in_bytes);

    if ((uint32_t) brk < (0x200013FF + 1))
    {
        brk = '\0';
    }
    putsUart0("Address from malloc: ");
    itoa_h((uint32_t) brk);
    return p;
}

void * mallocFromHeap(uint32_t size_in_bytes)
{
    svcMalloc();
}

// REQUIRED: add your MPU functions here
void allowFlashAccess(void)
{
    // Memory Region Number 1 (Flash Region)
    // for size : log_2(256 * 1024) = 18 - 1 = 17 = 0x11 << 1= 0x22
    NVIC_MPU_NUMBER_R = 1;   //region 1
    NVIC_MPU_BASE_R = NVIC_MPU_BASE1_VALID | 0x1;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_ENABLE | 0x22 | 0x3000000;    // 0x11 << 1 for size
}
void setupSramAccess(void)
{
    // Memory Region Number 2 -> SRAM Region 0 -> Subregions 0-7
    // Base Address = 0x20000000, AP field = 0x1000000 for RW Priv only
    // Size = log2( 8 * 1024 ) = 13 - 1 = 12 = 0xC << 1 = 0x18
    NVIC_MPU_NUMBER_R = 2;
    NVIC_MPU_BASE_R = NVIC_MPU_BASE1_VALID | 0x20000000 | 0x2;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | 0x18 | 0x1000000 | NVIC_MPU_ATTR_ENABLE; // 0xD << 1 moves 0xD into size register

    // Memory Region Number 3 -> SRAM Region 1 -> Subregions 8-15
    NVIC_MPU_NUMBER_R = 3;
    NVIC_MPU_BASE_R = NVIC_MPU_BASE1_VALID | 0x20002000 | 0x3;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | 0x18 | 0x1000000 | NVIC_MPU_ATTR_ENABLE;

    // Memory Region Number 4 -> SRAM Region 2 -> Subregions 16-23
    NVIC_MPU_NUMBER_R = 4;
    NVIC_MPU_BASE_R = NVIC_MPU_BASE1_VALID | 0x20004000 | 0x4;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | 0x18 | 0x1000000 | NVIC_MPU_ATTR_ENABLE;

    // Memory Region Number 5 -> SRAM Region 3 -> Subregions 24-31
    NVIC_MPU_NUMBER_R = 5;
    NVIC_MPU_BASE_R = NVIC_MPU_BASE1_VALID | 0x20006000 | 0x5;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_XN | 0x18 | 0x1000000 | NVIC_MPU_ATTR_ENABLE;
}
void setupBackgroundRegion()
{
    // Rw/Rw 4G size
    NVIC_MPU_NUMBER_R = 0;
    NVIC_MPU_BASE_R = NVIC_MPU_BASE1_VALID | 0x0;
    NVIC_MPU_ATTR_R = NVIC_MPU_ATTR_BUFFRABLE | NVIC_MPU_ATTR_SIZE_M | NVIC_MPU_ATTR_XN | 0x3000000 | NVIC_MPU_ATTR_ENABLE;
}
void enableFaultHandlers(uint32_t mask)
{
    NVIC_SYS_HND_CTRL_R |= mask;
    NVIC_CFG_CTRL_R |= NVIC_CFG_CTRL_DIV0 | NVIC_CFG_CTRL_UNALIGNED;
}
void hardFaultStat(void)
{
    if (NVIC_HFAULT_STAT_R & NVIC_HFAULT_STAT_DBG)
    {
        putsUart0("Hard Fault triggered with a Debug Event\n");
    }
    if (NVIC_HFAULT_STAT_R & NVIC_HFAULT_STAT_FORCED)
    {
        putsUart0("Hard Fault triggered with a Forced Hard Fault\n");
    }
    if(NVIC_HFAULT_STAT_R & NVIC_HFAULT_STAT_VECT)
    {
        putsUart0("Hard Fault triggered with a Vector Stable Read Fault\n");
    }
}

void printOffendingInstruction(void)
{
    uint32_t *pspVar = getPSP();
    itoa_h(*(pspVar + 6));
}

void dumpRegisters(void)
{
    uint32_t *pspVar = getPSP();
    putsUart0("R0: ");
    itoa_h((uint32_t)*(pspVar + 0));
    putsUart0("R1: ");
    itoa_h(*(pspVar + 1));
    putsUart0("R2: ");
    itoa_h(*(pspVar + 2));
    putsUart0("R3: ");
    itoa_h(*(pspVar + 3));
    putsUart0("R12: ");
    itoa_h(*(pspVar + 4));
    putsUart0("LR: ");
    itoa_h(*(pspVar + 5));
    putsUart0("PC: ");
    itoa_h(*(pspVar + 6));
    putsUart0("xPSR: ");
    itoa_h(*(pspVar + 7));
}

// REQUIRED: initialize MPU here
void initMpu(void)
{
    // REQUIRED: call your MPU functions here
    setupBackgroundRegion();
    allowFlashAccess();
    setupSramAccess();
    NVIC_MPU_CTRL_R |= NVIC_MPU_CTRL_ENABLE;
}

//-----------------------------------------------------------------------------
// RTOS Kernel Functions
//-----------------------------------------------------------------------------

// REQUIRED: initialize systick for 1ms system timer
void initRtos()
{
    uint8_t i;
    // no tasks running
    taskCount = 0;
    // clear out tcb records
    for (i = 0; i < MAX_TASKS; i++)
    {
        tcb[i].state = STATE_INVALID;
        tcb[i].pid = 0;
    }
}

//TODO REQUIRED: Implement prioritization to 8 levels
int rtosScheduler()
{
    bool ok;
    static uint8_t task = 0xFF;
    ok = false;
    static uint8_t previousTasksArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};    // holds the previously ran task for each priority level in the cake
    uint8_t highestPriorityReached = MAX_PRIORITY_LEVEL;                // keeps track of the highest priority found in the schedule

    if(priorityFlag)
    {
        previousTasksArray[tcb[taskCurrent].priority] = taskCurrent;    // placing the previously ran task array at the priority level it ran at

        highestPriorityReached = MAX_PRIORITY_LEVEL;                    // initialized with the max priority of level of 7

        for(task = 0; task < MAX_TASKS; task++)                         // go through each task in tcb until you find a task with lower priority number (More Important)
        {
            if((tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN) && (tcb[task].priority < highestPriorityReached))
            {
                highestPriorityReached = tcb[task].priority;            // Once you find the task with a superior priority (lower number), set it to the highest reached priority
            }
            if(highestPriorityReached == 0)
            {
                break;
            }

        }

        task = previousTasksArray[highestPriorityReached];              // set the task lined up to schedule to the next highest priority task at that level

        while(!ok)
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;

            if((tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN) && (tcb[task].priority == highestPriorityReached))
            {
                ok = true;      // if we found the task with the priority with the highest priority then we break out of the loop
                //break;
            }

        }
    }

    else
    {
        while(!ok)              // Round Robin Scheduling
        {
            task++;
            if (task >= MAX_TASKS)
                task = 0;
            ok = (tcb[task].state == STATE_READY || tcb[task].state == STATE_UNRUN);
        }
    }
    return task;
}



bool createThread(_fn fn, const char name[], uint8_t priority, uint32_t stackBytes)
{
    bool ok = false;
    uint8_t i = 0;
    bool found = false;
    // REQUIRED:
    // store the thread name
    // allocate stack space and store top of stack in sp and spInit
    // add task if room in task list
    if (taskCount < MAX_TASKS)
    {
        // make sure fn not already in list (prevent reentrancy)
        while (!found && (i < MAX_TASKS))
        {
            found = (tcb[i++].pid == fn);
        }
        if (!found)
        {
            // find first available tcb record
            i = 0;
            while (tcb[i].state != STATE_INVALID) {i++;}
            tcb[i].state = STATE_UNRUN;
            tcb[i].pid = fn;
            tcb[i].spInit = alloc(stackBytes);     //sets the initial stack value, so it can return to this point on task switch
            tcb[i].sp = tcb[i].spInit;
            strncpy(tcb[i].name, name, 16);
            tcb[i].priority = priority;
            tcb[i].initPriority = priority;
            tcb[i].srd = calculateSubregionMask((uint32_t)tcb[i].sp - 1, stackBytes);
            tcb[i].size = stackBytes;
            //tcb[i].srd = calculateSubregionMask((uint32_t)tcb[i].sp, stackBytes);
            // increment task count
            taskCount++;
            ok = true;
        }
    }
    // REQUIRED: allow tasks switches again
    return ok;
}

// REQUIRED: modify this function to restart a thread
void restartThread(_fn fn)
{

}

// REQUIRED: modify this function to stop a thread
// REQUIRED: remove any pending semaphore waiting
// NOTE: see notes in class for strategies on whether stack is freed or not
void stopThread(_fn fn)
{

}

// REQUIRED: modify this function to set a thread priority
void setThreadPriority(_fn fn, uint8_t priority)
{
    svcSetThreadPriority();
}

bool createSemaphore(uint8_t semaphore, uint8_t count)
{
    bool ok = (semaphore < MAX_SEMAPHORES);
    {
        semaphores[semaphore].count = count;
    }
    return ok;
}

// REQUIRED: modify this function to start the operating system
// by calling scheduler, setting PSP, ASP bit, TMPL bit, and PC
void startRtos()
{
    taskCurrent = rtosScheduler();
    setPSP((uint32_t*) tcb[taskCurrent].sp - 1);                 // Setting SP for the function we're calling
    setASPBit();

    accessWindow((uint32_t) tcb[taskCurrent].sp - 1, 0x400);       // -1 because the actual memory is Zero based, 0x00000000 - 0x20007FFC
    tcb[taskCurrent].state = STATE_READY;
    _fn fn = (_fn) tcb[taskCurrent].pid;                          // cast to pnt to function, setting fn to address of the idle function

    NVIC_ST_RELOAD_R = 39999;   // for a 1kHz rate
    NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN | NVIC_ST_CTRL_ENABLE;

    // Starting the timer before first task gets executed
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
    setPrivilegeOff();
    fn();
    while(1);
}

// REQUIRED: modify this function to yield execution back to scheduler using pendsv
void yield()
{
    svcYield();
}

// REQUIRED: modify this function to support 1ms system timer
// execution yielded back to scheduler until time elapses using pendsv
void sleep(uint32_t tick)
{
    svcSleep();
}

// REQUIRED: modify this function to wait a semaphore using pendsv
void wait(int8_t semaphore)
{
    svcWait();
}

// REQUIRED: modify this function to signal a semaphore is available using pendsv
void post(int8_t semaphore)
{
    svcPost();
}

// REQUIRED: modify this function to add support for the system timer
// REQUIRED: in preemptive code, add code to request task switch
void systickIsr()
{
    uint8_t i;
    for (i = 0; i < MAX_TASKS; i++)
    {
        if (tcb[i].state == STATE_DELAYED)
        {
            tcb[i].ticks--;

            if (tcb[i].ticks == 0)
            {
                tcb[i].state = STATE_READY;
            }
        }
    }
    if (preemptionFlag && taskCount > 0)
    {
        NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
    }
}

// REQUIRED: in coop and preemptive, modify this function to add support for task switching
// REQUIRED: process UNRUN and READY tasks differently
void pendSvIsr()
{
    uint8_t regionNum, i;

    if (NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_DERR)
    {
        NVIC_FAULT_STAT_R &= ~NVIC_FAULT_STAT_DERR;
        putsUart0("Data access violation called by the MPU\n");
    }

    if (NVIC_FAULT_STAT_R & NVIC_FAULT_STAT_IERR)
    {
        NVIC_FAULT_STAT_R &= ~NVIC_FAULT_STAT_IERR;
        putsUart0("Instruction access violation called by the MPU\n");
    }

    // Now we must save the other registers (R4-R11) so that we can eventually
    // return back to our task in the same state that we left.
    pushContext();
    tcb[taskCurrent].sp = getPSP();             // This updates the tasks stack pointer to reflect the current stack now that things are added


    tcb[taskCurrent].time = TIMER1_TAV_R;


    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;

    TIMER1_TAV_R = 0;

    TIMER1_CTL_R |= TIMER_CTL_TAEN;

    taskCurrent = (uint32_t) rtosScheduler();   // Now that state info is saved we can begin scheduling the new task

/*
    2 options remain ...
    Option 1: task is READY, which means at this point in the program we can assume that the task has already ran.
              Since the task has already ran, we must restore the context of the new(-ish) task by loading in it's
              values from it's stack. To do this we set our PSP to the new tasks Stack Pointer that it left off at,
              and POP the context R11-R4.
              Upon returning pendsvIsr, the hardware will pop the remaining registes xpsr, PC, LR, r12, r3-r0.
    ---------------------------------------------------------------------------------------------------------------
    Option 2: task is UNRUN, which means we need to set the PSP to the new tasks stack, then we set up the new task
              to be ran. To do that we must fill in special values into xPSR, LR, and the pointer to our function into
              PC. Then we need to write dummy values into R12, R3-R0. This is done because the hardware is expecting
              to have those values filled in. Upon returning from PendSVIsr the hard will will POP (restore) those values
              thus causing a task switch because we forced those values in, without the hardware knowing it. The reason
              we don't save R4-R11, is because there is no context to be saved since the task has never ran.
 */
    if(tcb[taskCurrent].state == STATE_READY)
    {
        setPSP(tcb[taskCurrent].sp);
        popContext();
    }

    else if (tcb[taskCurrent].state == STATE_UNRUN)
    {
        tcb[taskCurrent].state = STATE_READY;
        //itoa_h((uint32_t)tcb[taskCurrent].spInit);
        setPSP((uint32_t*)tcb[taskCurrent].spInit);
        //itoa_h((uint32_t)getPSP());
        thatDummyStack(tcb[taskCurrent].pid);
        //itoa_h((uint32_t)getPSP());
    }

    for (i = 0; i < 4; i++)
    {
        regionNum = 2 + i;
        NVIC_MPU_NUMBER_R = regionNum;
        NVIC_MPU_ATTR_R &= ~(0x0000FF00);
        NVIC_MPU_ATTR_R |= ((uint32_t)tcb[taskCurrent].srd >> (i * 8) & (0xFF)) << 8;
    }
    // xPSR and then push manually the rest of the registers
}

// REQUIRED: modify this function to add support for the service call
// REQUIRED: in preemptive code, add code to handle synchronization primitives
void svCallIsr()
{
    char *arr;
    uint8_t priority;
    uint32_t pid;
    uint8_t toggle;
    uint8_t svcNum;
    uint8_t i;
    uint32_t *psp = getPSP();
    uint32_t size_in_bytes;

    // this goes to PC then grabs the first Byte which is the SVC #

    svcNum = *(uint8_t*)( *(psp + 6) - 2);

    switch (svcNum)
    {
        case YIELD_SVC:
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

        case SLEEP_SVC:
            tcb[taskCurrent].ticks = *getPSP();
            tcb[taskCurrent].state = STATE_DELAYED;
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

        case WAIT_SVC:
            // if semaphore not available, block task and put into queue
            if (semaphores[*psp].count > 0)      // *psp is what was passed into wait(some number) for oneshot its 3
            {
                semaphores[*psp].count--;        // no task switch
            }
            else
            {
                if (semaphores[*psp].queueSize < MAX_QUEUE_SIZE)
                {
                    semaphores[*psp].processQueue[semaphores[*psp].queueSize++] = taskCurrent;
                    tcb[taskCurrent].state = STATE_BLOCKED;
                    tcb[taskCurrent].semaphore = &semaphores[*psp];
                    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
                }
            }
            break;

        case POST_SVC:
            semaphores[*psp].count++;
            if (semaphores[*psp].queueSize > 0)                                 // if there are processes in the semaphore Queue
            {                                                                   // mark the next waiting task as ready
                tcb[semaphores[*psp].processQueue[0]].state = STATE_READY;      // This is the next task in the queue

                semaphores[*psp].count--;
                for (i = 0; i < semaphores[*psp].queueSize; i++)
                {
                    semaphores[*psp].processQueue[i] = semaphores[*psp].processQueue[i + 1];      // move them up
                }
                semaphores[*psp].queueSize--;

            }
            NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
            break;

        case MALLOC_SVC:

            size_in_bytes = *psp;
            void *p = heap;
            size_in_bytes = (((size_in_bytes - 1)/1024) + 1) * (1024);
            heap = (uint32_t)heap + (uint32_t)(size_in_bytes);
            calculateSubregionMask((uint32_t)heap, size_in_bytes);
            if ((uint32_t) heap < (0x200013FF + 1))
            {
                heap = '\0';
            }
            putsUart0("Address from heap malloc: ");
            itoa_h((uint32_t) heap);
            //return p;
            break;

        case REBOOT_SVC:
            NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;
            break;

        case IPCS_SVC:

            break;

        case PS_SVC:
        {
            programStatus *ps;
            ps = (programStatus*) *(psp + 1);
            for(i = 0; i < 10; i++)
            {
                ps[i].priority = tcb[i].priority;
                ps[i].pid = (uint32_t)tcb[i].pid;
                strncpy(ps[i].processName, tcb[i].name, 16);
                ps[i].state = tcb[i].state;
                ps[i].time = tcb[i].time;
                totalTime += ps[i].time;
            }
            ps[0].totalTimeSpent = totalTime;
            break;
        }

        case CHANGE_SCHEDULE_SVC:
            toggle = *psp;
            priorityFlag = toggle;
            break;

        case SET_THREAD_PRIORITY_SVC:

            priority = *(psp + 1);
            pid = *(psp);

            for(i = 0; i < MAX_TASKS; i++)
            {
                if(pid == (uint32_t)tcb[i].pid)
                {
                    tcb[i].priority = priority;
                    break;
                }
            }
            break;

        case PMAP_SVC:
        {
            programMap *pm;
            pm = (programMap*) *(psp + 1);
            pid = pm->pid;
            bool found = 0;

            for(i = 0; i < MAX_TASKS; i++)
            {
                if (pid == (uint32_t)tcb[i].pid)
                {
                    found = 1;
                    pm->address = (uint32_t) tcb[i].sp;
                    strncpy(pm->processName, tcb[i].name, 16);
                    pm->size = tcb[i].size;
                    if (strCmp(pm->processName, "lengthyFn") == 0)
                        pm->stackOrHeap = 0;
                    else
                        pm->stackOrHeap = 1;
                }

            }
            if (!found)
                putsUart0("\nNOT FOUND\n");

            break;
        }

        case PIDOF_SVC:
            arr = *(psp);
            for(i = 0; i < MAX_TASKS; i++)
            {
                if (strCmp(arr, tcb[i].name) == 0)
                {
                    pid = (uint32_t) tcb[i].pid;
                    break;
                }
                else
                    pid = 0;
            }
            putsUart0("PID: ");
            itoa_s(pid);
            break;

        case KILL_SVC:

            break;

        case CHANGE_PREEMPTION_SVC:
            toggle = *psp;
            preemptionFlag = toggle;
            break;
    }
}

// REQUIRED: code this function
void mpuFaultIsr()
{
    putsUart0("MPU fault in process ");
    itoa_s((uint32_t)tcb[taskCurrent].pid);
    putsUart0("PSP: ");
    itoa_h( (uint32_t) getPSP());
    putsUart0("MSP: ");
    itoa_h( (uint32_t) getMSP());
    putsUart0("MPU Fault Status Register: ");
    itoa_h(NVIC_FAULT_STAT_R & 0xFF);    // gets the 7-0 bits

    if (NVIC_FAULT_STAT_R & 0x1)
    {
        putsUart0("MPU Fault caused with a Instruction Access Violation\n");
    }
    if (NVIC_FAULT_STAT_R & 0x2)
    {
        putsUart0("MPU Fault caused with a Data Access Violation\n");
    }
    if (NVIC_FAULT_STAT_R & 0x8)
    {
        putsUart0("MPU Fault caused with a Unstack Access Violation\n");
    }
    if (NVIC_FAULT_STAT_R & 0x10)
    {
        putsUart0("MPU Fault caused with a Stack Access Violation\n");
    }
    if (NVIC_FAULT_STAT_R & 0x20)
    {
        putsUart0("MPU Fault caused with a Floating-Point Lazy State Preservation\n");
    }
    if (NVIC_FAULT_STAT_R & 0x80)
    {
        putsUart0("MPU Fault caused with a Fault Address Register Valid\n");
    }
    putsUart0("Address of location that generated the fault: ");
    itoa_h(NVIC_MM_ADDR_R);

    putsUart0("Address of instruction that generated the fault: ");
    printOffendingInstruction();

    dumpRegisters();
    NVIC_SYS_HND_CTRL_R &= ~(NVIC_SYS_HND_CTRL_MEMP);
    NVIC_INT_CTRL_R |= NVIC_INT_CTRL_PEND_SV;
}

// REQUIRED: code this function
void hardFaultIsr()
{
    putsUart0("Hard Fault in process PID\n");
    putsUart0("PSP: ");
    itoa_h( (uint32_t) getPSP());
    putsUart0("MSP: ");
    itoa_h( (uint32_t) getMSP());
    putsUart0("Fault Status Register: ");
    itoa_h(NVIC_HFAULT_STAT_R);
    hardFaultStat();
    dumpRegisters();
    while(1);
}

// REQUIRED: code this function
void busFaultIsr()
{
    putsUart0("Bus Fault in process PID\n");
    putsUart0("PSP: ");
    itoa_h( (uint32_t) getPSP());
    putsUart0("MSP: ");
    itoa_h( (uint32_t) getMSP());
    dumpRegisters();
    while(1);
}

// REQUIRED: code this function
void usageFaultIsr()
{
    putsUart0("Usage Fault in process PID\n");
    putsUart0("PSP: ");
    itoa_h( (uint32_t) getPSP());
    putsUart0("MSP: ");
    itoa_h( (uint32_t) getMSP());
    putsUart0("Usage Fault status register: ");
    itoa_h(NVIC_FAULT_STAT_R & 0xFFFF0000);

    if (NVIC_FAULT_STAT_R & 0x2000000)
    {
        putsUart0("Usage Fault cause: Divide-by-Zero Usage fault\n");
    }
    if (NVIC_FAULT_STAT_R & 0x1000000)
    {
        putsUart0("Usage Fault cause: Unaligned Access Usage fault\n");
    }
    if (NVIC_FAULT_STAT_R & 0x80000)
    {
        putsUart0("Usage Fault cause: No Coprocessor Usage fault\n");
    }
    if (NVIC_FAULT_STAT_R & 0x40000)
    {
        putsUart0("Usage Fault cause: Invalid PC Load Usage fault\n");
    }
    if (NVIC_FAULT_STAT_R & 0x20000)
    {
        putsUart0("Usage Fault cause: Invalid State Usage fault\n");
    }
    if (NVIC_FAULT_STAT_R & 0x10000)
    {
        putsUart0("Usage Fault cause: Undefined Instruction Usage fault\n");
    }
    dumpRegisters();
    while(1);
}

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
// REQUIRED: Add initialization for blue, orange, red, green, and yellow LEDs
//           6 pushbuttons
void initHw()
{
    initSystemClockTo40Mhz();
    enablePort(PORTA);
    enablePort(PORTC);
    enablePort(PORTD);
    enablePort(PORTF);
    enablePort(PORTE);

    selectPinPushPullOutput(ORANGE_LED);
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(YELLOW_LED);
    selectPinPushPullOutput(BLUE_LED);

    setPinCommitControl(PB_1);

    selectPinDigitalInput(PB_0);
    selectPinDigitalInput(PB_1);
    selectPinDigitalInput(PB_2);
    selectPinDigitalInput(PB_3);
    selectPinDigitalInput(PB_4);
    selectPinDigitalInput(PB_5);

    enablePinPullup(PB_0);
    enablePinPullup(PB_1);
    enablePinPullup(PB_2);
    enablePinPullup(PB_3);
    enablePinPullup(PB_4);
    enablePinPullup(PB_5);

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                                     // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;                               // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD | TIMER_TAMR_TACDIR;          // configure for periodic mode (count up)
    TIMER1_TAILR_R = TIMER_TAILR_M;                                      // set load value

// 1/1000 = x, x/(1/40,000,000) = 40000 - 1 = 39999

}

// REQUIRED: add code to return a value from 0-63 indicating which of 6 PBs are pressed
uint8_t readPbs()
{
    if (!getPinValue(PB_0))
    {
        return 1;
    }
    else if (!getPinValue(PB_1))
    {
        return 2;
    }
    else if (!getPinValue(PB_2))
    {
        return 4;
    }
    else if (!getPinValue(PB_3))
    {
        return 8;
    }
    else if (!getPinValue(PB_4))
    {
        return 16;
    }
    else if (!getPinValue(PB_5))
    {
        return 32;
    }
    else {
        return 0;
    }
}

//-----------------------------------------------------------------------------
// YOUR UNIQUE CODE
// REQUIRED: add any custom code in this space
//-----------------------------------------------------------------------------

void getData(uint8_t type, void * generic)
{
    switch (type)
    {
        case IPCS_SVC:

           break;

       case PS_SVC:
           svcPS();
           break;

       case PID_SVC:

           break;

       case PMAP_SVC:
           svcPMAP();
           break;

       case PIDOF_SVC:
           svcPidOf();

           break;

       case KILL_SVC:

           break;

    }
}

uint32_t calculateSubregionMask(uint32_t baseAdd, uint32_t size_in_bytes)
{
    uint32_t disableSubregionsMask = 0;
    uint8_t startingSubregionNum = (baseAdd - 0x20000000) / 0x400;
    uint8_t numOfSubregions = size_in_bytes / 0x400;
    //uint8_t regionNum;
    uint8_t i;

    if (size_in_bytes % 0x400 != 0)
    {
        numOfSubregions += 1;
    }
    for (i = 0; i < numOfSubregions; i++)
    {
        disableSubregionsMask |= (1 << (startingSubregionNum + i));
    }

    return disableSubregionsMask;

}

void accessWindow(uint32_t baseAdd, uint32_t size_in_bytes)
{
    uint32_t disableSubregionsMask = 0;
    uint8_t startingSubregionNum = (baseAdd - 0x20000000) / 0x400;
    uint8_t numOfSubregions = size_in_bytes / 0x400;
    uint8_t regionNum;
    uint8_t i;

    if (size_in_bytes % 0x400 != 0)
    {
        numOfSubregions += 1;
    }

    for (i = 0; i < numOfSubregions; i++)
    {
        disableSubregionsMask |= (1 << (startingSubregionNum + i));
    }
    for (i = 0; i < 4; i++)
    {
        regionNum = 2 + i;
        NVIC_MPU_NUMBER_R = regionNum;
        NVIC_MPU_ATTR_R &= ~(0x0000FF00);
        NVIC_MPU_ATTR_R |= (disableSubregionsMask >> (i * 8) & (0xFF)) << 8;
    }
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// one task must be ready at all times or the scheduler will fail
// the idle task is implemented for this purpose
void idle()
{

    while(true)
    {
        setPinValue(ORANGE_LED, 1);
        waitMicrosecond(1000);
        setPinValue(ORANGE_LED, 0);
        yield();
    }
}

//void idle2()
//{
//
//    while(true)
//    {
//        setPinValue(RED_LED, 1);
//        waitMicrosecond(1000);
//        setPinValue(RED_LED, 0);
//        yield();
//    }
//}

void flash4Hz()
{
    while(true)
    {
        setPinValue(GREEN_LED, !getPinValue(GREEN_LED));
        sleep(125);
    }
}

void oneshot()
{
    while(true)
    {
        wait(flashReq);
        setPinValue(YELLOW_LED, 1);
        sleep(1000);
        setPinValue(YELLOW_LED, 0);
    }
}

void partOfLengthyFn()
{
    // represent some lengthy operation
    waitMicrosecond(990);
    // give another process a chance to run
    yield();
}

void lengthyFn()
{
    uint16_t i;
    uint8_t *p;

    // Example of allocating memory from stack
    // This will show up in the pmap command for this thread
    p = mallocFromHeap(1024);
    *p = 0;

    while(true)
    {
        wait(resource);
        for (i = 0; i < 5000; i++)
        {
            partOfLengthyFn();
        }
        setPinValue(RED_LED, !getPinValue(RED_LED));
        post(resource);
    }
}

void readKeys()
{
    uint8_t buttons;
    while(true)
    {
        wait(keyReleased);
        buttons = 0;
        while (buttons == 0)
        {
            buttons = readPbs();
            yield();
        }
        post(keyPressed);
        if ((buttons & 1) != 0)
        {
            setPinValue(YELLOW_LED, !getPinValue(YELLOW_LED));
            setPinValue(RED_LED, 1);
        }
        if ((buttons & 2) != 0)
        {
            post(flashReq);
            setPinValue(RED_LED, 0);
        }
        if ((buttons & 4) != 0)
        {
            restartThread(flash4Hz);
        }
        if ((buttons & 8) != 0)
        {
            stopThread(flash4Hz);
        }
        if ((buttons & 16) != 0)
        {
            setThreadPriority(lengthyFn, 4);
        }
        yield();
    }
}

void debounce()
{
    uint8_t count;
    while(true)
    {
        wait(keyPressed);
        count = 10;
        while (count != 0)
        {
            sleep(10);
            if (readPbs() == 0)
                count--;
            else
                count = 10;
        }
        post(keyReleased);
    }
}

void uncooperative()
{
    while(true)
    {
        while (readPbs() == 8)
        {
        }
        yield();
    }
}

void errant()
{
    uint32_t* p = (uint32_t*)0x20000000;
    while(true)
    {
        while (readPbs() == 32)
        {
            *p = 0;
        }
        yield();
    }
}

void important()
{
    while(true)
    {
        wait(resource);
        setPinValue(BLUE_LED, 1);
        sleep(1000);
        setPinValue(BLUE_LED, 0);
        post(resource);
    }
}

// REQUIRED: add processing for the shell commands through the UART here
void shell()
{
    uint8_t i;
    USER_DATA data;
    uint32_t pidNumber;
    bool preemptToggle;
    bool schedToggle;
    char name[12];
    uint64_t percentage = 0;

    programStatus ps[10] = {0};
    programMap pm;

    putsUart0("\n");
    putsUart0("> ");
    while (true)
    {
        getsUart0(&data);
        parseFields(&data);

        // command evaluation
        if (isCommand(&data, "reboot", 0))
        {
            svcReboot();
        }

        else if (isCommand(&data, "ps", 0))
        {
            getData(PS_SVC, (void *)&ps);
            putsUart0("CPU %: \t");
            putsUart0("  Name: \t\t");
            putsUart0("  PID:     ");
            putsUart0("  Priority: \t");
            putsUart0(" State: \n");
            putsUart0("-------------------------------------------------------\n");
            for(i = 0; i < 10; i++)
            {
                percentage = (ps[i].time * 10000) / ps[0].totalTimeSpent;
                itoa_s_percentage(percentage);
                //itoa_s(ps[i].time);
                putsUart0("    \t");

                if (strCmp(ps[i].processName, "idle") == 0 )
                    putsUart0("\t");

                putsUart0(ps[i].processName);

                putsUart0("     \t");

                if (strCmp(ps[i].processName, "shell") == 0 || strCmp(ps[i].processName, "errant") == 0)
                    putsUart0("\t");

                if (strCmp(ps[i].processName, "idle") == 0 )
                    putsUart0("\t");
                itoa_s((uint32_t)ps[i].pid);
                putsUart0("       \t");


                if (ps[i].priority == 0)
                {
                    putsUart0("0");
                }
                itoa_s(ps[i].priority);
                putsUart0("      \t\t");



                switch (ps[i].state)

                {
                    case STATE_INVALID:
                        putsUart0("STATE_INVALID\n");
                        break;

                    case STATE_UNRUN:
                        putsUart0("STATE_UNRUN\n");
                        break;

                    case STATE_READY:
                        putsUart0("STATE_READY\n");
                        break;

                    case STATE_DELAYED:
                        putsUart0("STATE_DELAYED\n");
                        break;

                    case STATE_BLOCKED:
                        putsUart0("STATE_BLOCKED\n");
                        break;
                }
                //putsUart0("\n");

            }
        }

        else if (isCommand(&data, "ipcs", 0))
        {
            //ipcs();
        }

        else if (isCommand(&data, "kill", 0))
        {
            pidNumber = getFieldInteger(&data, 1);
            //kill(pidNumber, getFieldString(&data, 1));
        }

        else if (isCommand(&data, "pmap", 0))
        {
            pidNumber = getFieldInteger(&data, 1);
            pm.pid = pidNumber;
            getData(PMAP_SVC, (void *)&pm);

            putsUart0("Name: \t");
            putsUart0("  Address: \t\t");
            putsUart0("  Size:     \t");
            putsUart0("  Stack vs Heap: \n");
            putsUart0("-------------------------------------------------------\n");
            putsUart0(pm.processName);
            putsUart0("     \t");
            itoa_h_pmap(pm.address);
            putsUart0("     \t");
            itoa_s(pm.size);
            putsUart0("           \t");
            if(pm.stackOrHeap)
            {
                putsUart0("Stack\n");
            }
            else
                putsUart0("Heap\n");


        }

        else if (isCommand(&data, "preempt", 0))
        {
            if (strCmp(getFieldString(&data, 1) , "on") == 0)
            {
                preemptToggle = 1;
            }
            else
            {
                preemptToggle = 0;
            }
            svcChangePreemption();
        }

        else if (isCommand(&data, "sched", 0))
        {
            if (strCmp(getFieldString(&data, 1) , "prio") == 0)
            {
                schedToggle = 1;
            }
            else if (strCmp(getFieldString(&data, 1) , "rr") == 0)
            {
                schedToggle = 0;
            }
            svcChangeSchedule();
        }

        else if (isCommand(&data, "pidof", 0))
        {
            strncpy(name,getFieldString(&data, 1),12);
            strncpy(data.buf, name, 16);
            svcPidOf();
            putsUart0("\n");
        }

        else if (isCommand(&data, "run", 0))
        {
            //name = getFieldString(&data, 1);
            strncpy(name,getFieldString(&data, 1),12);
        }
        putsUart0("> ");
        getcUart0();    // Clears the buffer

    }
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;
    enableFaultHandlers(NVIC_SYS_HND_CTRL_USAGE | NVIC_SYS_HND_CTRL_BUS | NVIC_SYS_HND_CTRL_MEM );

    // Initialize hardware
    initHw();
    initUart0();
    initMpu();
    initRtos();

    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    putsUart0("\n");

    // Power-up flash
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(250000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(250000);

    // Initialize semaphores
    createSemaphore(keyPressed, 1);
    createSemaphore(keyReleased, 0);
    createSemaphore(flashReq, 5);
    createSemaphore(resource, 1);

    // Add required idle process at lowest priority
    ok =  createThread(idle, "Idle", 7, 1024);
    //ok &= createThread(idle2, "Idle2", 7, 1024);

    // Add other processes

    ok &= createThread(lengthyFn, "LengthyFn", 6, 1024);
    ok &= createThread(flash4Hz, "Flash4Hz", 4, 1024);
    ok &= createThread(oneshot, "OneShot", 2, 1024);
    ok &= createThread(readKeys, "ReadKeys", 6, 1024);
    ok &= createThread(debounce, "Debounce", 6, 1024);
    ok &= createThread(important, "Important", 0, 1024);
    ok &= createThread(uncooperative, "Uncoop", 6, 1024);
    ok &= createThread(errant, "Errant", 6, 1024);
    ok &= createThread(shell, "Shell", 6, 2048);

    // Start up RTOS
    if (ok)
        startRtos(); // never returns
    else
        setPinValue(RED_LED, 1);

    return 0;
}

/*******************************************************************************
*                                   uC/OS-II
*                             The Real-Time Kernel
*                         Atmel ATmega128 Sample Code
* File     : RobotRTOS2019STUDENT.C
* By       : John Collins
* Date     : 22 May 2019
* Description :
*     This program uses the uC/OS-II RTOS to implement navigation commands
*     including travelling to a target (x, y) using PID control
*     and turning to a specified heading angle.
*     The robot is controlled using commands received in the serial port.
* Limitations :
*     This program assumes the robot starts at the origin,
*     and is pointing along the x axis.
* Student Version :
*     This version is a starting point for students.
*     It will be a runnable program that has limited capability in these tasks:
*         Command task: Only the name of the command is interpreted.
*         Motor task: Only very basic motor operation is provided:
*             GO = both motors on at full speed.
*             ANGLE = one motor on (full speed) and one motor off.
*             STOP = both motors off.
*         Position task: The position is only approximately calculated.
*     ALL other tasks and functions (including interrupts) are fully operational.
*******************************************************************************/

/*******************************************************************************
* INCLUDES
* Include any other header files required by your program.
* iom128.h header file is already included in includes.h
*******************************************************************************/
#include <includes.h>
#include <pgmspace.h> // time delay
#include <stdio.h>
#include <string.h>
#include <math.h>

/*******************************************************************************
* DEFINES AND ENUMERATIONS
*******************************************************************************/
#define PI 3.14159265

// ENUMERATIONS
// commands
enum {STOP_CMD, GO_CMD, ANGLE_CMD};

// STRUCTURES
struct Position
{
    float x; // x and y position coordinates in metres
    float y;
    float h; // heading angle in radians, anti-clockwise from the x-axis
};

struct Target
{
    char cmd; // command enumeration value
    float x;
    float y;
    float h;
};

/*******************************************************************************
* RTOS TASK STACKS
*******************************************************************************/
#define OS_HARD_STACK_SIZE_0 20
#define OS_TASK_STACK_SIZE_0 ( 90 + OS_HARD_STACK_SIZE_0) // Startup (10+63)
// use priority 1 for sprintfMutex
#define OS_HARD_STACK_SIZE_2 30
#define OS_TASK_STACK_SIZE_2 ( 90 + OS_HARD_STACK_SIZE_2) // SerialOutput (15 + 65)
#define OS_HARD_STACK_SIZE_3 40
#define OS_TASK_STACK_SIZE_3 (330 + OS_HARD_STACK_SIZE_3) // Motor (21+287)
#define OS_HARD_STACK_SIZE_4 40
#define OS_TASK_STACK_SIZE_4 (320 + OS_HARD_STACK_SIZE_4) // Position (22+278)
#define OS_HARD_STACK_SIZE_5 40
#define OS_TASK_STACK_SIZE_5 (350 + OS_HARD_STACK_SIZE_5) // Command (26+305)
OS_STK TaskStack0[OS_TASK_STACK_SIZE_0];
OS_STK TaskStack2[OS_TASK_STACK_SIZE_2];
OS_STK TaskStack3[OS_TASK_STACK_SIZE_3];
OS_STK TaskStack4[OS_TASK_STACK_SIZE_4];
OS_STK TaskStack5[OS_TASK_STACK_SIZE_5];

/*******************************************************************************
* OTHER RTOS DEFINES
*******************************************************************************/
#define SERIAL_OUT_Q_SIZE  12 // number of serial out queue items (strings)
#define SERIAL_IN_Q_SIZE   32 // number of serial in queue items (bytes)
#define NUM_MSG_MEM_BLOCKS 12 // number of message memory partition blocks
#define MEM_BLOCK_SIZE     32 // number of bytes in each memory block

/*******************************************************************************
* GLOBAL VARIABLES
* These are all required by the RTOS.
*******************************************************************************/
// semaphores
OS_EVENT *LeftPulseSem; // pulse count from Left Pulse Int to Position (flag 0)
OS_EVENT *RightPulseSem; // pulse count from Right Pulse Int to Position (flag 1)
OS_EVENT *TimerSem; // timer signal from 100ms Timer Int to PID in Motor (flag 2)
OS_EVENT *TransmitDoneSem; // transmit done from Serial Transmit Int to Serial Output
OS_EVENT *CommandSem; // new command from Serial Receive Int to Command task
// message boxes
OS_EVENT *PositionMbox; // position from Position to Motor task (flag 0)
OS_EVENT *TargetMbox; // target from Command to Motor task (flag 1)
OS_EVENT *TransmitMbox; // message from Serial Output Task to Serial Transmit Int
// queues
OS_EVENT *SerialOutQ; // queue for messages to Serial Output task
void *serialOutQPtrs[SERIAL_OUT_Q_SIZE]; // array of pointers for serial queue
OS_EVENT *SerialInQ; // queue for messages to Serial Output task
void *serialInQPtrs[SERIAL_IN_Q_SIZE]; // array of pointers for serial queue
// flag groups
OS_FLAG_GRP *PositionFlags; // unblock Position task (1=left, 2=right)
OS_FLAG_GRP *MotorFlags; // unblock Motor task (1=position, 2=command, 4=timer)
// mutexes
OS_EVENT *SprintfMutex; // control access to sprintf function
// memory partitions
OS_MEM *MessageMem; // memory partition for messages
// memory partition array needs to aligned on pointer boundary
#pragma data_alignment=sizeof(void*)
// array for message memory partition
char messageMemory[NUM_MSG_MEM_BLOCKS * MEM_BLOCK_SIZE];

volatile int prioCount[OS_LOWEST_PRIO + 1]; // count of priority run times

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void StartupTask(void *data); // startup task function (task 0)
void SerialOutputTask(void *data); // serial output (task 2)
void MotorTask(void *data); // PID and turning operation for motors (task 3)
void PositionTask(void *data); // calculate position from wheel pulses (task 4)
void CommandTask(void *data); // command interpretation (task 5)
void Setup(void); // ATmega128 initialisation for this program
// function for serial output before the RTOS is operational
void SendSerial(char* msg);
// function using RTOS to send a string to PC
void OutputString(char *msg);
// wrapper functions for RTOS function calls, for detecting memory bugs
void* MemGet(OS_MEM *pmem, unsigned char *error);
unsigned char MemPut(OS_MEM *pmem, void *mem);
unsigned char QPost(OS_EVENT *q, void *message);
unsigned char MboxPost(OS_EVENT *mbox, void *message);

/*******************************************************************************
* MAIN FUNCTION
*******************************************************************************/
void main (void)
{
    unsigned char error = 0xAA; // error code from RTOS function calls
    Setup(); // initialise ATmega128
    SendSerial("\r\nMAIN STARTING\r\n");
    OSInit(); // initialise RTOS
    // create communication channels
    // semaphores
    LeftPulseSem = OSSemCreate(0);
    RightPulseSem = OSSemCreate(0);
    TimerSem = OSSemCreate(0);
    TransmitDoneSem = OSSemCreate(0);
    CommandSem = OSSemCreate(0);
    // message boxes
    PositionMbox = OSMboxCreate(NULL);
    TargetMbox = OSMboxCreate(NULL);
    TransmitMbox = OSMboxCreate(NULL);
    // queues
    SerialOutQ = OSQCreate(serialOutQPtrs, SERIAL_OUT_Q_SIZE);
    SerialInQ = OSQCreate(serialInQPtrs, SERIAL_IN_Q_SIZE);
    // flag groups
    PositionFlags = OSFlagCreate(0, &error);
    MotorFlags = OSFlagCreate(0, &error);
    // create SprintfMutex with priority 1
    // higher than all tasks except Startup
    SprintfMutex = OSMutexCreate(1, &error);
    // memory partitions
    MessageMem = OSMemCreate(messageMemory, NUM_MSG_MEM_BLOCKS, MEM_BLOCK_SIZE, &error);
    // create startup task
    // initialize these global variables before creating tasks
    // THIS IS AWFUL PROGRAMMING TECHNIQUE!
    OSTaskStkSize = OS_TASK_STACK_SIZE_0; // total data and hardware stack size
    OSTaskStkSizeHard = OS_HARD_STACK_SIZE_0; // hardware stack size
    OSTaskCreateExt(StartupTask, NULL,
        &TaskStack0[OS_TASK_STACK_SIZE_0-1], 0, 0,
        &TaskStack0[0], OS_TASK_STACK_SIZE_0, NULL,
        OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    // start RTOS multitasking
    SendSerial("RTOS STARTING\r\n");
    OSStart();
}

/*******************************************************************************
* STARTUP TASK 0
*******************************************************************************/
void StartupTask(void* data) // startup task, creates other tasks
{
    // create other tasks
    OSTaskStkSize = OS_TASK_STACK_SIZE_2; // total data and hardware stack size
    OSTaskStkSizeHard = OS_HARD_STACK_SIZE_2; // hardware stack size
    OSTaskCreateExt(SerialOutputTask, NULL,
        &TaskStack2[OS_TASK_STACK_SIZE_2-1], 2, 2,
        &TaskStack2[0], OS_TASK_STACK_SIZE_2, NULL,
        OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    OSTaskStkSize = OS_TASK_STACK_SIZE_3; // total data and hardware stack size
    OSTaskStkSizeHard = OS_HARD_STACK_SIZE_3; // hardware stack size
    OSTaskCreateExt(MotorTask, NULL,
        &TaskStack3[OS_TASK_STACK_SIZE_3-1], 3, 3,
        &TaskStack3[0], OS_TASK_STACK_SIZE_3, NULL,
        OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    OSTaskStkSize = OS_TASK_STACK_SIZE_4; // total data and hardware stack size
    OSTaskStkSizeHard = OS_HARD_STACK_SIZE_4; // hardware stack size
    OSTaskCreateExt(PositionTask, NULL,
        &TaskStack4[OS_TASK_STACK_SIZE_4-1], 4, 4,
        &TaskStack4[0], OS_TASK_STACK_SIZE_4, NULL,
        OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    OSTaskStkSize = OS_TASK_STACK_SIZE_5; // total data and hardware stack size
    OSTaskStkSizeHard = OS_HARD_STACK_SIZE_5; // hardware stack size
    OSTaskCreateExt(CommandTask, NULL,
        &TaskStack5[OS_TASK_STACK_SIZE_5-1], 5, 5,
        &TaskStack5[0], OS_TASK_STACK_SIZE_5, NULL,
        OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    OSTaskDel(OS_PRIO_SELF); // delete this task
}

/*******************************************************************************
* SERIAL OUTPUT TASK 2
*******************************************************************************/
// transmit messages from the queue out the serial port
void SerialOutputTask(void* data)
{
    unsigned char error; // error code from RTOS function calls
    char* pMessage; // pointer to message got from serial message queue
    OutputString("SERIAL OUT TASK STARTING\r\n"); // display the task message
    while (1)
    {
        // wait for a message from the serial queue
        pMessage = (char*)OSQPend(SerialOutQ, 0, &error);
        // if message is not empty
        if (*pMessage != 0)
        {
            // send the message address to the interrupt function
            MboxPost(TransmitMbox, pMessage);
            UCSR0B_UDRIE0 = 1; // enable DRE interrupt
            // wait for interrupt to finish sending the message
            OSSemPend(TransmitDoneSem, 0, &error);
        }
        MemPut(MessageMem, pMessage); // free the message memory
    }
}

/*******************************************************************************
* MOTOR TASK 3
*******************************************************************************/
void MotorTask(void* data) // operate the motors
{
    unsigned char error; // error code from RTOS function calls
    struct Position* pPos; // pointer to received position message
    struct Position pos; // current robot position
    struct Target* pTarget; // pointer to received target mesaage
    struct Target target; // current robot target
    int newCmd; // new command flag
    OutputString("MOTOR TASK STARTING\r\n"); // display the task message
    // initialise motors off
    target.cmd = STOP_CMD;
    OCR1A = 0;
    OCR1B = 0;
    // get initial position message from the Position task
    pPos = OSMboxPend(PositionMbox, 0, &error);
    pos = *pPos; // copy the message into the local position structure
    MemPut(MessageMem, pPos); // return memory block to memory manager
    while (1)
    {
        // wait for an event using MotorFlags
        OSFlagPend(MotorFlags, 7, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &error);
        // check for a new target message
        pTarget = OSMboxAccept(TargetMbox);
        newCmd = 0; // clear the new command flag
        // if there is a target message
        if(pTarget != NULL)
        {
            // copy the message into the local target structure
            target = *pTarget;
            // return memory block to memory manager
            OSMemPut(MessageMem, pTarget);
            newCmd = 1; // set the new command flag
            OutputString("111\r\n");
        }
        // check for a new position message
        pPos = OSMboxAccept(PositionMbox); 
        // if there is a position message
        if(pTarget != NULL); 
        {
            // copy the message into the local position structure
            pos = *pPos;
            // return memory block to memory manager
            OSMemPut(MessageMem, pPos);
        }
        // if there is a new command
        if (newCmd)
        {
            // implement the command
            switch (target.cmd)
            {
            case STOP_CMD:
                OCR1A = 0; // stop
                OCR1B = 0;
                break;
            case GO_CMD:
                OCR1A = 1023; // go forward
                OCR1B = 1023;
                break;
            case ANGLE_CMD:
                if (target.h >= 0)
                {
                    OCR1A = 0; // turn left
                    OCR1B = 1023;
                }
                else
                {
                    OCR1A = 1023; // turn right
                    OCR1B = 0;
                }
                break;
            }
        }
    }
}

/*******************************************************************************
* POSITION TASK 4
*******************************************************************************/
void PositionTask(void* data) // calculate robot position
{
    unsigned char error; // error code from RTOS function calls
    char* pMessage; // pointer to message for SerialOut task
    static struct Position pos; // current robot position
    struct Position* pPos; // pointer to position message for Motor task
    int leftCount, rightCount; // wheel pulse counts
    OutputString("POSITION TASK STARTING\r\n"); // display the task message
    pos.x = 0; // set initial position in metres
    pos.y = 0;
    pos.h = 0; // set initial heading in radians
    // construct initial position string message for PC
    // get memory block for the message
    pMessage = MemGet(MessageMem, &error);
    // use sprintf to construct the message string
    OSMutexPend(SprintfMutex, 0, &error);
    sprintf(pMessage, "POS,%6.3f,%6.3f,%4.0f\r\n", pos.x, pos.y, pos.h * 180. / PI);
    OSMutexPost(SprintfMutex);
    // post the message to the serial out queue
    QPost(SerialOutQ, pMessage);
    // construct position structure message for motor task
    // get memory block for the message
    pPos = MemGet(MessageMem, &error);
    // copy the position to the memory block
    *pPos = pos;
    // post the message to the position message box
    MboxPost(PositionMbox, pPos);
    // enable external interrupts for wheel counters (INT0, INT1)
    EICRA |= 0x0F; // use rising edges of wheel pulses
    EIFR = 0x03; // clear interrupt flags
    EIMSK |= 0x03; // enable INT0, INT1
    while (1)
    {
        // wait for a right or left wheel pulse using PositionFlags
        OSFlagPend(PositionFlags, 3, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &error);
        // get the wheel pulse counts from the pulse semaphores
        leftCount = 0;
        rightCount = 0;
        while(OSSemAccept(LeftPulseSem))
            leftCount++;
        while(OSSemAccept(RightPulseSem))
            rightCount++;
        // calculate new robot position and heading (approximate)
        pos.x += 0.007114 * (rightCount + leftCount) / 2 * cos(pos.h);
        pos.y += 0.007114 * (rightCount + leftCount) / 2 * sin(pos.h);
        pos.h += 0.06686 * (rightCount - leftCount);
        if (pos.h < -PI)
        {
            pos.h += 2 * PI;
        }
        else if (pos.h > +PI)
        {
            pos.h -= 2 * PI;
        }
        // post the new position to Motor task
        // check if there is still a message in the message box
        pPos = OSMboxAccept(PositionMbox);
        // if the message box is empty
        if(pPos == 0)
        // ADD CODE LINE
        {
            // get memory block for the message
            pPos = MemGet(MessageMem, &error);
        }
        // copy the position structure to the memory block
        *pPos = pos;
        // post the message to the position message box
        MboxPost(PositionMbox, pPos);
        // post the flag to unblock the motor task
        OSFlagPost(MotorFlags, 1, OS_FLAG_SET, &error);
        // construct position string message for PC
        // get memory block for the message
        pMessage = MemGet(MessageMem, &error);
        // use mutex and sprintf to construct the message string
        OSMutexPend(SprintfMutex, 0, &error);
        sprintf(pMessage, "POS,%6.3f,%6.3f,%5.0f\r\n",
            pos.x, pos.y, pos.h * 180. / PI);
        OSMutexPost(SprintfMutex);
        // post the message to the serial out queue
        QPost(SerialOutQ, pMessage);
    }
}

/*******************************************************************************
* COMMAND TASK 5
*******************************************************************************/
void CommandTask(void* data) // receive commands from PC
{
    unsigned char error; // error code from RTOS function calls
    struct Target* ptarget; // pointer to target structure message
    char *pch; // pointer to received character
    char ch; // received character value
    char command[MEM_BLOCK_SIZE]; // array for received characters
    int index; // array index for next character
    OutputString("COMMAND TASK STARTING\r\n"); // display the task message
    error = UDR0; // dummy read to clear the serial receive interrupt flag
    UCSR0B_Bit7 = 1; // enable serial receive interrupt
    while (1)
    {
        // wait for command semaphore (from serial receive interrupt)
        OSSemPend(CommandSem, 0, &error);
        // then get the command chars from SerialInQ
        index = 0; // initialise array index
        do
        {
            // get the next character (pointer) from the serial in queue
            pch = OSQPend(SerialInQ, 0, &error);
            ch = *pch; // get the character value
            if (ch == '\r') // if end of command string
            {
                command[index] = 0; // null char for end of string
            }
            else
            {
                command[index] = ch; // put char into local array
                index++; // increment array index
            }
            //pch = OSQPend(SerialInQ, 0, &error);
        } while (ch != '\r'); // loop wile not end of command

        // analyse the command
        // if it is a stop command
        if (!strncmp(command, "STOP", 4))
        {
            // post message to PC
            OutputString("STOP\r\n");
            // post target structure message to the motor task
            // get memory block for target message
            ptarget = MemGet(MessageMem, &error);
            // put the command into the structure
            ptarget->cmd = STOP_CMD;
            // post the structure pointer to target message box
            MboxPost(TargetMbox, ptarget);
            // post flag to unblock motor task
            OSFlagPost(MotorFlags, 2, OS_FLAG_SET, &error);
            
        }
        // if it is a go command
        else if (!strncmp(command, "GO", 2))
        {
            // post message to PC
            OutputString("GO\r\n");
            float f1,f2;
            int n = sscanf(command +3,"%f,%f",&f1,&f2);
            if (n == 2)
            {
                // get memory block for target message
                ptarget = MemGet(MessageMem, &error);
                // put the command into the structure
                ptarget->cmd = GO_CMD;
                ptarget->x = f1;
                ptarget->y = f2;
                // post the structure pointer to target message box
                MboxPost(TargetMbox, ptarget);
                // post flag to unblock motor task
                OSFlagPost(MotorFlags, 2, OS_FLAG_SET, &error);
            }
        }
        // if it is an angle command
        else if (!strncmp(command, "ANGLE", 5))
        {
            // post message to PC
            OutputString("ANGLE\r\n");
            // post target structure message to the motor task
            float f;
            int n = sscanf(command +6,"%f",&f);
         
            // get memory block for target message
            ptarget = MemGet(MessageMem, &error);
            // put the command into the structure
            ptarget->cmd = ANGLE_CMD;
            ptarget->h = f;
            // post the structure pointer to target message box
            MboxPost(TargetMbox, ptarget);
            // post flag to unblock motor task
            OSFlagPost(MotorFlags, 2, OS_FLAG_SET, &error);
        }
        else // invalid command
        {
            // post message to PC
            OutputString("COMMAND INVALID\r\n");
        }
    }
}

/*******************************************************************************
* OTHER FUNCTIONS CALLED BY TASKS
*******************************************************************************/
// send a string to the PC using the RTOS
void OutputString(char *msg)
{
    unsigned char error;
    char* pmsg = MemGet(MessageMem, &error); // get memory for the message
    strncpy(pmsg, msg, MEM_BLOCK_SIZE - 1); // construct the message
    QPost(SerialOutQ, pmsg); // send the message to the serial queue
}

/*******************************************************************************
* INTERRUPT FUNCTIONS
*******************************************************************************/
#pragma vector = INT0_vect
__interrupt void LeftPulseISR(void)  // left wheel pulse counter
{
    static unsigned char error; // error code from RTOS function calls
    OSIntEnter(); // must call this function first in any interrupt function
    OSSemPost(LeftPulseSem); // post semaphore for left wheel to Position task
    // set the event flag for the left wheel pulse
    OSFlagPost(PositionFlags, 1, OS_FLAG_SET, &error);
    OSIntExit(); // must call this function last in any interrupt function
}

#pragma vector = INT1_vect
__interrupt void RightPulseISR(void) // right wheel pulse counter
{
    static unsigned char error; // error code from RTOS function calls
    OSIntEnter(); // must call this function first in any interrupt function
    // post semaphore for right wheel pulse to Position task
    OSSemPost(RightPulseSem);
    // set the event flag for the right wheel
    OSFlagPost(PositionFlags, 2, OS_FLAG_SET, &error);
    OSIntExit(); // must call this function last in any interrupt function
}

#pragma vector = USART0_UDRE_vect
__interrupt void SerialTransmitISR(void) // serial transmit
{
    static char* messagePtr = NULL;
    OSIntEnter(); // must call this function first in any interrupt function
    if (messagePtr == NULL)
    {
        messagePtr = OSMboxAccept(TransmitMbox);
    }
    UDR0 = *messagePtr; // transmit the next character
    messagePtr++; // increment the character pointer
    if (*messagePtr == 0) // if end of message
    {
        UCSR0B_UDRIE0 = 0; // disable interrupt
        OSSemPost(TransmitDoneSem); // post semaphore indicating end of message
        messagePtr = NULL;
    }
    OSIntExit(); // must call this function last in any interrupt function
}

#pragma vector = USART0_RXC_vect
__interrupt void SerialReceiveISR(void) // serial receive
{
    static char ch; // received char
    static char queue[SERIAL_IN_Q_SIZE]; // received char array
    static int qi = 0; // queue index
    static unsigned char error; // error code from RTOS function calls
    OSIntEnter(); // must call this function first in any interrupt function
    ch = UDR0;
    // post char in queue
    queue[qi] = ch;
    error = OSQPost(SerialInQ, queue + qi);
    if (error == OS_NO_ERR)
    {
        // increment and wrap the queue index
        qi = (qi + 1) & (SERIAL_IN_Q_SIZE - 1);
    }
	// post semaphore at end of command
    if (ch == '\r')
    {
        OSSemPost(CommandSem);
    }
    // char gets lost if the queue is full
    OSIntExit(); // must call this function last in any interrupt function
}

#pragma vector = TIMER3_COMPA_vect
__interrupt void PIDTimerISR(void) // 100 ms interrupt for PID
{
    static unsigned char error; // error code from RTOS function calls
    OSIntEnter(); // must call this function first in any interrupt function
    OSSemSet(TimerSem, 1, &error); // set the semaphore to the Motor task for PID
    OSFlagPost(MotorFlags, 4, OS_FLAG_SET, &error); // and post the flag
    OSIntExit(); // must call this function last in any interrupt function
}

#pragma vector = TIMER0_COMP_vect
__interrupt void OSTickISR(void) // 1 ms timer tick interrupt for RTOS
{
    OSIntEnter(); // must call this function first in any interrupt function
    OSTimeTick(); // check tasks waiting for timeout
    prioCount[OSPrioCur]++;
    OSIntExit(); // must call this function last in any interrupt function
}

/*******************************************************************************
* OTHER FUNCTIONS
*******************************************************************************/
void Setup(void) // ATmega128 setup
{
    // use timer 0 to generate 1ms interrupts
    TCCR0 = 0x0C; // prescale 8MHz by 64 = 8us period, use CTC mode
    OCR0 = 124; // tick period = (124 + 1) * 8us = 1ms
    TIMSK = 0x02; // enable OCR0 compare match interrupt
    // timer setup for pwm
    TCCR1A = 0xA3; // enable pwm outputs and mode 7
    TCCR1B = 0x0C; // mode 7, prescale = 256
    TCCR1C = 0x00;
    OCR1A = 0; // motors off
    OCR1B = 0;
    // timer setup for PID period = 100ms
    TCCR3A = 0x00; // mode 4, CTC mode
    TCCR3B = 0x0B; // mode 4, prescale = 64
    TCCR3C = 0x00;
    OCR3AH = 48; // 12499 = 48 * 256 + 211
    OCR3AL = 211; // 8MHz / 64 / (12499 + 1) = 10Hz
    // enable this interrupt when required for PID
    ETIMSK_OCIE3A = 0; // disable OCR3A compare match interrupt
    // digital input/output
    DDRB_Bit5 = 1; // enable motor outputs
    DDRB_Bit6 = 1;
    DDRA_Bit6 = 1; // enable motor direction outputs
    DDRA_Bit7 = 1;
    PORTA_Bit6 = 0; // motors forward
    PORTA_Bit7 = 0;
    DDRC_Bit3 = 1; // enable wheel counters
    PORTC_Bit3 = 1;
    // serial output
    UCSR0A = 0;
    UCSR0B = 0x18; // enable receive and transmit
    UCSR0C = 0x06; // 8 data bits, no parity, 1 stop
    UBRR0H = 0; // 9600 bps = 51, 19200 bps = 25, 38400 = 12
    UBRR0L = 12;
    // give hardware time to settle
    __delay_cycles(4000000); // 500ms
    // do NOT enable global interrupt
    // enable required interrupts in appropriate tasks
    // RTOS will enable global interrupt in OSStart
}

/*******************************************************************************
* SERIAL OUTPUT FUNCTION (NOT A TASK)
* Allows serial output before the serial task is operational.
*******************************************************************************/
void SendSerial(char* msg)
{
    int n;
    for (n = 0; n < strlen(msg); n++)
    {
        while (!UCSR0A_UDRE0);
        UDR0 = msg[n];
    }
}

/*******************************************************************************
* WRAPPER FUNCTIONS
* To ensure these RTOS functions perform their operations correctly.
*******************************************************************************/
void* MemGet(OS_MEM *pmem, unsigned char *error) // get memory without error
{
    void *mem; // pointer to memory block
    mem = OSMemGet(pmem, error); // try to get memory
    if (*error != OS_ERR_NONE) // if there is an error (e.g. no memory available)
    {
        // on error: disable interrupts, turn motors off and loop forever
        OS_ENTER_CRITICAL();
        OCR1A = 0;
        OCR1B = 0;
        while (1);
    }
    return mem; // return pointer to memory block
}

unsigned char MemPut(OS_MEM *pmem, void *mem) // put memory with error checking
{
    unsigned char error; // error code from RTOS function calls
    error = OSMemPut(pmem, mem); // try to put memory
    if (error != OS_ERR_NONE) // if there is an error (e.g. invalid memory address)
    {
        // on error: disable interrupts, turn motors off and loop forever
        OS_ENTER_CRITICAL();
        OCR1A = 0;
        OCR1B = 0;
        while (1);
    }
    return error; // return pointer to memory block
}

unsigned char QPost(OS_EVENT *q, void *message) // post to queue without error
{
    unsigned char error; // error code from RTOS function calls
    error = OSQPost(q, message); // try to post message to queue
    if (error != OS_ERR_NONE) // if there is an error (e.g. queue is full)
    {
        // on error: disable interrupts, turn motors off and loop forever
        OS_ENTER_CRITICAL();
        OCR1A = 0;
        OCR1B = 0;
        while (1);
    }
    return error;
}

unsigned char MboxPost(OS_EVENT *mbox, void *message) //post to message box without error
{
    unsigned char error; // error code from RTOS function calls
    error = OSMboxPost(mbox, message); // try to post message to message box
    if (error != OS_ERR_NONE) // if there is an error (e.g. message box is full)
    {
        // on error: disable interrupts, turn motors off and loop forever
        OS_ENTER_CRITICAL();
        OCR1A = 0;
        OCR1B = 0;
        while (1);
    }
    return error;
}

/*******************************************************************************
* END OF PROGRAM
*******************************************************************************/

/*******************************************************************************
*                                   uC/OS-II
*                             The Real-Time Kernel
*                         Atmel ATmega128 Sample Code
* File     : RobotRTOS2019.C
* By       : John Collins
* Date     : 8 May 2019
* Description :
*     This program uses the uC/OS-II RTOS to implement navigation commands
*     including travelling to a target (x, y) using PID control
*     and turning to a specified heading angle.
*     The robot is controlled using commands received in the serial port.
* Limitations :
*     This program assumes the robot starts at the origin,
*     and is pointing along the x axis.
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
* DEFINES
*******************************************************************************/
#define PI 3.14159265

#define RIGHT_SPEED 1023
#define LEFT_SPEED  1023

#define KP     2000
#define KI     2000
#define KD     75
// ENUMERATIONS
// commands
enum {STOP_CMD, GO_CMD, ANGLE_CMD};
// Navigation states
enum {STOPPED_STATE, MOVING_STATE, TURNING_STATE};
// GO command states
enum {GO_START_STATE, GO_TURN_STATE, GO_PID_STATE, GO_STOP_STATE};

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
#define OS_HARD_STACK_SIZE_0 24
#define OS_TASK_STACK_SIZE_0 ( 96 + OS_HARD_STACK_SIZE_0) // Startup
// use priority 1 for sprintfMutex
#define OS_HARD_STACK_SIZE_2 32
#define OS_TASK_STACK_SIZE_2 ( 96 + OS_HARD_STACK_SIZE_2) // Serial Output
#define OS_HARD_STACK_SIZE_3 40
#define OS_TASK_STACK_SIZE_3 (288 + OS_HARD_STACK_SIZE_3) // Command
#define OS_HARD_STACK_SIZE_4 40
#define OS_TASK_STACK_SIZE_4 (288 + OS_HARD_STACK_SIZE_4) // Position
#define OS_HARD_STACK_SIZE_5 40
#define OS_TASK_STACK_SIZE_5 (352 + OS_HARD_STACK_SIZE_5) // Motor
OS_STK TaskStack0[OS_TASK_STACK_SIZE_0];
OS_STK TaskStack2[OS_TASK_STACK_SIZE_2];
OS_STK TaskStack3[OS_TASK_STACK_SIZE_3];
OS_STK TaskStack4[OS_TASK_STACK_SIZE_4];
OS_STK TaskStack5[OS_TASK_STACK_SIZE_5];

/*******************************************************************************
* OTHER RTOS DEFINES
*******************************************************************************/
#define SERIAL_OUT_Q_SIZE  12 // number of serial out queue items
#define SERIAL_IN_Q_SIZE  12 // number of serial in queue items
#define NUM_MSG_MEM_BLOCKS 12 // number of message memory partition blocks
#define MEM_BLOCK_SIZE 30 // number of bytes in each memory block

/*******************************************************************************
* GLOBAL VARIABLES
* These are all required by the RTOS.
*******************************************************************************/
// semaphores
OS_EVENT *LeftPulseSem; // pulse count from Left Pulse Int to Position (flag 0)
OS_EVENT *RightPulseSem; // pulse count from Right Pulse Int to Position (flag 1)
OS_EVENT *TimerSem; // timer signal from 100ms Timer Int to PID in Motor (flag 2)
OS_EVENT *TransmitDoneSem; // transmit done from Serial Transmit Int to Serial Output
// message boxes
OS_EVENT *CommandMbox; // command from Serial Receive Int to Command task
OS_EVENT *PositionMbox; // position from Position to Motor task (flag 0)
OS_EVENT *TargetMbox; // target from Command to Motor task (flag 1)
OS_EVENT *TransmitMbox; // message from Serial Output Task to Serial Transmit Int
// queues
OS_EVENT *SerialOutQ; // queue for messages to Serial Output task
void *serialOutQPtrs[SERIAL_OUT_Q_SIZE]; // array of pointers for serial queue
OS_EVENT *SerialInQ; // queue for messages to Serial Output task
void *serialInQPtrs[SERIAL_IN_Q_SIZE]; // array of pointers for serial queue
// flag groups
OS_FLAG_GRP *PositionFlags; // unblock Position task
OS_FLAG_GRP *MotorFlags; // unblock Motor task
// mutexes
OS_EVENT *SprintfMutex; // control access to sprintf function
// memory partitions
OS_MEM *MessageMem; // memory partition for messages
// memory partition array needs to aligned on pointer boundary
#pragma data_alignment=sizeof(void*)
// array for message memory partition
char messageMemory[NUM_MSG_MEM_BLOCKS * MEM_BLOCK_SIZE];

/*******************************************************************************
* FUNCTION PROTOTYPES
*******************************************************************************/
void StartupTask(void *data); // startup task function (task 0)
void SerialOutputTask(void *data); // serial output (task 2)
void PositionTask(void *data); // calculate position from wheel pulses (task 3)
void MotorTask(void *data); // PID and turning operation for motors (task 4)
void CommandTask(void *data); // command interpretation (task 5)
//int Move(struct Position *pPos, struct Target* pTgt); // control moving to an xy target
int Move(struct Position *ppos,struct Target *ptarget, int startingMove);
void PID(struct Position *pPos, struct Target *pTgt, int startingPID); // use PID to go to an xy target
void Setup(void); // ATmega128 initialisation for this program
// function for serial output before the RTOS is operational
void SendSerial(char* msg);
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
    unsigned char error; // error code from RTOS function calls
    Setup(); // initialise ATmega128
    SendSerial("\r\nMAIN STARTING\r\n");
    OSInit(); // initialise RTOS
    // create communication channels
    // semaphores
    LeftPulseSem = OSSemCreate(0);
    RightPulseSem = OSSemCreate(0);
    TimerSem = OSSemCreate(0);
    TransmitDoneSem = OSSemCreate(0);
    // message boxes
    CommandMbox = OSMboxCreate(NULL);
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
    OSTaskCreateExt(CommandTask, NULL,
        &TaskStack3[OS_TASK_STACK_SIZE_3-1], 4, 4,
        &TaskStack3[0], OS_TASK_STACK_SIZE_3, NULL,
        OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    OSTaskStkSize = OS_TASK_STACK_SIZE_4; // total data and hardware stack size
    OSTaskStkSizeHard = OS_HARD_STACK_SIZE_4; // hardware stack size
    OSTaskCreateExt(PositionTask, NULL,
        &TaskStack4[OS_TASK_STACK_SIZE_4-1], 5, 5,
        &TaskStack4[0], OS_TASK_STACK_SIZE_4, NULL,
        OS_TASK_OPT_STK_CHK | OS_TASK_OPT_STK_CLR);
    OSTaskStkSize = OS_TASK_STACK_SIZE_5; // total data and hardware stack size
    OSTaskStkSizeHard = OS_HARD_STACK_SIZE_5; // hardware stack size
    OSTaskCreateExt(MotorTask, NULL,
        &TaskStack5[OS_TASK_STACK_SIZE_5-1], 3, 3,
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
    pMessage = MemGet(MessageMem, &error); // get memory for the message
    strcpy(pMessage, "SERIAL OUT TASK STARTING\r\n"); // construct the message
    QPost(SerialOutQ, pMessage); // send the message to the serial queue
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
* COMMAND TASK 3
*******************************************************************************/
void CommandTask(void* data) // receive commands from PC
{
    unsigned char error; // error code from RTOS function calls
    char* pMessage; // pointer to output message 
    struct Target* pTarget; // pointer to target structure
    char *pch; // pointer to received character
    unsigned char cmdError = 0; // serial received error
    unsigned char cmdEnd = 0; // end of command line
    char command[MEM_BLOCK_SIZE - 4];
    int cmdIndex = 0;
    char ch;
    float f1, f2; // command parameters
    int n; // number of parameters
    pMessage = MemGet(MessageMem, &error); // get memory for the message
    strcpy(pMessage, "COMMAND TASK STARTING\r\n"); // construct the message 
    QPost(SerialOutQ, pMessage); // send the message to the serial queue
    // clear serial receive interrupt flag
    error = UDR0; // dummy read to clear the receive flag
    // enable serial receive interrupt   
    UCSR0B_Bit7 = 1;
    while (1)
    {
        // get next command char
        pch = OSQPend(SerialInQ, 0, &error);
        ch = *pch;
        if (cmdIndex >= MEM_BLOCK_SIZE - 3) // command too long
        {
            cmdError = 1;
            cmdIndex = 0;
        }
        else if (ch == '\r') // end of command string
        {
            command[cmdIndex] = 0;
            cmdIndex = 0;
            cmdEnd = 1;
        }
        else
        {
            command[cmdIndex] = ch;
            cmdIndex++;
        }
        if (cmdError == 1)
        {
            pMessage = MemGet(MessageMem, &error); // get memory for message
            strcpy(pMessage, "SERIAL COMMS ERROR\r\n");
            QPost(SerialOutQ, pMessage); // send the message to the serial queue
            cmdError = 0;
        }
        else if (cmdEnd == 1)
        {
            // echo the command to the PC
            pMessage = MemGet(MessageMem, &error); // get memory for message
            OSMutexPend(SprintfMutex, 0, &error);
            sprintf(pMessage, "%s\r\n", command);
            OSMutexPost(SprintfMutex);
            QPost(SerialOutQ, pMessage); // send the message to the serial queue
            cmdEnd = 0;
            // analyse the command
            
            
            if (strncmp(command, "GO", 2) == 0)
            {         
                n = sscanf(command +3,"%f,%f",&f1,&f2);
                if (n == 2)
                {
                    // get memory for message 
                    pTarget = MemGet(MessageMem, &error);
                    // set value
                    pTarget->cmd = GO_CMD;
                    pTarget->x = f1;
                    pTarget->y = f2;
                    // post valid commands to the Motor task
                    MboxPost(TargetMbox, pTarget);
                    OSFlagPost(MotorFlags, 2, OS_FLAG_SET, &error);
                }
            }else if (strncmp(command, "STOP", 4) == 0)
            {
                // get memory for message 
                pTarget = MemGet(MessageMem, &error);
                //set value
                pTarget->cmd = STOP_CMD;
                // post valid commands to the Motor task
                MboxPost(TargetMbox, pTarget);
                OSFlagPost(MotorFlags, 2, OS_FLAG_SET, &error);
            }
            if (cmdError == 1)
            {
                pMessage = MemGet(MessageMem, &error); // get memory for message
                strcpy(pMessage, "INVALID COMMAND\r\n");
                QPost(SerialOutQ, pMessage); // send the message to the serial queue
                cmdError = 0;
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
    float R; // radius of curvature of arc moved through
    float newHeading; // new heading in radians
    pMessage = MemGet(MessageMem, &error); // get memory for the message
    strcpy(pMessage, "POSITION TASK STARTING\r\n"); // construct the message 
    QPost(SerialOutQ, pMessage); // send the message to the serial queue
    pos.x = 0; // set initial position in metres
    pos.y = 0;    
    pos.h = 0; 
    // construct and post initial position message to PC
    pMessage = MemGet(MessageMem, &error); // get memory for string
    OSMutexPend(SprintfMutex, 0, &error);
    sprintf(pMessage, "POS,%6.3f,%6.3f,%5.0f\r\n", pos.x, pos.y, pos.h * 180. / PI);
    OSMutexPost(SprintfMutex);
    QPost(SerialOutQ, pMessage); // send the message to the serial queue
    // post position to motor task
    pPos = MemGet(MessageMem, &error); // get memory for message
    *pPos = pos;
    MboxPost(PositionMbox, pPos);
    OSFlagPost(MotorFlags, 1, OS_FLAG_SET, &error);
    // enable external interrupts for wheel counters (INT0, INT1)
    EICRA |= 0x0F; // use rising edges of wheel pulses
    EIFR = 0x03; // clear interrupt flags
    EIMSK |= 0x03; // enable INT0, INT1
    while (1)
    {
        // wait for a right or left wheel interrupt using MotorFlags
        OSFlagPend(PositionFlags, 3, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &error);
        // get the wheel pulse counts
#if 1		
        leftCount = OSSemAccept_ext(LeftPulseSem);  //always clear semaphore value
        rightCount = OSSemAccept_ext(RightPulseSem);
#endif		
#if 0
        leftCount = 0;
        rightCount = 0;
        while(OSSemAccept(LeftPulseSem))
            leftCount++;
        while(OSSemAccept(RightPulseSem))
            rightCount++;
#endif
        // calculate new robot position
        if(leftCount || rightCount)
        {
            if(leftCount == rightCount)
            {
                pos.x += 0.007114 * leftCount * cos(pos.h);
                pos.y += 0.007114 * leftCount * sin(pos.h);
            }
            else
            {
                newHeading = (rightCount - leftCount)*0.06686;
                R = 0.0532 * (rightCount + leftCount)/(rightCount - leftCount); 
                pos.x += R*(sin((pos.h) + newHeading) - sin(pos.h));
                pos.y += R*(cos(pos.h) - cos((pos.h) + newHeading));
                pos.h += newHeading;
              
                while(pos.h >= PI)
                    pos.h -=2*PI;
                while(pos.h < -PI)
                    pos.h +=2*PI;
            }
        }
        // post the new position to Motor task
        pPos = MemGet(MessageMem, &error); // get memory for message
        *pPos = pos;
        MboxPost(PositionMbox, pPos);
        OSFlagPost(MotorFlags, 1, OS_FLAG_SET, &error); //  post the flag  
        // send message to PC
        pMessage = MemGet(MessageMem, &error);
        OSMutexPend(SprintfMutex, 0, &error);
        sprintf(pMessage, "POS,%6.3f,%6.3f,%5.0f\r\n",
            pos.x, pos.y, pos.h * 180. / PI);
        OSMutexPost(SprintfMutex);
        QPost(SerialOutQ, pMessage);
    }
}

/*******************************************************************************
* MOTOR TASK 5
*******************************************************************************/
void MotorTask(void* data) // operate the motors
{
    unsigned char error;
    char *pMessage;
    struct Position* pPos;
    struct Position pos;
    struct Target* pTarget;
    struct Target target;
    int state;
    int stopped;
    int state_update;
    pMessage = MemGet(MessageMem, &error); // get memory for the message
    strcpy(pMessage, "MOTOR TASK STARTING\r\n"); // construct the message 
    QPost(SerialOutQ, pMessage); // send the message to the serial queue
    // initialise motors off
    OCR1A = 0;
    OCR1B = 0;
    // get initial position
    pPos = OSMboxPend(PositionMbox, 0, &error);
    pos = *pPos;
    MemPut(MessageMem, pPos); // free memory
    // set initial target
    state = STOP_CMD;
    state_update = 0;
    while (1)
    {
        // wait for an event using PositionFlags
        OSFlagPend(MotorFlags, 7, OS_FLAG_WAIT_SET_ANY + OS_FLAG_CONSUME, 0, &error);
        // check for a new position
		pPos = OSMboxAccept(PositionMbox);
        if (pPos != NULL)
        {
            pos = *pPos;
            OSMemPut(MessageMem, pPos);
        }
        
        // check for a new target command
        pTarget = OSMboxAccept(TargetMbox);
        if(pTarget != NULL)
        {
            target = *pTarget;
            OSMemPut(MessageMem, pTarget);
            state = target.cmd;
            state_update = 1; //state to be updated 
        }
#if 0 //debug
        pMessage = MemGet(MessageMem, &error);
        OSMutexPend(SprintfMutex, 0, &error);
        sprintf(pMessage, "POSc,%6.3f,%6.3f,%5.0f\r\n",
            pos.x, pos.y, pos.h * 180. / PI);
        OSMutexPost(SprintfMutex);
        QPost(SerialOutQ, pMessage);
        pMessage = MemGet(MessageMem, &error);
        OSMutexPend(SprintfMutex, 0, &error);
        sprintf(pMessage, "POSt,%6.3f,%6.3f,%5.0f\r\n",
            target.x, target.y, target.h * 180. / PI);
        OSMutexPost(SprintfMutex);
        QPost(SerialOutQ, pMessage);
#endif
        // handle each command (calls Move)
        switch (state)
        {
            case STOP_CMD:
                OCR1A = 0;
                OCR1B = 0;
                state_update = 0;
                break;
            case GO_CMD:
                if (Move(&pos,&target, state_update))
                {
                    state = STOP_CMD;
                }
                state_update = 0;
                break;
        }
    }
}

/*******************************************************************************
* OTHER FUNCTIONS CALLED BY TASKS
*******************************************************************************/
// GO command state diagram
/*
int Move(struct Position *pPos, struct Target* pTgt)
{
    // implement the MOVING state diagram
// ***
    // return true if the robot has stopped
    return 1;
}*/

int Move(struct Position *ppos,struct Target *ptarget, int startingMove)
{
    //state machine
    enum ROBOT_STATE {START_STATE, TURN_STATE, PID_STATE, STOP_STATE};
    static int robotState = START_STATE;
    static int startPID = 0;

    
    float e = ppos->h - atan2(ptarget->y - ppos->y,ptarget->x - ppos->x);
    float distance = sqrt((ptarget->y - ppos->y)*(ptarget->y - ppos->y)
                     + (ptarget->x - ppos->x)*(ptarget->x - ppos->x));
    
       
    if (startingMove){
        robotState = START_STATE;      
    }
    
    while (e>PI)
      e -=2*PI;
    while (e<-PI)
      e +=2*PI;
	
    switch (robotState)
    {
        case START_STATE:
            if (distance <= 0.050) // meters
                robotState = STOP_STATE;
            else
                robotState = TURN_STATE;
            break;
        case TURN_STATE:
            if (distance <= 0.050) // meters
                robotState = STOP_STATE;
            else if (e > 0.5 ) // radians
            {
                OCR1A = LEFT_SPEED; 
                OCR1B = 0;
            } else if (e < -0.5)
            {
                OCR1A = 0;
                OCR1B = RIGHT_SPEED;
            }
            else
            {
                robotState = PID_STATE;
                startPID = 1;
            }
            break;
        case PID_STATE:
            if (distance <= 0.050) // meters
                robotState = STOP_STATE;
            else if (e > 0.5 || e < -0.5) // radians
                robotState = TURN_STATE;
            else
                PID(ppos, ptarget, startPID); // call separate PID function
            startPID = 0;
            break;
        case STOP_STATE:
            OCR1A = 0;
            OCR1B = 0;
            return 1;
            break;
    }
    return 0;
}

// perform PID

void PID(struct Position *pPos, struct Target *pTgt, int startingPID)
{
    
    float f = 0.0;
    static float prevError = 0.0;
    static float integral = 0.0;
    
    //use this to keep startingPID value,
    //in case TimerSem = 0;
    static int state_startingPID = 0;
    state_startingPID = startingPID;
    
    float currError = pPos->h - atan2(pTgt->y - pPos->y,pTgt->x - pPos->x);
    
    if(OSSemAccept(TimerSem))
    {
        if(state_startingPID)
        {
            integral = 0;
            prevError = currError;  //the first differentiation result should be 0
            state_startingPID = 0;
        }
        
        integral += (currError + prevError)*0.05; //0.05=T X 1/2
        f = KP*currError+ KI*integral +KD*(currError-prevError)*10; //10=(1/T)
        if (f >= 0)
        {
            OCR1A = LEFT_SPEED;
            if ((RIGHT_SPEED - (int)f)>0)
                OCR1B = RIGHT_SPEED -(int)f;
            else
                OCR1B = 0;
        }else
        {
            if((LEFT_SPEED + (int)f)>0)
                OCR1A = LEFT_SPEED + (int)f;
            else
                OCR1A = 0;
                OCR1B = RIGHT_SPEED;
        }
        prevError = currError;
    }
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
    ETIMSK_OCIE3A = 1; // disable OCR3A compare match interrupt
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

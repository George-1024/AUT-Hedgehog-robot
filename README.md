# AUT-Hedgehog-robot
 My RTOS assignment in AUT
 
These code show how a typical RTOS system runs.


RobotRTOS2019.c is a long code version, which includes the PID control.
RobotRTOS2019_simple.c is a simpler version.

In the first one, I tried to create and use a function OSSemAccept_ext(), 
which can reduce the inner counter value to zero when it is called.

OSSemAccept_ext() source code was added in the file "os_sem.c" , near to the function OSSemAccept().

#include "cmsis_os2.h"                          // CMSIS RTOS header file
 
/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
osThreadId_t tid_ThDisplay;                        // thread id
 
void Th_Display (void *argument);                   // thread function
 
int Init_Th_Display (void) {
 
  tid_ThDisplay = osThreadNew(Th_Display, NULL, NULL);
  if (tid_ThDisplay == NULL) {
    return(-1);
  }
 
  return(0);
}
 
void Th_Display (void *argument) {
 
  while (1) {
    ; // Insert thread code here...
    osThreadYield();                            // suspend thread
  }
}

#include "cmsis_os2.h"                          // CMSIS RTOS header file
#include "stm32f4xx_hal.h"

/*----------------------------------------------------------------------------
 *      Thread 1 'Thread_Name': Sample thread
 *---------------------------------------------------------------------------*/
 
osThreadId_t tid_ThRTC;                        // thread id
 
void Th_RTC (void *argument);                   // thread function

RTC_HandleTypeDef rtc; // Manejador del RTC
RTC_TimeTypeDef time; // Configuracion de la hora
RTC_DateTypeDef date; // Configuracion de la fecha
HAL_RTCStateTypeDef state; // Estado del RTC

 
int Init_Th_RTC (void) {
 
  tid_ThRTC = osThreadNew(Th_RTC, NULL, NULL);
  if (tid_ThRTC == NULL) {
    return(-1);
  }
 
  return(0);
}
 
void Th_RTC (void *argument) {

	/* Configuracion RTC */
	 __HAL_RCC_RTC_ENABLE(); //Habilitacion del puero para el periférico
	
	rtc.Instance = RTC;
	rtc.Init.HourFormat = RTC_HOURFORMAT_24;
	rtc.Init.AsynchPrediv = 127; // 128 -> 32768/128 = 256 
	rtc.Init.SynchPrediv = 255; // 256/256 -> 1Hz -> 1s
	
	/* Configuracion hora */
	time.Hours = 10;
	time.Minutes = 30;
	time.Seconds = 30;
	time.TimeFormat = RTC_HOURFORMAT12_AM;
	time.DayLightSaving = RTC_DAYLIGHTSAVING_NONE; 
	
	/* Configuracion de la fecha */
	date.Date = 1;
	date.Month = RTC_MONTH_JANUARY;
	date.WeekDay = RTC_WEEKDAY_MONDAY;
	date.Year = 24;
	
	HAL_RTC_SetTime(&rtc, &time, RTC_FORMAT_BCD); 
	HAL_RTC_SetDate(&rtc, &date, RTC_FORMAT_BCD);
	
	HAL_RTC_Init(&rtc);
	
  while (1) {
    ; // Insert thread code here...
    osThreadYield();                            // suspend thread
  }
}

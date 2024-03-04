#include "stm32f4xx_hal.h"
#include "Driver_SPI.h"
#include "Arial12x12.h"
#include "cmsis_os2.h"

/*Funciones prototipo*/
void LCD_Reset(void);
void LCD_Init(void);
void LCD_update(void);
void LCD_wr_data(unsigned char data);
void LCD_wr_cmd(unsigned char cmd);

void symbolToLocalBuffer_L1(uint8_t symbol);
void symbolToLocalBuffer_L2(uint8_t symbol);
void LCD_Clean(void);
void symbolToLocalBuffer(uint8_t line,uint8_t symbol);
void textToLocalBufferL1(char text[]);
void textToLocalBufferL2(char text[]);

void mySPI_Callback(uint32_t event);
void delay (uint32_t n_microsegundos);


/*Variables globales*/
GPIO_InitTypeDef GPIO_InitStruct;
TIM_HandleTypeDef htim7;
unsigned char buffer[512]; // Variable global: array de char(char es de 8 bits), 512*8 = 4096 bits para los pixels del display
extern ARM_DRIVER_SPI Driver_SPI1; //Para acceder a la estructura con numero del recurso utilizado
ARM_DRIVER_SPI* SPIdrv = &Driver_SPI1; // Puntero de tipo ARM_DRIVER_SPI para manejar la estructura

extern osThreadId_t tid_ThDisplay;
uint8_t posicion_L1 = 0; // Me indica la posición dentro de la linea 1
uint8_t posicion_L2 = 0; // Me indica la posición dentro de la linea 2

/*Definiciones de funciones*/
/***********************************************************************************************/
void LCD_Reset(void){
	
	/*Inicializar y configurar el SPI para su comunicacion con el LCD
	Configuración del SPI:
	- Modo Master
	- CPOL1 y CPHA1
	- Organizacion de la información MSB a LSB
	- 8 bits de datos
	- Frecuencia del SCLK, 20MHz
	*/
	SPIdrv->Initialize(mySPI_Callback); //Inicializar the SPI driver
	SPIdrv->PowerControl(ARM_POWER_FULL); //Alimentar el periférico SPI
	SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL1_CPHA1 | ARM_SPI_MSB_LSB | ARM_SPI_DATA_BITS(8), 20000000 ); 
	/*Permite configurar el funcionamiento del periférico. Buscamos la estructura ARM_DRIVER_SPI en Driver_SPI 
	y allí vemos el valor de los parámetros según el modo que queramos, parametro control es una mascara de 22 bits
	y el parametro args es la velocidad del bus (20MHz = 20Mbits/s)*/
	
	/*Configuracion pines GPIO de salida para A0 PF13, Reset PA6 y CS_N PD14 y su valor por defecto*/
	__HAL_RCC_GPIOF_CLK_ENABLE(); // AO PF13
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
	
	__HAL_RCC_GPIOA_CLK_ENABLE(); // Reset PA6
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	__HAL_RCC_GPIOD_CLK_ENABLE(); // CS_N PD14
	
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; 
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = GPIO_PIN_14;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	// Valor por defecto: estan a nivel alto
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOD,GPIO_PIN_14, GPIO_PIN_SET);
	
	/*Generar la señal de Reset*/
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,  GPIO_PIN_RESET); //Ponemos a nivel bajo el pin 
	delay(1); //Esperamos 1 us que es que el tiempo minimo (trw = 1us)
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,  GPIO_PIN_RESET); //Ponemos a nivel alto el pin
	
	/*Retardo de 1ms despues del Reset segun el enunciado*/
	delay(1000);
}
/***********************************************************************************************/

/***********************************************************************************************/
void LCD_Init(void){ // Funcion de configuración para el LCD
	LCD_wr_cmd(0xAE); // Display off
	LCD_wr_cmd(0xA2); // Fija el valor de la relación de la tensión de polarización del LCD a 1/9
	LCD_wr_cmd(0xA0); // El direccionamiento de la RAM de datos del display es la normal
	LCD_wr_cmd(0xC8); // El scan en las salidas COM es el normal
	LCD_wr_cmd(0x22); // Fija la relación de resistencias interna a 2
	LCD_wr_cmd(0x2F); // Power on
	LCD_wr_cmd(0x40); // Display empieza en la línea 0
	LCD_wr_cmd(0xAF); // Display ON
	LCD_wr_cmd(0x81); // Contraste
	LCD_wr_cmd(0x17); // Valor Contraste
	LCD_wr_cmd(0xA4); // Display all points normal
	LCD_wr_cmd(0xA6); // LCD Display normal
}
/***********************************************************************************************/

/***********************************************************************************************/
void LCD_update(void)
{
 int i;
 LCD_wr_cmd(0x00); // 4 bits de la parte baja de la dirección a 0 
 LCD_wr_cmd(0x10); // 4 bits de la parte alta de la dirección a 0
 LCD_wr_cmd(0xB0); // Establecer la direccion de la Página 0 -> COMANDO Page Address Set

 for(i=0;i<128;i++){
	 LCD_wr_data(buffer[i]);
 }

 LCD_wr_cmd(0x00); // 4 bits de la parte baja de la dirección a 0
 LCD_wr_cmd(0x10); // 4 bits de la parte alta de la dirección a 0
 LCD_wr_cmd(0xB1); // Establecer la direccion de la Página 1

 for(i=128;i<256;i++){
	 LCD_wr_data(buffer[i]);
 }

 LCD_wr_cmd(0x00);
 LCD_wr_cmd(0x10);
 LCD_wr_cmd(0xB2); // Establecer la direccion de la Página 2
 for(i=256;i<384;i++){
	 LCD_wr_data(buffer[i]);
 }

 LCD_wr_cmd(0x00);
 LCD_wr_cmd(0x10);
 LCD_wr_cmd(0xB3); // Establecer la direccion de la Página 3


 for(i=384;i<512;i++){
	 LCD_wr_data(buffer[i]);
 }
}
/***********************************************************************************************/

/***********************************************************************************************/
void LCD_wr_data(unsigned char data){ // Funcion para escribir un dato
	ARM_SPI_STATUS SPI_state; // estructura con informacion sobre el estado del SPI
	
	/*Seleccionar CS = 0*/
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	
	/*Seleccionar A0 = 1*/
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_SET);
	
	/*Escribir un dato (data) usando la función SPIDrv->Send(…)*/
	SPIdrv->Send(&data, sizeof(data)); // Comienza a enviar datos a la interfaz SPI, datos es un puntero a un buffer y num es el numero de elementos a enviar
	
	/*Esperar a que se libere el bus: a que deje de estar ocupado*/
	//sThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
	do {
		SPI_state = SPIdrv->GetStatus(); // Obtenemos el estado del SPI
	}while(SPI_state.busy == 1); // Espera a que deje de estar ocupado
	//while(SPIdrv->GetStatus().busy ==1); sin el bucle do, se queda bloqueado ya que solo 
	// leería el estado una sola vez
	
	/*Seleccionar CS = 1*/
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	
}
/***********************************************************************************************/

/***********************************************************************************************/
void LCD_wr_cmd(unsigned char cmd){ // Funcion para escribir un comando
	ARM_SPI_STATUS SPI_state; // estructura con informacion sobre el estado del SPI
	
	/*Seleccionar CS = 0*/
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
	
	/*Seleccionar A0 = 0*/
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, GPIO_PIN_RESET);
	
	/*Escribir un dato (data) usando la función SPIDrv->Send(…)*/
	SPIdrv->Send(&cmd, sizeof(cmd)); // Comienza a enviar datos a la interfaz SPI, datos es un puntero a un buffer y num es el numero de elementos a enviar
	
	/*Esperar a que se libere el bus: a que deje de estar ocupado*/	
	//osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever);
	do {
		//SPI_state = SPIdrv->GetStatus(); // obtenemos el estado del SPI
	}while(SPI_state.busy == 1); // esperar a que deje de estar ocupado
	//while(SPIdrv->GetStatus().busy ==1); sin el bucle do, se queda bloqueado ya que solo 
	// leería el estado una sola vez
	
	/*Seleccionar CS = 1*/
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
	
}
/***********************************************************************************************/

/***********************************************************************************************/
void symbolToLocalBuffer_L1(uint8_t symbol){ 
  uint8_t i, value1, value2;
  uint16_t offset=0;
  
  offset=25*(symbol - ' '); /* Para identificar en Arial12x12 a partir de donde se encuentra el carácter
                               'H' es 0100 1000 - ' ' es 0100 0000 */
  for(i=0; i<12; i++){      // En cada iteraccion cogemos dos bytes, 12*2 = 24 bytes y primer byte la linea son 25 bytes
    value1=Arial12x12[offset+i*2+1];
    value2=Arial12x12[offset+i*2+2];
    
    buffer[i+posicion_L1]=value1;     // Cada columna del 0 al 7
    buffer[i+128+posicion_L1]=value2; // Cda columna sería el 8 
		
		// PARA OTRO LUGAR de PAGINA podríamos hacer esto, pero para ello creamos una funcion similar a esta
		//buffer[i+128]=value1;   // Cada columna del 0 al 7 
    //buffer[i+256]=value2;   // Cda columna sería el 8 
  }
	posicion_L1 = posicion_L1 + Arial12x12[offset]; /*Con Array12x12[offset] el que no se usa para el buffer,
	// se usa para la posicion de la linea*/
}
/***********************************************************************************************/

/***********************************************************************************************/
void symbolToLocalBuffer_L2(uint8_t symbol){
	uint8_t i, value1, value2;
  uint16_t offset=0;
  
  offset=25*(symbol - ' '); /* Para identificar en Arial12x12 a partir de donde se encuentra el carácter
                               'H' es 0100 1000 - ' ' es 0100 0000 */
  for(i=0; i<12; i++){      // En cada iteraccion cogemos dos bytes, 12*2 = 24 bytes y primer byte la linea son 25 bytes
    value1=Arial12x12[offset+i*2+1];
    value2=Arial12x12[offset+i*2+2];
    
		buffer[i+256+posicion_L2]=value1;   // Cada columna del 0 al 7 
    buffer[i+384+posicion_L2]=value2;   // Cda columna sería el 8 
  }
	posicion_L2 = posicion_L2 + Arial12x12[offset]; /*Con Array12x12[offset] el que no se usa para el buffer,
	// se usa para la posicion de la linea*/
}
/***********************************************************************************************/

/***********************************************************************************************/
void symbolToLocalBuffer(uint8_t line,uint8_t symbol){
	if (line == 1) {
		symbolToLocalBuffer_L1(symbol);
	}
	else{
		symbolToLocalBuffer_L2(symbol);
	}
}
/***********************************************************************************************/

/***********************************************************************************************/
void textToLocalBufferL1(char text[]){
	
	for(int i = 0; i < strlen(text); i++){
		symbolToLocalBuffer_L1(text[i]); 
	}
}
/***********************************************************************************************/

/***********************************************************************************************/
void textToLocalBufferL2(char text[]){
	
	for(int i = 0; i < strlen(text); i++){
		symbolToLocalBuffer_L2(text[i]); 
	}
}
/***********************************************************************************************/

/***********************************************************************************************/
void LCD_Clean(void){
	for(int i = 0; i < 512; i++){
		buffer[i] = 0x00; 
	}
}
/***********************************************************************************************/

/***********************************************************************************************/
void delay (uint32_t n_microsegundos)
{
	/*Configuracion Timer 7 para generar un evento pasados n microsegundos en modo basico sin interrupciones*/
	// Hemos establecido unn SYSCLK de 168 MHz, le llegarían 84 MHz (bus APB1 Timer Clock)
	htim7.Instance = TIM7; // Como maximo el bus APB1 Timer Clock tiene una frecuencia de 45 MHz (Si establecemos a 84MHz no se sabe si funciona)
	htim7.Init.Prescaler = 83; // 84M/84 (Hz) -> 1MHz -> us
	htim7.Init.Period = n_microsegundos-1; // Contamos hasta el valor que queremos 
	
	__HAL_RCC_TIM7_CLK_ENABLE();// Habilitar el reloj para el periférico
	
	HAL_TIM_Base_Init(&htim7); // Iniciar la congiguracion del Timer
	HAL_TIM_Base_Start(&htim7); // Iniciar el Timer
	
	/*Esperar a que se active el FLAG del registro de Match correspondiente*/
	while(__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE) == false);
	//while(__HAL_TIM_GET_COUNTER(&htim7)<(n_microsegundos-1)){} -> Registro CNT TIM7->CNT
		
	/*Borrar el FLAG*/
	__HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
	
	/*Para el Timer y ponerlo a 0 para la siguiente llamada a funcion*/
	HAL_TIM_Base_Stop(&htim7); //Parar el timer
	__HAL_TIM_SET_COUNTER(&htim7, 0); //Establecemos la cuenta a 0
	//HAL_TIM_Base_DeInit(&htim7);
}
/***********************************************************************************************/

/***********************************************************************************************/
void mySPI_Callback(uint32_t event){
	switch (event)
	{
    case ARM_SPI_EVENT_TRANSFER_COMPLETE:
        /* Success: Wakeup Thread */
        //osThreadFlagsSet(tid_ThDisplay, 0x01);
        break;
    case ARM_SPI_EVENT_DATA_LOST:
        /*  Occurs in slave mode when data is requested/sent by master
            but send/receive/transfer operation has not been started
            and indicates that data is lost. Occurs also in master mode
            when driver cannot transfer data fast enough. */
        //__breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
        break;
    case ARM_SPI_EVENT_MODE_FAULT:
        /*  Occurs in master mode when Slave Select is deactivated and
            indicates Master Mode Fault. */
        //__breakpoint(0);  /* Error: Call debugger or replace with custom error handling */
        break;
    }
}
/***********************************************************************************************/






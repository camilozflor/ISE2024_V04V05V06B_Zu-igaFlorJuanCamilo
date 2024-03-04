#ifndef __LCD_H
#define __LCD_H

#include "stdint.h"

/* Declaraciones de funciones*/
void LCD_Reset(void);// Inicializacion y configuracion de SPI y pines GPIO
void LCD_Init(void);// Configuración para el LCD
void LCD_update(void);// Establecer en la RAM del display lo que queremos representar
void LCD_wr_data(unsigned char data);
void LCD_wr_cmd(unsigned char cmd);
//Funciones que envían al buffer el símbolo que queremos representar según la linea, de este 
//manera no tenemos que rellenar nosotros el buffer manualmente, ya que en Arial12x12.h se 
//encuentran los pixels(bits) que se deben activar en el LCD para cada símbolo
void symbolToLocalBuffer_L1(uint8_t symbol);
void symbolToLocalBuffer_L2(uint8_t symbol);
void symbolToLocalBuffer(uint8_t line,uint8_t symbol); //Funcion que representa un simbolo indicando la linea donde se quiere representar
void textToLocalBufferL1(char text[]);
void textToLocalBufferL2(char text[]);			
void LCD_Clean(void); //Para limpiar el buffer del display
void delay (uint32_t n_microsegundos);

#endif /* __LCD_H */

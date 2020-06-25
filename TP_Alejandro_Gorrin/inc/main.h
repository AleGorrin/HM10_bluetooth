/*
 * main.h
 *
 *  Created on: Jun 22, 2020
 *      Author: rocku
 */

#ifndef TP_ALEJANDROG_PRUEBAS_INC_MAIN_H_
#define TP_ALEJANDROG_PRUEBAS_INC_MAIN_H_

#define UART_PC        UART_USB
#define UART_BLUETOOTH UART_232

uint8_t data = 0;

int a;
bool_t periodo = TRUE;
tick_t periodo_normal = 5000;
tick_t periodo_emergencia = 3000;
uint32_t d = 10;
int p = 5;
uint16_t b = 100;
uint16_t muestra = 0;
uint32_t distanceInCms;

bool_t hm10bleTest( int32_t uart );
void hm10blePrintATCommands( int32_t uart );
void inicializacion_sensor_ultrasonido(uint32_t d);
void modo_normal(tick_t periodo_normal, tick_t periodo_emergencia, uint32_t d, uint16_t b);
void modo_emergencia(tick_t periodo_normal, tick_t periodo_emergencia, uint32_t d, uint16_t b);
void configuracion(void);



#endif /* TP_ALEJANDROG_PRUEBAS_INC_MAIN_H_ */

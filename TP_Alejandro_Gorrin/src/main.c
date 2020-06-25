/* Copyright 2017, Danilo Zecchin.
 * Copyright 2018, Eric Pernia.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*===========================================================================*/

#include "sapi.h"        // <= Inclusion de la Biblioteca sAPI
#include "main.h"
#include <string.h>

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void )
{
   // ---------- CONFIGURACIONES ------------------------------

   boardConfig();												            // Inicializar y configurar la plataforma
   uartConfig( UART_PC, 115200 );									   // Inicializar UART_USB para conectar a la PC

   uartConfig( UART_BLUETOOTH, 9600 );							      // Inicializar UART_232 para conectar al modulo bluetooth
   
   pwmConfig( 0, PWM_ENABLE );
   pwmConfig( PWM7, PWM_ENABLE_OUTPUT );
   
   adcConfig( ADC_ENABLE );
   dacConfig( DAC_ENABLE );
   
   delay_t delaySensor;
   delay_t delay1;

   delayConfig( &delay1, periodo_emergencia );
   delayConfig( &delaySensor, periodo_normal);
   
   static char uartBuff[10];
   uint8_t data = 0;
   //uint32_t distanceInCms;
   delay(100);
   bool_t flag1 =0 ;
   bool_t flag2 =0 ;
   
   inicializacion_sensor_ultrasonido(d);

   // ---------- REPETIR POR SIEMPRE --------------------------
	while(TRUE)
	{
      // Si leo un dato de una UART lo envio a al otra (bridge)
      if( uartReadByte( UART_PC, &data ) ) {
         uartWriteString( UART_BLUETOOTH, data );
      }
      if( uartReadByte( UART_BLUETOOTH, &data ) ) {
         if (data == 'w' || flag2 == 1){
            modo_normal(periodo_normal, periodo_emergencia, d, b);
            flag2 = 0;
            flag1 = 1;
         }
         else if(data == 'e' || flag1 == 1){
            modo_emergencia(periodo_normal, periodo_emergencia, d, b);
            flag1 = 0;
            flag2 = 1;
         }
         else if( data == 'c' ) {
            configuracion();
         }
      }
   }
   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamente sobre un microcontrolador y no es llamado por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

void modo_normal(tick_t periodo_normal, tick_t periodo_emergencia, uint32_t d, uint16_t b){

   delay_t delaySensor;
   delay_t delay1;

   delayConfig( &delay1, periodo_emergencia );
   delayConfig( &delaySensor, periodo_normal);

   delay_t verificar;
   delayConfig( &verificar, 1000 );

   static char uartBuff[10];
   uint8_t data = 0;
   uint32_t distanceInCms;
   while(1){
      distanceInCms = ultrasonicSensorGetDistance(ULTRASONIC_SENSOR_0, CM);
      if (distanceInCms > d) {
         if (delayRead(&delaySensor)){
            uartWriteString( UART_BLUETOOTH, "Distancia: " );
            itoa( distanceInCms, uartBuff, 10 );
            uartWriteString( UART_BLUETOOTH, uartBuff );
            uartWriteString( UART_BLUETOOTH, " cm y un valor de tension : " );
            muestra = adcRead( CH1 );
            itoa( muestra, uartBuff, 10 );
            uartWriteString( UART_BLUETOOTH, uartBuff );
            uartWriteString( UART_BLUETOOTH, " \r\n" );
            printf( "Distancia: %d cm y valor de tension: %d \r\n", distanceInCms, muestra);
            if (muestra <= b){
               printf( "Valor bajo de bateria.\r\n");
               muestra = adcRead( CH1 );
               uartWriteString( UART_BLUETOOTH, "Valor bajo de bateria de: " );
               itoa( muestra, uartBuff, 10 );
               uartWriteString( UART_BLUETOOTH, uartBuff );
               uartWriteString( UART_BLUETOOTH, " \r\n" );
               break;
            }
         }
      }
      else if (distanceInCms <= d ){
         if (delayRead(&verificar)){
            printf( "Se detecto un obstaculo a una distancia: %d cm.\r\n", distanceInCms);
            uartWriteString( UART_BLUETOOTH, "Se detecto un obstaculo a una distancia: " );
            itoa( distanceInCms, uartBuff, 10 );
            uartWriteString( UART_BLUETOOTH, uartBuff );
            uartWriteString( UART_BLUETOOTH, " cm \r\n" );
            break;
         }
      }
   }
}

void modo_emergencia(tick_t periodo_normal, tick_t periodo_emergencia, uint32_t d, uint16_t b){
   delay_t delaySensor;
   delayConfig( &delaySensor, periodo_normal);

   delay_t delay1;

   delayConfig( &delay1, periodo_emergencia );

   delay_t verificar2;
   delayConfig( &verificar2, 1000 );

   static char uartBuff[10];
   uint8_t data = 0;
   uint32_t distanceInCms;
   int p = 5;
   while(1){
      distanceInCms = ultrasonicSensorGetDistance(ULTRASONIC_SENSOR_0, CM);
      if (distanceInCms <= d || muestra <= 100){
         if (delayRead(&delay1)){
            if ((distanceInCms <= d) && (muestra <= b)){
               printf( "Alerta maxima \r\n");
               uartWriteString( UART_BLUETOOTH, "Alerta maxima \r\n" );
               printf( "Obstaculo visto a una distancia: %d cm y valor bajo de bateria: %d \r\n", distanceInCms, muestra);
               uartWriteString( UART_BLUETOOTH, "Obstaculo visto a una distancia: " );
               itoa( distanceInCms, uartBuff, 10 );
               uartWriteString( UART_BLUETOOTH, uartBuff );
               uartWriteString( UART_BLUETOOTH, " cm y un valor de tension bajo de : " );
               muestra = adcRead( CH1 );
               itoa( muestra, uartBuff, 10 );
               uartWriteString( UART_BLUETOOTH, uartBuff );
               uartWriteString( UART_BLUETOOTH, " \r\n" );
            }
            if (distanceInCms <= d){
               printf( "Alerta de obstaculo a una distancia: %d cm \r\n", distanceInCms);
               uartWriteString( UART_BLUETOOTH, "Se detecto un obstaculo a una distancia: " );
               itoa( distanceInCms, uartBuff, 10 );
               uartWriteString( UART_BLUETOOTH, uartBuff );
               uartWriteString( UART_BLUETOOTH, " cm \r\n" );
            }
            if (muestra <= b){
               muestra = adcRead( CH1 );
               printf( "Alerta de bateria con un valor: %d \r\n", muestra);
               uartWriteString( UART_BLUETOOTH, "Valor bajo de bateria de: " );
               itoa( muestra, uartBuff, 10 );
               uartWriteString( UART_BLUETOOTH, uartBuff );
               uartWriteString( UART_BLUETOOTH, " \r\n" );
            }
         }
      }
      else if (distanceInCms > d && ( muestra > b )){
         if (delayRead(&verificar2)){
            printf( "Saliendo del estado Emergencia.\r\n");
            uartWriteString( UART_BLUETOOTH, "Saliendo del estado Emergencia. \r\n" );
            break;
         }
      }
   }
}

void modo_informacion_supervisor(int p)
{
   uint32_t distanceInCms;
   ultrasonicSensorConfig( ULTRASONIC_SENSOR_0, ULTRASONIC_SENSOR_ENABLE );
   distanceInCms = ultrasonicSensorGetDistance(ULTRASONIC_SENSOR_0, CM);
   printf( "Distancia: %d cm.\r\n", distanceInCms );
   int i=0;
   for (i=0; i < p ; i++){
      delay(1000);
   }
}

void inicializacion_sensor_ultrasonido(uint32_t d)
{
   uint32_t distanceInCms;
   ultrasonicSensorConfig( ULTRASONIC_SENSOR_0, ULTRASONIC_SENSOR_ENABLE );
   delay(100);
   while(1){
      distanceInCms = ultrasonicSensorGetDistance(ULTRASONIC_SENSOR_0, CM);
      if (d < distanceInCms ){
         //printf( "Inicializado sensor de ultrasonido\r\n" );
         uartWriteString( UART_BLUETOOTH, "Inicializado sensor de ultrasonido\r\n" );
         break;
      }
   }
}
bool_t hm10bleTest( int32_t uart )
{
   uartWriteString( uart, "AT\r\n" );
   return waitForReceiveStringOrTimeoutBlocking( uart,
                                                 "OK\r\n", strlen("OK\r\n"),
                                                 1000 );
}
void hm10blePrintATCommands( int32_t uart )
{
   uartWriteString( uart, "AT+HELP?\r\n" );
}
void configuracion(void)
{
   uartWriteString( UART_PC, "Que desea configurar?\r\nA. Periodicidad de informacion del supervisor\r\nB. Sensor ultrasonido\r\nC. Sensor de bateria\r\nD. Regresar valores por defecto\r\nE. Salir\r\n" );
   uartWriteString( UART_BLUETOOTH, "Que desea configurar?\r\nA. Periodicidad de informacion del supervisor\r\nB. Sensor ultrasonido\r\nC. Sensor de bateria\r\n" );
   while(1){
      if( uartReadByte( UART_BLUETOOTH, &data ) ) {
         if( data == 'a'){
            uartWriteString( UART_PC, "Introduzca periodo en seg del supervisor\r\n" );
            uartWriteString( UART_BLUETOOTH, "Introduzca periodo en seg del supervisor\r\n" );
         }
         else if( data == 'j'){
            uartWriteString( UART_PC, "Nuevo timepo de periodo de envio de datos a 3 seg\r\n" );
            uartWriteString( UART_BLUETOOTH, "Nuevo timepo de periodo de envio de datos a 3 seg\r\n" );
            periodo_normal = 3000;
         }
         else if( data == 'k'){
            uartWriteString( UART_PC, "Nuevo timepo de periodo de envio de datos a 4 seg\r\n" );
            uartWriteString( UART_BLUETOOTH, "Nuevo timepo de periodo de envio de datos a 4 seg\r\n" );
            periodo_normal = 4000;
         }
         else if( data == 'l'){
            uartWriteString( UART_PC, "Nuevo timepo de periodo de envio de datos a 5 seg\r\n" );
            uartWriteString( UART_BLUETOOTH, "Nuevo timepo de periodo de envio de datos a 5 seg\r\n" );
            periodo_normal = 5000;
         }
         else if( data == 'b'){
            uartWriteString( UART_PC, "Introduzca valor minimo en cm a detectar por el sensor\r\n" );
            uartWriteString( UART_BLUETOOTH, "Introduzca valor minimo en cm a detectar por el sensor\r\n" );
         }
         else if( data == 'm'){
            uartWriteString( UART_PC, "Nuevo valor minimo a detecar 4 cm\r\n" );
            uartWriteString( UART_BLUETOOTH, "Nuevo valor minimo a detectar 4 cm\r\n" );
            d = 4;
         }
         else if( data == 'n'){
            uartWriteString( UART_PC, "Nuevo valor minimo a detecar 7 cm\r\n" );
            uartWriteString( UART_BLUETOOTH, "Nuevo valor minimo a detectar 7 cm\r\n" );
            d = 7;
         }
         else if( data == 'o'){
            uartWriteString( UART_PC, "Nuevo valor minimo a detecar 10 cm\r\n" );
            uartWriteString( UART_BLUETOOTH, "Nuevo valor minimo a detectar 10 cm\r\n" );
            d = 10;
         }
         else if( data == 'k'){
            uartWriteString( UART_PC, "Introduzca valor minimo de bateria a detectar\r\n" );
            uartWriteString( UART_BLUETOOTH, "Introduzca valor minimo de bateria a detectar\r\n" );
         }
         else if( data == 'e'){
            uartWriteString( UART_PC, "Nuevo valor minimo a detecar de 200\r\n" );
            uartWriteString( UART_BLUETOOTH, "Nuevo valor minimo a detectar de 200\r\n" );
            b = 200;
         }
         else if( data == 'f'){
            uartWriteString( UART_PC, "Nuevo valor minimo a detecar de 300\r\n" );
            uartWriteString( UART_BLUETOOTH, "Nuevo valor minimo a detectar de 300\r\n" );
            b = 300;
         }
         else if( data == 'g'){
            uartWriteString( UART_PC, "Nuevo valor minimo a detecar de 250\r\n" );
            uartWriteString( UART_BLUETOOTH, "Nuevo valor minimo a detectar de 250\r\n" );
            b = 250;
         }
         else if( data == 'i'){
            uartWriteString( UART_PC, "Valores defauld del sistema\r\n" );
            uartWriteString( UART_BLUETOOTH, "Valores defauld del sistema\r\n" );
            periodo_normal = 5000;
            periodo_emergencia = 3000;
            d = 10;
            b = 100;
         }
         else if (data == 's'){
            break;
         }
      }
   }
}

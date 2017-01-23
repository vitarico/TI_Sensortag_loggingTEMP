/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC26XX EM
 *   - sensortag-cc26xx: CC26XX sensortag
 *   - The CC2650 LaunchPad
 *
 *   By default, the example will build for the srf06-cc26xx board. To switch
 *   between platforms:
 *   - make clean
 *   - make BOARD=sensortag-cc26xx savetarget
 *
 *     or
 *
 *     make BOARD=srf06-cc26xx savetarget
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_SENSOR_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_SENSOR_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "button-sensor.h"
#include "board-peripherals.h"
#include "cc26xx-uart.h"

#include "ti-lib.h"
#define printf(x) 
#define PRINTF(x)

/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       CLOCK_SECOND
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_RED
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_SENSOR_NONE         (void *)0xFFFFFFFF

#define CC26XX_DEMO_SENSOR_1     &button_left_sensor
#define CC26XX_DEMO_SENSOR_2     &button_right_sensor

#define CC26XX_DEMO_SENSOR_3     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_4     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_5     &reed_relay_sensor
/*---------------------------------------------------------------------------*/
static struct etimer et;
/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */
#define SENSOR_READING_PERIOD CLOCK_SECOND * 10

static struct ctimer opt_timer, hdc_timer;
/*---------------------------------------------------------------------------*/

//Arrays to store Data
// Light

uint16_t alight[2] = {0,0};
uint16_t ahdc[4] = {0,0};


static void write_byte(uint8_t data){

  switch (data>>4){
    case 0x0: cc26xx_uart_write_byte(48);
    break;
    case 0x1: cc26xx_uart_write_byte(49);
    break;
    case 0x2: cc26xx_uart_write_byte(50);
    break;
    case 0x3: cc26xx_uart_write_byte(51);
    break;
    case 0x4: cc26xx_uart_write_byte(52);
    break;
    case 0x5: cc26xx_uart_write_byte(53);
    break;
    case 0x6: cc26xx_uart_write_byte(54);
    break;
    case 0x7: cc26xx_uart_write_byte(55);
    break;
    case 0x8: cc26xx_uart_write_byte(56);
    break;
    case 0x9: cc26xx_uart_write_byte(57);
    break; 
    case 0xA: cc26xx_uart_write_byte(65);
    break;
    case 0xB: cc26xx_uart_write_byte(66);
    break;
    case 0xC: cc26xx_uart_write_byte(67);
    break;
    case 0xD: cc26xx_uart_write_byte(68);
    break;
    case 0xE: cc26xx_uart_write_byte(69);
    break;
    case 0xF: cc26xx_uart_write_byte(70);
    break;
    default: 
    break;
  }
    switch (data&0xF){
    case 0x0: cc26xx_uart_write_byte(48);
    break;
    case 0x1: cc26xx_uart_write_byte(49);
    break;
    case 0x2: cc26xx_uart_write_byte(50);
    break;
    case 0x3: cc26xx_uart_write_byte(51);
    break;
    case 0x4: cc26xx_uart_write_byte(52);
    break;
    case 0x5: cc26xx_uart_write_byte(53);
    break;
    case 0x6: cc26xx_uart_write_byte(54);
    break;
    case 0x7: cc26xx_uart_write_byte(55);
    break;
    case 0x8: cc26xx_uart_write_byte(56);
    break;
    case 0x9: cc26xx_uart_write_byte(57);
    break; 
    case 0xA: cc26xx_uart_write_byte(65);
    break;
    case 0xB: cc26xx_uart_write_byte(66);
    break;
    case 0xC: cc26xx_uart_write_byte(67);
    break;
    case 0xD: cc26xx_uart_write_byte(68);
    break;
    case 0xE: cc26xx_uart_write_byte(69);
    break;
    case 0xF: cc26xx_uart_write_byte(70);
    break;
    default: 
    break;
  }

}

static void print_data(void){

  //light

  write_byte(alight[0]>>8);
  write_byte(alight[0]&0xFF); 
  cc26xx_uart_write_byte(44);
  write_byte(alight[1]&0xFF);
  cc26xx_uart_write_byte(59);
  //TMP

  write_byte(ahdc[0]>>8);
  write_byte(ahdc[0]&0xFF); 
  cc26xx_uart_write_byte(44);
  write_byte(ahdc[1]&0xFF);
  cc26xx_uart_write_byte(59);

  //HUM

  write_byte(ahdc[2]>>8);
  write_byte(ahdc[2]&0xFF); 
  cc26xx_uart_write_byte(44);
  write_byte(ahdc[3]&0xFF);

}

static void init_opt_reading(void *not_used);
static void init_hdc_reading(void *not_used);

/*---------------------------------------------------------------------------*/
static void
get_hdc_reading()
{
  int value;
  clock_time_t next = SENSOR_READING_PERIOD;

  value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_TEMP);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    ahdc[0]=value/100;
    ahdc[1]=value%100;
  } else {
    //printf("HDC: Temp Read Error\n");
  }

  value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_HUMIDITY);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    ahdc[2]=value/100;
    ahdc[3]=value%100;
  } else {
    //printf("HDC: Humidity Read Error\n");
  }

  ctimer_set(&hdc_timer, next, init_hdc_reading, NULL);
}
/*---------------------------------------------------------------------------*/
static void
get_light_reading()
{
  int value;
  clock_time_t next = SENSOR_READING_PERIOD;

  value = opt_3001_sensor.value(0);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    alight[0]=value/100;
    alight[1]=value%100;
  } else {
    //printf("OPT: Light Read Error\n");
  }

  /* The OPT will turn itself off, so we don't need to call its DEACTIVATE */
  ctimer_set(&opt_timer, next, init_opt_reading, NULL);
}

/*---------------------------------------------------------------------------*/
static void
init_opt_reading(void *not_used)
{
  SENSORS_ACTIVATE(opt_3001_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_hdc_reading(void *not_used)
{
  SENSORS_ACTIVATE(hdc_1000_sensor);
}
/*---------------------------------------------------------------------------*/
static void
init_sensor_readings(void)
{
  SENSORS_ACTIVATE(hdc_1000_sensor);
  SENSORS_ACTIVATE(opt_3001_sensor);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{

  PROCESS_BEGIN();

  printf("CC26XX demo\n");

  etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
  init_sensor_readings();

  while(1) {

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {
        leds_toggle(CC26XX_DEMO_LEDS_PERIODIC);
        etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
      }
    } else if(ev == sensors_event) {
      if(data == CC26XX_DEMO_SENSOR_1) {
        print_data();
        leds_toggle(CC26XX_DEMO_LEDS_BUTTON);
      }  else if(ev == sensors_event && data == &opt_3001_sensor) {
        get_light_reading();
      } else if(ev == sensors_event && data == &hdc_1000_sensor) {
        get_hdc_reading();
      } 
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 * @}
 */
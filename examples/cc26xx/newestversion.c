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
#include "random.h"
#include "button-sensor.h"
#include "batmon-sensor.h"
#include "board-peripherals.h"
#include "ti-lib.h"

#include <stdio.h>
#include <stdint.h>
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       CLOCK_SECOND // sets the main Process to run every 1 second
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
#define SENSOR_READING_PERIOD (CLOCK_SECOND * 10)

static struct ctimer /* bmp_timer, opt_timer, hdc_timer, tmp_timer,*/ sensor_timer;
/*---------------------------------------------------------------------------*/

static void activate_sensor_readings(void *not_used);

//Arrays to store Data
// Light
uint16_t alight[2] = {0,0};
// Altimeter/Pressure
uint16_t abmp[4] = {0,0,0,0};
// Temperature
uint16_t atmp[4] = {0,0,0,0};
// Humidity
uint16_t ahdc[4] = {0,0,0,0};
// Battery status and temperature
uint16_t abat[4] = {0,0,0,0};
// Final array containing values to broadcast
uint16_t asend[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

/*---------------------------------------------------------------------------*/
/*#define STATE_NORMAL           0
#define STATE_NOTIFY_OBSERVERS 1
#define STATE_VERY_SLEEPY      2
*/
/*---------------------------------------------------------------------------*/
/*static struct stimer st_duration;
static struct stimer st_interval;
static struct stimer st_min_mac_on_duration;
static struct etimer et_periodic;
static process_event_t event_new_config;
static uint8_t state;

*/
/*---------------------------------------------------------------------------*/

/*typedef struct sleepy_config_s {
  unsigned long interval;
  unsigned long duration;
  uint8_t mode;
} sleepy_config_t;

sleepy_config_t config;
*/
/*---------------------------------------------------------------------------*/
/*static void
switch_to_normal(void)
{
  state = STATE_NOTIFY_OBSERVERS;
*/
  /*
   * Stay in normal mode for 'duration' secs.
   * Transition back to normal in 'interval' secs, _including_ 'duration'
   */
 /* stimer_set(&st_duration, config.duration);
  stimer_set(&st_interval, config.interval);
}
*/
/*---------------------------------------------------------------------------*/

/*static void
switch_to_very_sleepy(void)
{
  state = STATE_VERY_SLEEPY;
}
*/
/*---------------------------------------------------------------------------*/
/*static void
switch_to_normal(void)
{
  state = STATE_NOTIFY_OBSERVERS;
*/
  /*
   * Stay in normal mode for 'duration' secs.
   * Transition back to normal in 'interval' secs, _including_ 'duration'
   */
 /* stimer_set(&st_duration, config.duration);
  stimer_set(&st_interval, config.interval);
}*/
/*---------------------------------------------------------------------------*/

static void print_data(void){

  asend[0] = alight[0];
  asend[1] = alight[1];
  asend[2] = abmp[0];
  asend[3] = abmp[1];
  asend[4] = abmp[2];
  asend[5] = abmp[3];
  asend[6] = atmp[0];
  asend[7] = atmp[1];
  asend[8] = atmp[2];
  asend[9] = atmp[3];
  asend[10] = ahdc[0];
  asend[11] = ahdc[1];
  asend[12] = ahdc[2];
  asend[13] = ahdc[3];
  asend[14] = abat[0];
  asend[15] = abat[1];
  printf("OPT: Light=%d.%02d\n, BMP: %d.%02d, %d.%02d\n, TMP: %d.%03d, t%d.%03d, HDC: %d.%02d, HUMIDITY: %d.%02d\n, BAT: %d, %d\n", alight[0], alight[1], abmp[0], abmp[1], abmp[2],abmp[3], atmp[0],atmp[1],atmp[2],atmp[3],ahdc[0],ahdc[1],ahdc[2],ahdc[3],abat[0],abat[1]);

}
/*---------------------------------------------------------------------------*/

static void
get_sensor_readings(void)
{
  int value;

  value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_PRESS);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    abmp[0]=value/100;
    abmp[1]=value%100;
  } 

  value = bmp_280_sensor.value(BMP_280_SENSOR_TYPE_TEMP);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    abmp[2]=value/100;
    abmp[3]=value%100;
  } 

  SENSORS_DEACTIVATE(bmp_280_sensor);

  //XXX debug vars

  //int value_all = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_ALL);
  int value_amb = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_AMBIENT);
  
  value=value_amb;
  printf("value %x \n",value);
 // printf("value_all %d \n",value_all);
  printf("value_amb %d \n",value_amb);
  if(value != CC26XX_SENSOR_READING_ERROR) {
  
  atmp[0]=value/1000;
  atmp[1]=value%1000;

  value = tmp_007_sensor.value(TMP_007_SENSOR_TYPE_OBJECT);
  atmp[2]=value/1000;
  atmp[3]=value%1000;
}
  SENSORS_DEACTIVATE(tmp_007_sensor);

  value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_TEMP);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    ahdc[0]=value/100;
    ahdc[1]=value%100;

  } 
  value = hdc_1000_sensor.value(HDC_1000_SENSOR_TYPE_HUMIDITY);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    ahdc[2]=value/100;
    ahdc[3]=value%100;
  }

  value = opt_3001_sensor.value(0);
  if(value != CC26XX_SENSOR_READING_ERROR) {
    alight[0]=value/100;
    alight[1]=value%100;
  } 

  ctimer_set(&sensor_timer, SENSOR_READING_PERIOD, activate_sensor_readings, NULL);

}
/*---------------------------------------------------------------------------*/
static void
get_sync_sensor_readings(void)
{
  int value;

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_TEMP);
  abat[0]=value;

  value = batmon_sensor.value(BATMON_SENSOR_TYPE_VOLT);
  abat[1]=value;

  return;
}
/*---------------------------------------------------------------------------*/
static void
init_sensors(void)
{
  SENSORS_ACTIVATE(reed_relay_sensor);
  SENSORS_ACTIVATE(batmon_sensor);
}
/*---------------------------------------------------------------------------*/
static void
activate_sensor_readings(void *not_used)
{
  SENSORS_ACTIVATE(hdc_1000_sensor);
  SENSORS_ACTIVATE(tmp_007_sensor);
  SENSORS_ACTIVATE(opt_3001_sensor);
  SENSORS_ACTIVATE(bmp_280_sensor);
}
static void
init_sensor_readings(void)
{
  void *not_used=NULL;
  activate_sensor_readings(not_used);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{

  PROCESS_BEGIN();

  printf("CC26XX demo\n");

  //config.mode = VERY_SLEEPY_MODE_OFF;
  //config.interval = PERIODIC_INTERVAL_DEFAULT;
  //config.duration = NORMAL_OP_DURATION_DEFAULT;

  //state = STATE_NORMAL;

  //event_new_config = process_alloc_event();
  //switch_to_normal();

  init_sensors();
  etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
  get_sync_sensor_readings();
  init_sensor_readings();
  

  while(1) {

    printf("time = %lu\n", clock_seconds());

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {
        leds_toggle(CC26XX_DEMO_LEDS_PERIODIC);

        get_sync_sensor_readings();

        etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
      }
    } else if(ev == sensors_event) {
      //wait until the last sensor is ready, since tmp takes a while
      // if not all ready, set timer and try in a while again

      get_sensor_readings();
      if(data == CC26XX_DEMO_SENSOR_1) {
        print_data();
        leds_toggle(CC26XX_DEMO_LEDS_BUTTON);
      } else if(data == CC26XX_DEMO_SENSOR_5) {
        if(buzzer_state()) {
          buzzer_stop();
        } else {
          buzzer_start(1000);
        }
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

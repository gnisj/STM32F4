/*
    ChibiOS/RT - Copyright (C) 2006-2013 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "ch.h"
#include "hal.h"
#include "test.h"
#include "chprintf.h"
#include "shell.h"
#include "lis3dsh.h" /* Location: /../ChibiOS_2.6.5/os/various/devices_lib/accel */
#include "usbcfg.h"
/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

/*===========================================================================*/
/* Command line related.                                                     */
/*===========================================================================*/

#define SHELL_WA_SIZE   THD_WA_SIZE(2048)
#define TEST_WA_SIZE    THD_WA_SIZE(256)

static void cmd_mem(BaseSequentialStream *chp, int argc, char *argv[]) {
  size_t n, size;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: mem\r\n");
    return;
  }
  n = chHeapStatus(NULL, &size);
  chprintf(chp, "Core free memory : %u bytes\r\n", chCoreStatus());
  chprintf(chp, "Heap fragments   : %u\r\n", n);
  chprintf(chp, "Heap free total  : %u bytes\r\n", size);
}

static void cmd_acc(BaseSequentialStream *chp, int argc, char *argv[]) {
  int32_t myx, myy, myz, myt, fifostatus, ctrlreg3, ctrlreg4, ctrlreg5, ctrlreg6;
  static int8_t xAxisL, xAxisH, yAxisL, yAxisH, zAxisL, zAxisH;   
  static int16_t xAxis, yAxis, zAxis;   
  systime_t mytime;
  unsigned i;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: acc\r\n");
    return;
  }

  myx = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_X_H);
  myy = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Y_H);
  myz = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Z_H);
  myt = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_T);
  mytime = chTimeNow(); 
  fifostatus = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_FIFO_SRC);
  ctrlreg3 = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_CTRL_REG3);
  ctrlreg4 = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_CTRL_REG4);
  ctrlreg5 = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_CTRL_REG5);
  ctrlreg6 = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_CTRL_REG6);

  /* 
for (i = 0; i < 10000; i++) {
      xAxisL = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_X_L);
      xAxisH = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_X_H);
      yAxisL = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Y_L);
      yAxisH = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Y_H);
      zAxisL = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Z_L);
      zAxisH = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Z_H);
      //chprintf(chp, "Timeae: %u seconds\r\n", mytime);
 }
*/
  chprintf(chp, "Time    :  %u seconds\r\n", mytime);

  /*chprintf(chp, "X-axis  : %u G\r\n", (uint8_t)myx);
  chprintf(chp, "Y-axis  : %u G\r\n", myy);
  chprintf(chp, "Z-axis  : %u G\r\n", myz);
  chprintf(chp, "Temp    : %u C\r\n", myt+25);
  chprintf(chp, "X-axisL : %x G\r\n", xAxisL);
  chprintf(chp, "X-axisH : %x G\r\n", xAxisH);
  chprintf(chp, "X-axis  : %x G\r\n", xAxis);
  chprintf(chp, "FIFO_SRC: %x \r\n", fifostatus & 0xFF);
  chprintf(chp, "LIS3DSH_CTRL_REG3: %x \r\n", ctrlreg3 & 0xFF);
  chprintf(chp, "LIS3DSH_CTRL_REG4: %x \r\n", ctrlreg4 & 0xFF);
  chprintf(chp, "LIS3DSH_CTRL_REG5: %x \r\n", ctrlreg5 & 0xFF);
  chprintf(chp, "LIS3DSH_CTRL_REG6: %x \r\n", ctrlreg6 & 0xFF);
  */
  while (1) {
    // chprintf(chp,"%x %x %x \r\n", (xAxisH & 0xFF) , (yAxisH & 0xFF), (zAxisH & 0xFF));
    xAxisH = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_X_H);
    yAxisH = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Y_H);
    zAxisH = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Z_H);
    
    long t = ((long)xAxisH << 8) | xAxisL;
    if (t >= 32768)
      t -=65536;
    xAxis = (int)t;
    
    //    xAxis = (int16_t)((((uint8_t) xAxisH) << 8) | ((uint8_t) xAxisL));
    //chprintf(chp, "X-axis  %x  %x  %x \r\n", xAxisL, xAxisH, xAxis);
    chprintf(chp, "%x \r\n", xAxis);
    chThdSleepMilliseconds(300);   

  }
}

static void cmd_threads(BaseSequentialStream *chp, int argc, char *argv[]) {
  static const char *states[] = {THD_STATE_NAMES};
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: threads\r\n");
    return;
  }
  chprintf(chp, "    Addr    Stack  Pri Refs      State time\r\n");
  tp = chRegFirstThread();
  do {
    chprintf(chp, "%.8lx %.8lx %4lu %4lu %9s %lu\r\n",
            (uint32_t)tp, (uint32_t)tp->p_ctx.r13,
            (uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
            states[tp->p_state], (uint32_t)tp->p_time);
    tp = chRegNextThread(tp);
  } while (tp != NULL);
}

static void cmd_test(BaseSequentialStream *chp, int argc, char *argv[]) {
  Thread *tp;

  (void)argv;
  if (argc > 0) {
    chprintf(chp, "Usage: test\r\n");
    return;
  }
  tp = chThdCreateFromHeap(NULL, TEST_WA_SIZE, chThdGetPriority(),
                           TestThread, chp);
  if (tp == NULL) {
    chprintf(chp, "Out of memory\r\n");
    return;
  }
  chThdWait(tp);
}

static const ShellCommand commands[] = {
  {"mem", cmd_mem},
  {"acc", cmd_acc},
  {"threads", cmd_threads},
  {"test", cmd_test},
  {NULL, NULL}
};

static const ShellConfig shell_cfg1 = {
  (BaseSequentialStream *)&SDU1,
  commands
};

/*===========================================================================*/
/* Accelerometer related.                                                    */
/*===========================================================================*/

/*
 * PWM configuration structure.
 * Cyclic callback enabled, channels 1 and 4 enabled without callbacks,
 * the active state is a logic one.
 */
static const PWMConfig pwmcfg = {
  100000,                                   /* 100kHz PWM clock frequency.  */
  128,                                      /* PWM period is 128 cycles.    */
  NULL,
  {
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL},
   {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  /* HW dependent part.*/
  0,
  0
};

/*
 * SPI1 configuration structure.
 * Speed 5.25MHz, CPHA=1, CPOL=1, 8bits frames, MSb transmitted first.
 * The slave select line is the pin GPIOE_CS_SPI on the port GPIOE.
 */
static const SPIConfig spi1cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOE,
  GPIOE_CS_SPI,
  SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_CPOL | SPI_CR1_CPHA
};

/*
 * SPI2 configuration structure.
 * Speed 21MHz, CPHA=0, CPOL=0, 8bits frames, MSb transmitted first.
 * The slave select line is the pin 12 on the port GPIOA.
 */
static const SPIConfig spi2cfg = {
  NULL,
  /* HW dependent part.*/
  GPIOB,
  12,
  0
};

/*
 * This is a periodic thread that reads accelerometer and outputs
 * result to SPI2 and PWM.
 */
static WORKING_AREA(waThread1, 128);
static msg_t Thread1(void *arg) {
  static int8_t xbuf[4], ybuf[4];   /* Last accelerometer data.*/
  static int8_t xAxisL, xAxisH, yAxisL, yAxisH, zAxisL, zAxisH; 
  systime_t time;                   /* Next deadline.*/

  (void)arg;
  chRegSetThreadName("reader");

  /* LIS3DSH initialization.*/
  lis3dshWriteRegister(&SPID1, LIS3DSH_CTRL_REG3, 0x88);
  lis3dshWriteRegister(&SPID1, LIS3DSH_CTRL_REG4, 0x67);
  lis3dshWriteRegister(&SPID1, LIS3DSH_CTRL_REG5, 0xE0);
  lis3dshWriteRegister(&SPID1, LIS3DSH_CTRL_REG6, 0x52);
  lis3dshWriteRegister(&SPID1, LIS3DSH_FIFO_CTRL, 0x40);

  /* Reader thread loop.*/
  time = chTimeNow();
  while (TRUE) {
    int32_t x, y;
    unsigned i;

    /* Keeping an history of the latest four accelerometer readings.*/
    for (i = 3; i > 0; i--) {
      xbuf[i] = xbuf[i - 1];
      ybuf[i] = ybuf[i - 1];
    }

    /* Reading MEMS accelerometer X, Y and Z registers.*/
    /* for (i = 10; i > 0; i++) {
      xAxisL = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_X_L);
      xAxisH = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_X_H);
      yAxisL = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Y_L);
      yAxisH = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Y_H);
      zAxisL = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Z_L);
      zAxisH = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Z_H);
      }*/
    /* 
    xbuf[0] = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_X_H);
    ybuf[0] = (int8_t)lis3dshReadRegister(&SPID1, LIS3DSH_OUT_Y_H);
    */
    /*if INT1 = 1 eller if FIFO_SRC |! OVRN_FIFO == 1 read FIFO adr 1*/

    /* Transmitting accelerometer the data over SPI2.*/
    spiSelect(&SPID2);
    spiSend(&SPID2, 4, xbuf);
    spiSend(&SPID2, 4, ybuf);
    spiUnselect(&SPID2);

    /* Calculating average of the latest four accelerometer readings.*/
    x = ((int32_t)xbuf[0] + (int32_t)xbuf[1] +
         (int32_t)xbuf[2] + (int32_t)xbuf[3]) / 4;
    y = ((int32_t)ybuf[0] + (int32_t)ybuf[1] +
         (int32_t)ybuf[2] + (int32_t)ybuf[3]) / 4;

    /* Reprogramming the four PWM channels using the accelerometer data.*/
    if (y < 0) {
      pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)-y);
      pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)0);
    }
    else {
      pwmEnableChannel(&PWMD4, 1, (pwmcnt_t)y);
      pwmEnableChannel(&PWMD4, 3, (pwmcnt_t)0);
    }
    if (x < 0) {
      pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)-x);
      pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)0);
    }
    else {
      pwmEnableChannel(&PWMD4, 2, (pwmcnt_t)x);
      pwmEnableChannel(&PWMD4, 0, (pwmcnt_t)0);
    }

    /* Waiting until the next 250 milliseconds time interval.*/
    chThdSleepUntil(time += MS2ST(100));
  }
}

/*===========================================================================*/
/* Initialization and main thread.                                           */
/*===========================================================================*/

/*
 * Application entry point.
 */
int main(void) {
  Thread *shelltp = NULL;

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  /*
   * Shell manager initialization.
   */
  shellInit();

  /*
   * Initializes a serial-over-USB CDC driver.
   */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /*
   * Activates the USB driver and then the USB bus pull-up on D+.
   * Note, a delay is inserted in order to not have to disconnect the cable
   * after a reset.
   */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1000);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /*
   * Activates the serial driver 2 using the driver default configuration.
   * PA2(TX) and PA3(RX) are routed to USART2.
   */
  sdStart(&SD2, NULL);
  palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(7));
  palSetPadMode(GPIOA, 3, PAL_MODE_ALTERNATE(7));

  /*
   * Initializes the SPI driver 1 in order to access the MEMS. The signals
   * are already initialized in the board file.
   */
  spiStart(&SPID1, &spi1cfg);

  /*
   * Initializes the SPI driver 2. The SPI2 signals are routed as follow:
   * PB12 - NSS.
   * PB13 - SCK.
   * PB14 - MISO.
   * PB15 - MOSI.
   */
  spiStart(&SPID2, &spi2cfg);
  palSetPad(GPIOB, 12);
  palSetPadMode(GPIOB, 12, PAL_MODE_OUTPUT_PUSHPULL |
                           PAL_STM32_OSPEED_HIGHEST);           /* NSS.     */
  palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(5) |
                           PAL_STM32_OSPEED_HIGHEST);           /* SCK.     */
  palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(5));              /* MISO.    */
  palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(5) |
                           PAL_STM32_OSPEED_HIGHEST);           /* MOSI.    */

  /*
   * Initializes the PWM driver 4, routes the TIM4 outputs to the board LEDs.
   */
  pwmStart(&PWMD4, &pwmcfg);
  palSetPadMode(GPIOD, GPIOD_LED4, PAL_MODE_ALTERNATE(2));      /* Green.   */
  palSetPadMode(GPIOD, GPIOD_LED3, PAL_MODE_ALTERNATE(2));      /* Orange.  */
  palSetPadMode(GPIOD, GPIOD_LED5, PAL_MODE_ALTERNATE(2));      /* Red.     */
  palSetPadMode(GPIOD, GPIOD_LED6, PAL_MODE_ALTERNATE(2));      /* Blue.    */

  /*
   * Creates the example thread.
   */
  chThdCreateStatic(waThread1, sizeof(waThread1),
                    NORMALPRIO + 10, Thread1, NULL);

  /*
   * Normal main() thread activity, in this demo it just performs
   * a shell respawn upon its termination.
   */
  while (TRUE) {
    if (!shelltp) {
      if (SDU1.config->usbp->state == USB_ACTIVE) {
        /* Spawns a new shell.*/
        shelltp = shellCreate(&shell_cfg1, SHELL_WA_SIZE, NORMALPRIO);
      }
    }
    else {
      /* If the previous shell exited.*/
      if (chThdTerminated(shelltp)) {
        /* Recovers memory of the previous shell.*/
        chThdRelease(shelltp);
        shelltp = NULL;
      }
    }
    chThdSleepMilliseconds(500);
  }
}

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

/**
 * @file    lis3dsh.c
 * @brief   LIS3DSH MEMS interface module through SPI code.
 *
 * @addtogroup lis3dsh
 * @{
 */

#include "ch.h"
#include "hal.h"
#include "lis3dsh.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

static uint8_t txbuf[2];
static uint8_t rxbuf[2];

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Reads a register value.
 * @pre     The SPI interface must be initialized and the driver started.
 *
 * @param[in] spip      pointer to the SPI initerface
 * @param[in] reg       register number
 * @return              The register value.
 */
uint8_t lis3dshReadRegister(SPIDriver *spip, uint8_t reg) {

  spiSelect(spip);
  txbuf[0] = 0x80 | reg;
  txbuf[1] = 0xff;
  spiExchange(spip, 2, txbuf, rxbuf);
  spiUnselect(spip);
  return rxbuf[1];
}

/**
 * @brief   Writes a value into a register.
 * @pre     The SPI interface must be initialized and the driver must be started.
 *
 * @param[in] spip      pointer to the SPI interface
 * @param[in] reg       register number
 * @param[in] value     the value to be written
 */
void lis3dshWriteRegister(SPIDriver *spip, uint8_t reg, uint8_t value) {

  switch (reg) {
  default:
    /* Reserved register must not be written, according to the datasheet
       this could permanently damage the device.*/
    chDbgAssert(FALSE, "lis3dshWriteRegister(), #1", "reserved register");
  case LIS3DSH_INFO1:
  case LIS3DSH_INFO2:
  case LIS3DSH_WHO_AM_I:
  case LIS3DSH_STATUS:
  case LIS3DSH_OUT_T:
  case LIS3DSH_STAT:
  case LIS3DSH_OUT_X_L:
  case LIS3DSH_OUT_X_H:
  case LIS3DSH_OUT_Y_L:
  case LIS3DSH_OUT_Y_H:
  case LIS3DSH_OUT_Z_L:
  case LIS3DSH_OUT_Z_H:
  case LIS3DSH_FIFO_SRC:
  case LIS3DSH_PR1:
  case LIS3DSH_TC1_L:
  case LIS3DSH_TC1_H:
  case LIS3DSH_OUTS1:
  case LIS3DSH_PEAK1:
  case LIS3DSH_PR2:
  case LIS3DSH_TC2_L:
  case LIS3DSH_TC2_H:  
  case LIS3DSH_OUTS2:
  case LIS3DSH_PEAK2:
    /* Read only registers cannot be written, the command is ignored.*/
    return;

  case LIS3DSH_CTRL_REG3: 
  case LIS3DSH_CTRL_REG4: 
  case LIS3DSH_CTRL_REG5: 
  case LIS3DSH_CTRL_REG6: 
  case LIS3DSH_OFF_X: 
  case LIS3DSH_OFF_Y: 
  case LIS3DSH_OFF_Z: 
  case LIS3DSH_CS_X: 
  case LIS3DSH_CS_Y: 
  case LIS3DSH_CS_Z: 
  case LIS3DSH_LC_L: 
  case LIS3DSH_LC_H: 
  case LIS3DSH_VFC_1: 
  case LIS3DSH_VFC_2: 
  case LIS3DSH_VFC_3: 
  case LIS3DSH_VFC_4: 
  case LIS3DSH_THRS3: 
  case LIS3DSH_FIFO_CTRL: 
  case LIS3DSH_CTRL_REG1: 
  case LIS3DSH_ST1_1: 
  case LIS3DSH_ST1_2: 
  case LIS3DSH_ST1_3: 
  case LIS3DSH_ST1_4: 
  case LIS3DSH_ST1_5: 
  case LIS3DSH_ST1_6: 
  case LIS3DSH_ST1_7: 
  case LIS3DSH_ST1_8: 
  case LIS3DSH_ST1_9: 
  case LIS3DSH_ST1_10: 
  case LIS3DSH_ST1_11: 
  case LIS3DSH_ST1_12: 
  case LIS3DSH_ST1_13: 
  case LIS3DSH_ST1_14: 
  case LIS3DSH_ST1_15: 
  case LIS3DSH_ST1_16: 
  case LIS3DSH_TIM4_1: 
  case LIS3DSH_TIM3_1: 
  case LIS3DSH_TIM2_1_L: 
  case LIS3DSH_TIM2_1_H: 
  case LIS3DSH_TIM1_1_L: 
  case LIS3DSH_TIM1_1_H: 
  case LIS3DSH_THRS2_1: 
  case LIS3DSH_THRS1_1: 
  case LIS3DSH_MASK1_B: 
  case LIS3DSH_MASK1_A: 
  case LIS3DSH_SETT1: 
  case LIS3DSH_CTRL_REG2: 
  case LIS3DSH_ST2_1: 
  case LIS3DSH_ST2_2: 
  case LIS3DSH_ST2_3: 
  case LIS3DSH_ST2_4: 
  case LIS3DSH_ST2_5: 
  case LIS3DSH_ST2_6: 
  case LIS3DSH_ST2_7: 
  case LIS3DSH_ST2_8: 
  case LIS3DSH_ST2_9: 
  case LIS3DSH_ST2_10: 
  case LIS3DSH_ST2_11: 
  case LIS3DSH_ST2_12: 
  case LIS3DSH_ST2_13: 
  case LIS3DSH_ST2_14: 
  case LIS3DSH_ST2_15: 
  case LIS3DSH_ST2_16: 
  case LIS3DSH_TIM4_2: 
  case LIS3DSH_TIM3_2: 
  case LIS3DSH_TIM2_2_L: 
  case LIS3DSH_TIM2_2_H: 
  case LIS3DSH_TIM1_2_L: 
  case LIS3DSH_TIM1_2_H: 
  case LIS3DSH_THRS2_2: 
  case LIS3DSH_THRS1_2: 
  case LIS3DSH_MASK2_B: 
  case LIS3DSH_MASK2_A: 
  case LIS3DSH_SETT2: 
  case LIS3DSH_DES2: 

    spiSelect(spip);
    txbuf[0] = reg;
    txbuf[1] = value;
    spiSend(spip, 2, txbuf);
    spiUnselect(spip);
  }
}

/** @} */

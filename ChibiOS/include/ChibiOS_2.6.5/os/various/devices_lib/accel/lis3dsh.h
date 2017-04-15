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
 * @file    lis3dsh.h
 * @brief   LIS3DSH MEMS interface module through SPI header.
 *
 * @addtogroup lis3dsh
 * @{
 */

#ifndef _LIS3DSH_H_
#define _LIS3DSH_H_

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    LIS3DSH register names
 * @{
 */
#define LIS3DSH_INFO1               0x0D
#define LIS3DSH_INFO2               0x0E 
#define LIS3DSH_WHO_AM_I            0x0F
#define LIS3DSH_CTRL_REG3           0x23
#define LIS3DSH_CTRL_REG4           0x20
#define LIS3DSH_CTRL_REG5           0x24
#define LIS3DSH_CTRL_REG6           0x25
#define LIS3DSH_STATUS              0x27
#define LIS3DSH_OUT_T               0x0C
#define LIS3DSH_OFF_X               0x10
#define LIS3DSH_OFF_Y               0x11
#define LIS3DSH_OFF_Z               0x12
#define LIS3DSH_CS_X                0x13
#define LIS3DSH_CS_Y                0X14
#define LIS3DSH_CS_Z                0x15
#define LIS3DSH_LC_L                0x16
#define LIS3DSH_LC_H                0x17
#define LIS3DSH_STAT                0x18
#define LIS3DSH_VFC_1               0x1B
#define LIS3DSH_VFC_2               0x1C
#define LIS3DSH_VFC_3               0x1D
#define LIS3DSH_VFC_4               0X1E
#define LIS3DSH_THRS3               0x1F
#define LIS3DSH_OUT_X_L             0x28
#define LIS3DSH_OUT_X_H             0x29
#define LIS3DSH_OUT_Y_L             0x2A
#define LIS3DSH_OUT_Y_H             0x2B
#define LIS3DSH_OUT_Z_L             0x2C
#define LIS3DSH_OUT_Z_H             0x2D
#define LIS3DSH_FIFO_CTRL           0x2E
#define LIS3DSH_FIFO_SRC            0x2F
#define LIS3DSH_CTRL_REG1           0x21
#define LIS3DSH_ST1_1               0x40
#define LIS3DSH_ST1_2               0x41
#define LIS3DSH_ST1_3               0x42
#define LIS3DSH_ST1_4               0x43
#define LIS3DSH_ST1_5               0x44
#define LIS3DSH_ST1_6               0x45
#define LIS3DSH_ST1_7               0x46
#define LIS3DSH_ST1_8               0x47
#define LIS3DSH_ST1_9               0x48
#define LIS3DSH_ST1_10              0x49
#define LIS3DSH_ST1_11              0x4A
#define LIS3DSH_ST1_12              0x4B
#define LIS3DSH_ST1_13              0x4C
#define LIS3DSH_ST1_14              0x4D
#define LIS3DSH_ST1_15              0x4E
#define LIS3DSH_ST1_16              0x4F
#define LIS3DSH_TIM4_1              0x50
#define LIS3DSH_TIM3_1              0x51
#define LIS3DSH_TIM2_1_L            0x52
#define LIS3DSH_TIM2_1_H            0x53
#define LIS3DSH_TIM1_1_L            0x54
#define LIS3DSH_TIM1_1_H            0x55
#define LIS3DSH_THRS2_1             0x56
#define LIS3DSH_THRS1_1             0x57
#define LIS3DSH_MASK1_B             0x59
#define LIS3DSH_MASK1_A             0x5A
#define LIS3DSH_SETT1               0x5B
#define LIS3DSH_PR1                 0x5C
#define LIS3DSH_TC1_L               0x5D
#define LIS3DSH_TC1_H               0x5E
#define LIS3DSH_OUTS1               0x5F
#define LIS3DSH_PEAK1               0x19
#define LIS3DSH_CTRL_REG2           0x22
#define LIS3DSH_ST2_1               0x60
#define LIS3DSH_ST2_2               0x61
#define LIS3DSH_ST2_3               0x62
#define LIS3DSH_ST2_4               0x63
#define LIS3DSH_ST2_5               0x64
#define LIS3DSH_ST2_6               0x65
#define LIS3DSH_ST2_7               0x66
#define LIS3DSH_ST2_8               0x67
#define LIS3DSH_ST2_9               0x68
#define LIS3DSH_ST2_10              0x69
#define LIS3DSH_ST2_11              0x6A
#define LIS3DSH_ST2_12              0x6B
#define LIS3DSH_ST2_13              0x6C
#define LIS3DSH_ST2_14              0x6D
#define LIS3DSH_ST2_15              0x6E
#define LIS3DSH_ST2_16              0x6F
#define LIS3DSH_TIM4_2              0x70
#define LIS3DSH_TIM3_2              0x71
#define LIS3DSH_TIM2_2_L            0x72
#define LIS3DSH_TIM2_2_H            0x73
#define LIS3DSH_TIM1_2_L            0x74
#define LIS3DSH_TIM1_2_H            0x75
#define LIS3DSH_THRS2_2             0x76
#define LIS3DSH_THRS1_2             0x77
#define LIS3DSH_MASK2_B             0x79
#define LIS3DSH_MASK2_A             0x7A
#define LIS3DSH_SETT2               0x7B
#define LIS3DSH_PR2                 0x7C
#define LIS3DSH_TC2_L               0x7D
#define LIS3DSH_TC2_H               0x7E
#define LIS3DSH_OUTS2               0x7F
#define LIS3DSH_PEAK2               0x1A
#define LIS3DSH_DES2                0x78
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#ifdef __cplusplus
extern "C" {
#endif
  uint8_t lis3dshReadRegister(SPIDriver *spip, uint8_t reg);
  void lis3dshWriteRegister(SPIDriver *spip, uint8_t reg, uint8_t value);
#ifdef __cplusplus
}
#endif

#endif /* _LIS3DSH_H_ */

/** @} */

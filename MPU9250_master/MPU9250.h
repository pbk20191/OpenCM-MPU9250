// I2Cdev library collection - MPU9250 I2C device class
// Based on InvenSense MPU-9250 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
// 10/3/2011 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     ... - ongoing debug release

// NOTE: THIS IS ONLY A PARIAL RELEASE. THIS DEVICE CLASS IS CURRENTLY UNDERGOING ACTIVE
// DEVELOPMENT AND IS STILL MISSING SOME IMPORTANT FEATURES. PLEASE KEEP THIS IN MIND IF
// YOU DECIDE TO USE THIS PARTICULAR CODE FOR ANYTHING.

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _MPU9250_H_
#define _MPU9250_H_

#define MPU9250_INCLUDE_DMP_MOTIONAPPS41

#include "I2Cdev.h"

  #ifdef  MPU9250_INCLUDE_DMP_MOTIONAPPS41
    #include "helper_3dmath.h"
    #include <string.h>
   // #define DEBUGmod
      #ifdef DEBUGmod
        #define DEBUG_PRINT(x) SerialUSB.print(x)
        #define DEBUG_PRINTF(x, y) SerialUSB.print(x, y)
        #define DEBUG_PRINTLN(x) SerialUSB.println(x)
        #define DEBUG_PRINTLNF(x, y) SerialUSB.println(x, y)
      #else
        #define DEBUG_PRINT(x)
        #define DEBUG_PRINTF(x, y)
        #define DEBUG_PRINTLN(x)
        #define DEBUG_PRINTLNF(x, y)
      #endif

    #define MPU9250_DMP_CODE_SIZE       1962    // dmpMemory[]
    #define MPU9250_DMP_CONFIG_SIZE     232     // dmpConfig[]
    #define MPU9250_DMP_UPDATES_SIZE    140     // dmpUpdates[]
  #endif

//Magnetometer Registers
#define MPU9150_RA_MAG_ADDRESS		0x0C
#define MPU9150_RA_MAG_XOUT_L		0x03
#define MPU9150_RA_MAG_XOUT_H		0x04
#define MPU9150_RA_MAG_YOUT_L		0x05
#define MPU9150_RA_MAG_YOUT_H		0x06
#define MPU9150_RA_MAG_ZOUT_L		0x07
#define MPU9150_RA_MAG_ZOUT_H		0x08

#define MPU9250_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU9250_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU9250_DEFAULT_ADDRESS     MPU9250_ADDRESS_AD0_LOW

#define MPU9250_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU9250_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU9250_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU9250_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU9250_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU9250_RA_XA_OFFS_L_TC     0x07
#define MPU9250_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU9250_RA_YA_OFFS_L_TC     0x09
#define MPU9250_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU9250_RA_ZA_OFFS_L_TC     0x0B
#define MPU9250_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU9250_RA_XG_OFFS_USRL     0x14
#define MPU9250_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU9250_RA_YG_OFFS_USRL     0x16
#define MPU9250_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU9250_RA_ZG_OFFS_USRL     0x18
#define MPU9250_RA_SMPLRT_DIV       0x19
#define MPU9250_RA_CONFIG           0x1A
#define MPU9250_RA_GYRO_CONFIG      0x1B
#define MPU9250_RA_ACCEL_CONFIG     0x1C
#define MPU9250_RA_FF_THR           0x1D
#define MPU9250_RA_FF_DUR           0x1E
#define MPU9250_RA_MOT_THR          0x1F
#define MPU9250_RA_MOT_DUR          0x20
#define MPU9250_RA_ZRMOT_THR        0x21
#define MPU9250_RA_ZRMOT_DUR        0x22
#define MPU9250_RA_FIFO_EN          0x23
#define MPU9250_RA_I2C_MST_CTRL     0x24
#define MPU9250_RA_I2C_SLV0_ADDR    0x25
#define MPU9250_RA_I2C_SLV0_REG     0x26
#define MPU9250_RA_I2C_SLV0_CTRL    0x27
#define MPU9250_RA_I2C_SLV1_ADDR    0x28
#define MPU9250_RA_I2C_SLV1_REG     0x29
#define MPU9250_RA_I2C_SLV1_CTRL    0x2A
#define MPU9250_RA_I2C_SLV2_ADDR    0x2B
#define MPU9250_RA_I2C_SLV2_REG     0x2C
#define MPU9250_RA_I2C_SLV2_CTRL    0x2D
#define MPU9250_RA_I2C_SLV3_ADDR    0x2E
#define MPU9250_RA_I2C_SLV3_REG     0x2F
#define MPU9250_RA_I2C_SLV3_CTRL    0x30
#define MPU9250_RA_I2C_SLV4_ADDR    0x31
#define MPU9250_RA_I2C_SLV4_REG     0x32
#define MPU9250_RA_I2C_SLV4_DO      0x33
#define MPU9250_RA_I2C_SLV4_CTRL    0x34
#define MPU9250_RA_I2C_SLV4_DI      0x35
#define MPU9250_RA_I2C_MST_STATUS   0x36
#define MPU9250_RA_INT_PIN_CFG      0x37
#define MPU9250_RA_INT_ENABLE       0x38
#define MPU9250_RA_DMP_INT_STATUS   0x39
#define MPU9250_RA_INT_STATUS       0x3A
#define MPU9250_RA_ACCEL_XOUT_H     0x3B
#define MPU9250_RA_ACCEL_XOUT_L     0x3C
#define MPU9250_RA_ACCEL_YOUT_H     0x3D
#define MPU9250_RA_ACCEL_YOUT_L     0x3E
#define MPU9250_RA_ACCEL_ZOUT_H     0x3F
#define MPU9250_RA_ACCEL_ZOUT_L     0x40
#define MPU9250_RA_TEMP_OUT_H       0x41
#define MPU9250_RA_TEMP_OUT_L       0x42
#define MPU9250_RA_GYRO_XOUT_H      0x43
#define MPU9250_RA_GYRO_XOUT_L      0x44
#define MPU9250_RA_GYRO_YOUT_H      0x45
#define MPU9250_RA_GYRO_YOUT_L      0x46
#define MPU9250_RA_GYRO_ZOUT_H      0x47
#define MPU9250_RA_GYRO_ZOUT_L      0x48
#define MPU9250_RA_EXT_SENS_DATA_00 0x49
#define MPU9250_RA_EXT_SENS_DATA_01 0x4A
#define MPU9250_RA_EXT_SENS_DATA_02 0x4B
#define MPU9250_RA_EXT_SENS_DATA_03 0x4C
#define MPU9250_RA_EXT_SENS_DATA_04 0x4D
#define MPU9250_RA_EXT_SENS_DATA_05 0x4E
#define MPU9250_RA_EXT_SENS_DATA_06 0x4F
#define MPU9250_RA_EXT_SENS_DATA_07 0x50
#define MPU9250_RA_EXT_SENS_DATA_08 0x51
#define MPU9250_RA_EXT_SENS_DATA_09 0x52
#define MPU9250_RA_EXT_SENS_DATA_10 0x53
#define MPU9250_RA_EXT_SENS_DATA_11 0x54
#define MPU9250_RA_EXT_SENS_DATA_12 0x55
#define MPU9250_RA_EXT_SENS_DATA_13 0x56
#define MPU9250_RA_EXT_SENS_DATA_14 0x57
#define MPU9250_RA_EXT_SENS_DATA_15 0x58
#define MPU9250_RA_EXT_SENS_DATA_16 0x59
#define MPU9250_RA_EXT_SENS_DATA_17 0x5A
#define MPU9250_RA_EXT_SENS_DATA_18 0x5B
#define MPU9250_RA_EXT_SENS_DATA_19 0x5C
#define MPU9250_RA_EXT_SENS_DATA_20 0x5D
#define MPU9250_RA_EXT_SENS_DATA_21 0x5E
#define MPU9250_RA_EXT_SENS_DATA_22 0x5F
#define MPU9250_RA_EXT_SENS_DATA_23 0x60
#define MPU9250_RA_MOT_DETECT_STATUS    0x61
#define MPU9250_RA_I2C_SLV0_DO      0x63
#define MPU9250_RA_I2C_SLV1_DO      0x64
#define MPU9250_RA_I2C_SLV2_DO      0x65
#define MPU9250_RA_I2C_SLV3_DO      0x66
#define MPU9250_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU9250_RA_SIGNAL_PATH_RESET    0x68
#define MPU9250_RA_MOT_DETECT_CTRL      0x69
#define MPU9250_RA_USER_CTRL        0x6A
#define MPU9250_RA_PWR_MGMT_1       0x6B
#define MPU9250_RA_PWR_MGMT_2       0x6C
#define MPU9250_RA_BANK_SEL         0x6D
#define MPU9250_RA_MEM_START_ADDR   0x6E
#define MPU9250_RA_MEM_R_W          0x6F
#define MPU9250_RA_DMP_CFG_1        0x70
#define MPU9250_RA_DMP_CFG_2        0x71
#define MPU9250_RA_FIFO_COUNTH      0x72
#define MPU9250_RA_FIFO_COUNTL      0x73
#define MPU9250_RA_FIFO_R_W         0x74
#define MPU9250_RA_WHO_AM_I         0x75

#define MPU9250_TC_PWR_MODE_BIT     7
#define MPU9250_TC_OFFSET_BIT       6
#define MPU9250_TC_OFFSET_LENGTH    6
#define MPU9250_TC_OTP_BNK_VLD_BIT  0

#define MPU9250_VDDIO_LEVEL_VLOGIC  0
#define MPU9250_VDDIO_LEVEL_VDD     1

#define MPU9250_CFG_EXT_SYNC_SET_BIT    5
#define MPU9250_CFG_EXT_SYNC_SET_LENGTH 3
#define MPU9250_CFG_DLPF_CFG_BIT    2
#define MPU9250_CFG_DLPF_CFG_LENGTH 3

#define MPU9250_EXT_SYNC_DISABLED       0x0
#define MPU9250_EXT_SYNC_TEMP_OUT_L     0x1
#define MPU9250_EXT_SYNC_GYRO_XOUT_L    0x2
#define MPU9250_EXT_SYNC_GYRO_YOUT_L    0x3
#define MPU9250_EXT_SYNC_GYRO_ZOUT_L    0x4
#define MPU9250_EXT_SYNC_ACCEL_XOUT_L   0x5
#define MPU9250_EXT_SYNC_ACCEL_YOUT_L   0x6
#define MPU9250_EXT_SYNC_ACCEL_ZOUT_L   0x7

#define MPU9250_DLPF_BW_256         0x00
#define MPU9250_DLPF_BW_188         0x01
#define MPU9250_DLPF_BW_98          0x02
#define MPU9250_DLPF_BW_42          0x03
#define MPU9250_DLPF_BW_20          0x04
#define MPU9250_DLPF_BW_10          0x05
#define MPU9250_DLPF_BW_5           0x06

#define MPU9250_GCONFIG_FS_SEL_BIT      4
#define MPU9250_GCONFIG_FS_SEL_LENGTH   2

#define MPU9250_GYRO_FS_250         0x00
#define MPU9250_GYRO_FS_500         0x01
#define MPU9250_GYRO_FS_1000        0x02
#define MPU9250_GYRO_FS_2000        0x03

#define MPU9250_ACONFIG_XA_ST_BIT           7
#define MPU9250_ACONFIG_YA_ST_BIT           6
#define MPU9250_ACONFIG_ZA_ST_BIT           5
#define MPU9250_ACONFIG_AFS_SEL_BIT         4
#define MPU9250_ACONFIG_AFS_SEL_LENGTH      2
#define MPU9250_ACONFIG_ACCEL_HPF_BIT       2
#define MPU9250_ACONFIG_ACCEL_HPF_LENGTH    3

#define MPU9250_ACCEL_FS_2          0x00
#define MPU9250_ACCEL_FS_4          0x01
#define MPU9250_ACCEL_FS_8          0x02
#define MPU9250_ACCEL_FS_16         0x03

#define MPU9250_DHPF_RESET          0x00
#define MPU9250_DHPF_5              0x01
#define MPU9250_DHPF_2P5            0x02
#define MPU9250_DHPF_1P25           0x03
#define MPU9250_DHPF_0P63           0x04
#define MPU9250_DHPF_HOLD           0x07

#define MPU9250_TEMP_FIFO_EN_BIT    7
#define MPU9250_XG_FIFO_EN_BIT      6
#define MPU9250_YG_FIFO_EN_BIT      5
#define MPU9250_ZG_FIFO_EN_BIT      4
#define MPU9250_ACCEL_FIFO_EN_BIT   3
#define MPU9250_SLV2_FIFO_EN_BIT    2
#define MPU9250_SLV1_FIFO_EN_BIT    1
#define MPU9250_SLV0_FIFO_EN_BIT    0

#define MPU9250_MULT_MST_EN_BIT     7
#define MPU9250_WAIT_FOR_ES_BIT     6
#define MPU9250_SLV_3_FIFO_EN_BIT   5
#define MPU9250_I2C_MST_P_NSR_BIT   4
#define MPU9250_I2C_MST_CLK_BIT     3
#define MPU9250_I2C_MST_CLK_LENGTH  4

#define MPU9250_CLOCK_DIV_348       0x0
#define MPU9250_CLOCK_DIV_333       0x1
#define MPU9250_CLOCK_DIV_320       0x2
#define MPU9250_CLOCK_DIV_308       0x3
#define MPU9250_CLOCK_DIV_296       0x4
#define MPU9250_CLOCK_DIV_286       0x5
#define MPU9250_CLOCK_DIV_276       0x6
#define MPU9250_CLOCK_DIV_267       0x7
#define MPU9250_CLOCK_DIV_258       0x8
#define MPU9250_CLOCK_DIV_500       0x9
#define MPU9250_CLOCK_DIV_471       0xA
#define MPU9250_CLOCK_DIV_444       0xB
#define MPU9250_CLOCK_DIV_421       0xC
#define MPU9250_CLOCK_DIV_400       0xD
#define MPU9250_CLOCK_DIV_381       0xE
#define MPU9250_CLOCK_DIV_364       0xF

#define MPU9250_I2C_SLV_RW_BIT      7
#define MPU9250_I2C_SLV_ADDR_BIT    6
#define MPU9250_I2C_SLV_ADDR_LENGTH 7
#define MPU9250_I2C_SLV_EN_BIT      7
#define MPU9250_I2C_SLV_BYTE_SW_BIT 6
#define MPU9250_I2C_SLV_REG_DIS_BIT 5
#define MPU9250_I2C_SLV_GRP_BIT     4
#define MPU9250_I2C_SLV_LEN_BIT     3
#define MPU9250_I2C_SLV_LEN_LENGTH  4

#define MPU9250_I2C_SLV4_RW_BIT         7
#define MPU9250_I2C_SLV4_ADDR_BIT       6
#define MPU9250_I2C_SLV4_ADDR_LENGTH    7
#define MPU9250_I2C_SLV4_EN_BIT         7
#define MPU9250_I2C_SLV4_INT_EN_BIT     6
#define MPU9250_I2C_SLV4_REG_DIS_BIT    5
#define MPU9250_I2C_SLV4_MST_DLY_BIT    4
#define MPU9250_I2C_SLV4_MST_DLY_LENGTH 5

#define MPU9250_MST_PASS_THROUGH_BIT    7
#define MPU9250_MST_I2C_SLV4_DONE_BIT   6
#define MPU9250_MST_I2C_LOST_ARB_BIT    5
#define MPU9250_MST_I2C_SLV4_NACK_BIT   4
#define MPU9250_MST_I2C_SLV3_NACK_BIT   3
#define MPU9250_MST_I2C_SLV2_NACK_BIT   2
#define MPU9250_MST_I2C_SLV1_NACK_BIT   1
#define MPU9250_MST_I2C_SLV0_NACK_BIT   0

#define MPU9250_INTCFG_INT_LEVEL_BIT        7
#define MPU9250_INTCFG_INT_OPEN_BIT         6
#define MPU9250_INTCFG_LATCH_INT_EN_BIT     5
#define MPU9250_INTCFG_INT_RD_CLEAR_BIT     4
#define MPU9250_INTCFG_FSYNC_INT_LEVEL_BIT  3
#define MPU9250_INTCFG_FSYNC_INT_EN_BIT     2
#define MPU9250_INTCFG_I2C_BYPASS_EN_BIT    1
#define MPU9250_INTCFG_CLKOUT_EN_BIT        0

#define MPU9250_INTMODE_ACTIVEHIGH  0x00
#define MPU9250_INTMODE_ACTIVELOW   0x01

#define MPU9250_INTDRV_PUSHPULL     0x00
#define MPU9250_INTDRV_OPENDRAIN    0x01

#define MPU9250_INTLATCH_50USPULSE  0x00
#define MPU9250_INTLATCH_WAITCLEAR  0x01

#define MPU9250_INTCLEAR_STATUSREAD 0x00
#define MPU9250_INTCLEAR_ANYREAD    0x01

#define MPU9250_INTERRUPT_FF_BIT            7
#define MPU9250_INTERRUPT_MOT_BIT           6
#define MPU9250_INTERRUPT_ZMOT_BIT          5
#define MPU9250_INTERRUPT_FIFO_OFLOW_BIT    4
#define MPU9250_INTERRUPT_I2C_MST_INT_BIT   3
#define MPU9250_INTERRUPT_PLL_RDY_INT_BIT   2
#define MPU9250_INTERRUPT_DMP_INT_BIT       1
#define MPU9250_INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obivous
#define MPU9250_DMPINT_5_BIT            5
#define MPU9250_DMPINT_4_BIT            4
#define MPU9250_DMPINT_3_BIT            3
#define MPU9250_DMPINT_2_BIT            2
#define MPU9250_DMPINT_1_BIT            1
#define MPU9250_DMPINT_0_BIT            0

#define MPU9250_MOTION_MOT_XNEG_BIT     7
#define MPU9250_MOTION_MOT_XPOS_BIT     6
#define MPU9250_MOTION_MOT_YNEG_BIT     5
#define MPU9250_MOTION_MOT_YPOS_BIT     4
#define MPU9250_MOTION_MOT_ZNEG_BIT     3
#define MPU9250_MOTION_MOT_ZPOS_BIT     2
#define MPU9250_MOTION_MOT_ZRMOT_BIT    0

#define MPU9250_DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define MPU9250_DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define MPU9250_DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define MPU9250_DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define MPU9250_DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define MPU9250_DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define MPU9250_PATHRESET_GYRO_RESET_BIT    2
#define MPU9250_PATHRESET_ACCEL_RESET_BIT   1
#define MPU9250_PATHRESET_TEMP_RESET_BIT    0

#define MPU9250_DETECT_ACCEL_ON_DELAY_BIT       5
#define MPU9250_DETECT_ACCEL_ON_DELAY_LENGTH    2
#define MPU9250_DETECT_FF_COUNT_BIT             3
#define MPU9250_DETECT_FF_COUNT_LENGTH          2
#define MPU9250_DETECT_MOT_COUNT_BIT            1
#define MPU9250_DETECT_MOT_COUNT_LENGTH         2

#define MPU9250_DETECT_DECREMENT_RESET  0x0
#define MPU9250_DETECT_DECREMENT_1      0x1
#define MPU9250_DETECT_DECREMENT_2      0x2
#define MPU9250_DETECT_DECREMENT_4      0x3

#define MPU9250_USERCTRL_DMP_EN_BIT             7
#define MPU9250_USERCTRL_FIFO_EN_BIT            6
#define MPU9250_USERCTRL_I2C_MST_EN_BIT         5
#define MPU9250_USERCTRL_I2C_IF_DIS_BIT         4
#define MPU9250_USERCTRL_DMP_RESET_BIT          3
#define MPU9250_USERCTRL_FIFO_RESET_BIT         2
#define MPU9250_USERCTRL_I2C_MST_RESET_BIT      1
#define MPU9250_USERCTRL_SIG_COND_RESET_BIT     0

#define MPU9250_PWR1_DEVICE_RESET_BIT   7
#define MPU9250_PWR1_SLEEP_BIT          6
#define MPU9250_PWR1_CYCLE_BIT          5
#define MPU9250_PWR1_TEMP_DIS_BIT       3
#define MPU9250_PWR1_CLKSEL_BIT         2
#define MPU9250_PWR1_CLKSEL_LENGTH      3

#define MPU9250_CLOCK_INTERNAL          0x00
#define MPU9250_CLOCK_PLL_XGYRO         0x01
#define MPU9250_CLOCK_PLL_YGYRO         0x02
#define MPU9250_CLOCK_PLL_ZGYRO         0x03
#define MPU9250_CLOCK_PLL_EXT32K        0x04
#define MPU9250_CLOCK_PLL_EXT19M        0x05
#define MPU9250_CLOCK_KEEP_RESET        0x07

#define MPU9250_PWR2_LP_WAKE_CTRL_BIT       7
#define MPU9250_PWR2_LP_WAKE_CTRL_LENGTH    2
#define MPU9250_PWR2_STBY_XA_BIT            5
#define MPU9250_PWR2_STBY_YA_BIT            4
#define MPU9250_PWR2_STBY_ZA_BIT            3
#define MPU9250_PWR2_STBY_XG_BIT            2
#define MPU9250_PWR2_STBY_YG_BIT            1
#define MPU9250_PWR2_STBY_ZG_BIT            0

#define MPU9250_WAKE_FREQ_1P25      0x0
#define MPU9250_WAKE_FREQ_2P5       0x1
#define MPU9250_WAKE_FREQ_5         0x2
#define MPU9250_WAKE_FREQ_10        0x3

#define MPU9250_BANKSEL_PRFTCH_EN_BIT       6
#define MPU9250_BANKSEL_CFG_USER_BANK_BIT   5
#define MPU9250_BANKSEL_MEM_SEL_BIT         4
#define MPU9250_BANKSEL_MEM_SEL_LENGTH      5

#define MPU9250_WHO_AM_I_BIT        6
#define MPU9250_WHO_AM_I_LENGTH     8

#define MPU9250_DMP_MEMORY_BANKS        8
#define MPU9250_DMP_MEMORY_BANK_SIZE    256
#define MPU9250_DMP_MEMORY_CHUNK_SIZE   16


// note: DMP code memory blocks defined at end of header file

class MPU9250 {
    public:
        MPU9250();
        MPU9250(uint8 address);

        void initialize();
        boolean testConnection();

        // AUX_VDDIO register
        uint8 getAuxVDDIOLevel();
        void setAuxVDDIOLevel(uint8 level);

        // SMPLRT_DIV register
        uint8 getRate();
        void setRate(uint8 rate);

        // CONFIG register
        uint8 getExternalFrameSync();
        void setExternalFrameSync(uint8 sync);
        uint8 getDLPFMode();
        void setDLPFMode(uint8 bandwidth);

        // GYRO_CONFIG register
        uint8 getFullScaleGyroRange();
        void setFullScaleGyroRange(uint8 range);

        // ACCEL_CONFIG register
        boolean getAccelXSelfTest();
        void setAccelXSelfTest(boolean enabled);
        boolean getAccelYSelfTest();
        void setAccelYSelfTest(boolean enabled);
        boolean getAccelZSelfTest();
        void setAccelZSelfTest(boolean enabled);
        uint8 getFullScaleAccelRange();
        void setFullScaleAccelRange(uint8 range);
        uint8 getDHPFMode();
        void setDHPFMode(uint8 mode);

        // FF_THR register
        uint8 getFreefallDetectionThreshold();
        void setFreefallDetectionThreshold(uint8 threshold);

        // FF_DUR register
        uint8 getFreefallDetectionDuration();
        void setFreefallDetectionDuration(uint8 duration);

        // MOT_THR register
        uint8 getMotionDetectionThreshold();
        void setMotionDetectionThreshold(uint8 threshold);

        // MOT_DUR register
        uint8 getMotionDetectionDuration();
        void setMotionDetectionDuration(uint8 duration);

        // ZRMOT_THR register
        uint8 getZeroMotionDetectionThreshold();
        void setZeroMotionDetectionThreshold(uint8 threshold);

        // ZRMOT_DUR register
        uint8 getZeroMotionDetectionDuration();
        void setZeroMotionDetectionDuration(uint8 duration);

        // FIFO_EN register
        boolean getTempFIFOEnabled();
        void setTempFIFOEnabled(boolean enabled);
        boolean getXGyroFIFOEnabled();
        void setXGyroFIFOEnabled(boolean enabled);
        boolean getYGyroFIFOEnabled();
        void setYGyroFIFOEnabled(boolean enabled);
        boolean getZGyroFIFOEnabled();
        void setZGyroFIFOEnabled(boolean enabled);
        boolean getAccelFIFOEnabled();
        void setAccelFIFOEnabled(boolean enabled);
        boolean getSlave2FIFOEnabled();
        void setSlave2FIFOEnabled(boolean enabled);
        boolean getSlave1FIFOEnabled();
        void setSlave1FIFOEnabled(boolean enabled);
        boolean getSlave0FIFOEnabled();
        void setSlave0FIFOEnabled(boolean enabled);

        // I2C_MST_CTRL register
        boolean getMultiMasterEnabled();
        void setMultiMasterEnabled(boolean enabled);
        boolean getWaitForExternalSensorEnabled();
        void setWaitForExternalSensorEnabled(boolean enabled);
        boolean getSlave3FIFOEnabled();
        void setSlave3FIFOEnabled(boolean enabled);
        boolean getSlaveReadWriteTransitionEnabled();
        void setSlaveReadWriteTransitionEnabled(boolean enabled);
        uint8 getMasterClockSpeed();
        void setMasterClockSpeed(uint8 speed);

        // I2C_SLV* registers (Slave 0-3)
        uint8 getSlaveAddress(uint8 num);
        void setSlaveAddress(uint8 num, uint8 address);
        uint8 getSlaveRegister(uint8 num);
        void setSlaveRegister(uint8 num, uint8 reg);
        boolean getSlaveEnabled(uint8 num);
        void setSlaveEnabled(uint8 num, boolean enabled);
        boolean getSlaveWordByteSwap(uint8 num);
        void setSlaveWordByteSwap(uint8 num, boolean enabled);
        boolean getSlaveWriteMode(uint8 num);
        void setSlaveWriteMode(uint8 num, boolean mode);
        boolean getSlaveWordGroupOffset(uint8 num);
        void setSlaveWordGroupOffset(uint8 num, boolean enabled);
        uint8 getSlaveDataLength(uint8 num);
        void setSlaveDataLength(uint8 num, uint8 length);

        // I2C_SLV* registers (Slave 4)
        uint8 getSlave4Address();
        void setSlave4Address(uint8 address);
        uint8 getSlave4Register();
        void setSlave4Register(uint8 reg);
        void setSlave4OutputByte(uint8 data);
        boolean getSlave4Enabled();
        void setSlave4Enabled(boolean enabled);
        boolean getSlave4InterruptEnabled();
        void setSlave4InterruptEnabled(boolean enabled);
        boolean getSlave4WriteMode();
        void setSlave4WriteMode(boolean mode);
        uint8 getSlave4MasterDelay();
        void setSlave4MasterDelay(uint8 delay);
        uint8 getSlate4InputByte();

        // I2C_MST_STATUS register
        boolean getPassthroughStatus();
        boolean getSlave4IsDone();
        boolean getLostArbitration();
        boolean getSlave4Nack();
        boolean getSlave3Nack();
        boolean getSlave2Nack();
        boolean getSlave1Nack();
        boolean getSlave0Nack();

        // INT_PIN_CFG register
        boolean getInterruptMode();
        void setInterruptMode(boolean mode);
        boolean getInterruptDrive();
        void setInterruptDrive(boolean drive);
        boolean getInterruptLatch();
        void setInterruptLatch(boolean latch);
        boolean getInterruptLatchClear();
        void setInterruptLatchClear(boolean clear);
        boolean getFSyncInterruptLevel();
        void setFSyncInterruptLevel(boolean level);
        boolean getFSyncInterruptEnabled();
        void setFSyncInterruptEnabled(boolean enabled);
        boolean getI2CBypassEnabled();
        void setI2CBypassEnabled(boolean enabled);
        boolean getClockOutputEnabled();
        void setClockOutputEnabled(boolean enabled);

        // INT_ENABLE register
        uint8 getIntEnabled();
        void setIntEnabled(uint8 enabled);
        boolean getIntFreefallEnabled();
        void setIntFreefallEnabled(boolean enabled);
        boolean getIntMotionEnabled();
        void setIntMotionEnabled(boolean enabled);
        boolean getIntZeroMotionEnabled();
        void setIntZeroMotionEnabled(boolean enabled);
        boolean getIntFIFOBufferOverflowEnabled();
        void setIntFIFOBufferOverflowEnabled(boolean enabled);
        boolean getIntI2CMasterEnabled();
        void setIntI2CMasterEnabled(boolean enabled);
        boolean getIntDataReadyEnabled();
        void setIntDataReadyEnabled(boolean enabled);

        // INT_STATUS register
        uint8 getIntStatus();
        boolean getIntFreefallStatus();
        boolean getIntMotionStatus();
        boolean getIntZeroMotionStatus();
        boolean getIntFIFOBufferOverflowStatus();
        boolean getIntI2CMasterStatus();
        boolean getIntDataReadyStatus();

        // ACCEL_*OUT_* registers
        void getMotion9(int16* ax, int16* ay, int16* az, int16* gx, int16* gy, int16* gz, int16* mx, int16* my, int16* mz);
        void getMotion6(int16* ax, int16* ay, int16* az, int16* gx, int16* gy, int16* gz);
        void getAcceleration(int16* x, int16* y, int16* z);
        int16 getAccelerationX();
        int16 getAccelerationY();
        int16 getAccelerationZ();

        // TEMP_OUT_* registers
        int16 getTemperature();

        // GYRO_*OUT_* registers
        void getRotation(int16* x, int16* y, int16* z);
        int16 getRotationX();
        int16 getRotationY();
        int16 getRotationZ();

        // EXT_SENS_DATA_* registers
        uint8 getExternalSensorByte(int position);
        uint16 getExternalSensorWord(int position);
        uint32 getExternalSensorDWord(int position);

        // MOT_DETECT_STATUS register
        boolean getXNegMotionDetected();
        boolean getXPosMotionDetected();
        boolean getYNegMotionDetected();
        boolean getYPosMotionDetected();
        boolean getZNegMotionDetected();
        boolean getZPosMotionDetected();
        boolean getZeroMotionDetected();

        // I2C_SLV*_DO register
        void setSlaveOutputByte(uint8 num, uint8 data);

        // I2C_MST_DELAY_CTRL register
        boolean getExternalShadowDelayEnabled();
        void setExternalShadowDelayEnabled(boolean enabled);
        boolean getSlaveDelayEnabled(uint8 num);
        void setSlaveDelayEnabled(uint8 num, boolean enabled);

        // SIGNAL_PATH_RESET register
        void resetGyroscopePath();
        void resetAccelerometerPath();
        void resetTemperaturePath();

        // MOT_DETECT_CTRL register
        uint8 getAccelerometerPowerOnDelay();
        void setAccelerometerPowerOnDelay(uint8 delay);
        uint8 getFreefallDetectionCounterDecrement();
        void setFreefallDetectionCounterDecrement(uint8 decrement);
        uint8 getMotionDetectionCounterDecrement();
        void setMotionDetectionCounterDecrement(uint8 decrement);

        // USER_CTRL register
        boolean getFIFOEnabled();
        void setFIFOEnabled(boolean enabled);
        boolean getI2CMasterModeEnabled();
        void setI2CMasterModeEnabled(boolean enabled);
        void switchSPIEnabled(boolean enabled);
        void resetFIFO();
        void resetI2CMaster();
        void resetSensors();

        // PWR_MGMT_1 register
        void reset();
        boolean getSleepEnabled();
        void setSleepEnabled(boolean enabled);
        boolean getWakeCycleEnabled();
        void setWakeCycleEnabled(boolean enabled);
        boolean getTempSensorEnabled();
        void setTempSensorEnabled(boolean enabled);
        uint8 getClockSource();
        void setClockSource(uint8 source);

        // PWR_MGMT_2 register
        uint8 getWakeFrequency();
        void setWakeFrequency(uint8 frequency);
        boolean getStandbyXAccelEnabled();
        void setStandbyXAccelEnabled(boolean enabled);
        boolean getStandbyYAccelEnabled();
        void setStandbyYAccelEnabled(boolean enabled);
        boolean getStandbyZAccelEnabled();
        void setStandbyZAccelEnabled(boolean enabled);
        boolean getStandbyXGyroEnabled();
        void setStandbyXGyroEnabled(boolean enabled);
        boolean getStandbyYGyroEnabled();
        void setStandbyYGyroEnabled(boolean enabled);
        boolean getStandbyZGyroEnabled();
        void setStandbyZGyroEnabled(boolean enabled);

        // FIFO_COUNT_* registers
        uint16 getFIFOCount();

        // FIFO_R_W register
        uint8 getFIFOByte();
        void setFIFOByte(uint8 data);
        void getFIFOBytes(uint8 *data, uint8 length);

        // WHO_AM_I register
        uint8 getDeviceID();
        void setDeviceID(uint8 id);
        
        // ======== UNDOCUMENTED/DMP REGISTERS/METHODS ========
        
        // XG_OFFS_TC register
        uint8 getOTPBankValid();
        void setOTPBankValid(boolean enabled);
        int8 getXGyroOffset();
        void setXGyroOffset(int8 offset);

        // YG_OFFS_TC register
        int8 getYGyroOffset();
        void setYGyroOffset(int8 offset);

        // ZG_OFFS_TC register
        int8 getZGyroOffset();
        void setZGyroOffset(int8 offset);

        // X_FINE_GAIN register
        int8 getXFineGain();
        void setXFineGain(int8 gain);

        // Y_FINE_GAIN register
        int8 getYFineGain();
        void setYFineGain(int8 gain);

        // Z_FINE_GAIN register
        int8 getZFineGain();
        void setZFineGain(int8 gain);

        // XA_OFFS_* registers
        int16 getXAccelOffset();
        void setXAccelOffset(int16 offset);

        // YA_OFFS_* register
        int16 getYAccelOffset();
        void setYAccelOffset(int16 offset);

        // ZA_OFFS_* register
        int16 getZAccelOffset();
        void setZAccelOffset(int16 offset);

        // XG_OFFS_USR* registers
        int16 getXGyroOffsetUser();
        void setXGyroOffsetUser(int16 offset);

        // YG_OFFS_USR* register
        int16 getYGyroOffsetUser();
        void setYGyroOffsetUser(int16 offset);

        // ZG_OFFS_USR* register
        int16 getZGyroOffsetUser();
        void setZGyroOffsetUser(int16 offset);
        
        // INT_ENABLE register (DMP functions)
        boolean getIntPLLReadyEnabled();
        void setIntPLLReadyEnabled(boolean enabled);
        boolean getIntDMPEnabled();
        void setIntDMPEnabled(boolean enabled);
        
        // DMP_INT_STATUS
        boolean getDMPInt5Status();
        boolean getDMPInt4Status();
        boolean getDMPInt3Status();
        boolean getDMPInt2Status();
        boolean getDMPInt1Status();
        boolean getDMPInt0Status();

        // INT_STATUS register (DMP functions)
        boolean getIntPLLReadyStatus();
        boolean getIntDMPStatus();
        
        // USER_CTRL register (DMP functions)
        boolean getDMPEnabled();
        void setDMPEnabled(boolean enabled);
        void resetDMP();
        
        // BANK_SEL register
        void setMemoryBank(uint8 bank, boolean prefetchEnabled=false, boolean userBank=false);
        
        // MEM_START_ADDR register
        void setMemoryStartAddress(uint8 address);
        
        // MEM_R_W register
        uint8 readMemoryByte();
        void writeMemoryByte(uint8 data);
        void readMemoryBlock(uint8 *data, uint16 dataSize, uint8 bank=0, uint8 address=0);
        boolean writeMemoryBlock(const uint8 *data, uint16 dataSize, uint8 bank=0, uint8 address=0, boolean verify=true, boolean useProgMem=false);
        boolean writeProgMemoryBlock(const uint8 *data, uint16 dataSize, uint8 bank=0, uint8 address=0, boolean verify=true);

        boolean writeDMPConfigurationSet(const uint8 *data, uint16 dataSize, boolean useProgMem=false);
        boolean writeProgDMPConfigurationSet(const uint8 *data, uint16 dataSize);

        // DMP_CFG_1 register
        uint8 getDMPConfig1();
        void setDMPConfig1(uint8 config);

        // DMP_CFG_2 register
        uint8 getDMPConfig2();
        void setDMPConfig2(uint8 config);

        // special methods for MotionApps 4.1 implementation
        #ifdef MPU9250_INCLUDE_DMP_MOTIONAPPS41
            uint8 *dmpPacketBuffer;
            uint16 dmpPacketSize;

            uint8 dmpInitialize();
            boolean dmpPacketAvailable();

            uint8 dmpSetFIFORate(uint8 fifoRate);
            uint8 dmpGetFIFORate();
            uint8 dmpGetSampleStepSizeMS();
            uint8 dmpGetSampleFrequency();
            int32 dmpDecodeTemperature(int8 tempReg);
            
            // Register callbacks after a packet of FIFO data is processed
            //uint8 dmpRegisterFIFORateProcess(inv_obj_func func, int16 priority);
            //uint8 dmpUnregisterFIFORateProcess(inv_obj_func func);
            uint8 dmpRunFIFORateProcesses();
            
            // Setup FIFO for various output
            uint8 dmpSendQuaternion(uint16 accuracy);
            uint8 dmpSendGyro(uint16 elements, uint16 accuracy);
            uint8 dmpSendAccel(uint16 elements, uint16 accuracy);
            uint8 dmpSendLinearAccel(uint16 elements, uint16 accuracy);
            uint8 dmpSendLinearAccelInWorld(uint16 elements, uint16 accuracy);
            uint8 dmpSendControlData(uint16 elements, uint16 accuracy);
            uint8 dmpSendSensorData(uint16 elements, uint16 accuracy);
            uint8 dmpSendExternalSensorData(uint16 elements, uint16 accuracy);
            uint8 dmpSendGravity(uint16 elements, uint16 accuracy);
            uint8 dmpSendPacketNumber(uint16 accuracy);
            uint8 dmpSendQuantizedAccel(uint16 elements, uint16 accuracy);
            uint8 dmpSendEIS(uint16 elements, uint16 accuracy);

            // Get Fixed Point data from FIFO
            uint8 dmpGetAccel(int32 *data, const uint8* packet=0);
            uint8 dmpGetAccel(int16 *data, const uint8* packet=0);
            uint8 dmpGetAccel(VectorInt16 *v, const uint8* packet=0);
            uint8 dmpGetQuaternion(int32 *data, const uint8* packet=0);
            uint8 dmpGetQuaternion(int16 *data, const uint8* packet=0);
            uint8 dmpGetQuaternion(Quaternion *q, const uint8* packet=0);
            uint8 dmpGet6AxisQuaternion(int32 *data, const uint8* packet=0);
            uint8 dmpGet6AxisQuaternion(int16 *data, const uint8* packet=0);
            uint8 dmpGet6AxisQuaternion(Quaternion *q, const uint8* packet=0);
            uint8 dmpGetRelativeQuaternion(int32 *data, const uint8* packet=0);
            uint8 dmpGetRelativeQuaternion(int16 *data, const uint8* packet=0);
            uint8 dmpGetRelativeQuaternion(Quaternion *data, const uint8* packet=0);
            uint8 dmpGetGyro(int32 *data, const uint8* packet=0);
            uint8 dmpGetGyro(int16 *data, const uint8* packet=0);
            uint8 dmpGetGyro(VectorInt16 *v, const uint8* packet=0);
            uint8 dmpGetMag(int16 *data, const uint8* packet=0);
            uint8 dmpSetLinearAccelFilterCoefficient(float coef);
            uint8 dmpGetLinearAccel(int32 *data, const uint8* packet=0);
            uint8 dmpGetLinearAccel(int16 *data, const uint8* packet=0);
            uint8 dmpGetLinearAccel(VectorInt16 *v, const uint8* packet=0);
            uint8 dmpGetLinearAccel(VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity);
            uint8 dmpGetLinearAccelInWorld(int32 *data, const uint8* packet=0);
            uint8 dmpGetLinearAccelInWorld(int16 *data, const uint8* packet=0);
            uint8 dmpGetLinearAccelInWorld(VectorInt16 *v, const uint8* packet=0);
            uint8 dmpGetLinearAccelInWorld(VectorInt16 *v, VectorInt16 *vReal, Quaternion *q);
            uint8 dmpGetGyroAndAccelSensor(int32 *data, const uint8* packet=0);
            uint8 dmpGetGyroAndAccelSensor(int16 *data, const uint8* packet=0);
            uint8 dmpGetGyroAndAccelSensor(VectorInt16 *g, VectorInt16 *a, const uint8* packet=0);
            uint8 dmpGetGyroSensor(int32 *data, const uint8* packet=0);
            uint8 dmpGetGyroSensor(int16 *data, const uint8* packet=0);
            uint8 dmpGetGyroSensor(VectorInt16 *v, const uint8* packet=0);
            uint8 dmpGetControlData(int32 *data, const uint8* packet=0);
            uint8 dmpGetTemperature(int32 *data, const uint8* packet=0);
            uint8 dmpGetGravity(int32 *data, const uint8* packet=0);
            uint8 dmpGetGravity(int16 *data, const uint8* packet=0);
            uint8 dmpGetGravity(VectorInt16 *v, const uint8* packet=0);
            uint8 dmpGetGravity(VectorFloat *v, Quaternion *q);
            uint8 dmpGetUnquantizedAccel(int32 *data, const uint8* packet=0);
            uint8 dmpGetUnquantizedAccel(int16 *data, const uint8* packet=0);
            uint8 dmpGetUnquantizedAccel(VectorInt16 *v, const uint8* packet=0);
            uint8 dmpGetQuantizedAccel(int32 *data, const uint8* packet=0);
            uint8 dmpGetQuantizedAccel(int16 *data, const uint8* packet=0);
            uint8 dmpGetQuantizedAccel(VectorInt16 *v, const uint8* packet=0);
            uint8 dmpGetExternalSensorData(int32 *data, uint16 size, const uint8* packet=0);
            uint8 dmpGetEIS(int32 *data, const uint8* packet=0);
            
            uint8 dmpGetEuler(float *data, Quaternion *q);
            uint8 dmpGetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity);

            // Get Floating Point data from FIFO
            uint8 dmpGetAccelFloat(float *data, const uint8* packet=0);
            uint8 dmpGetQuaternionFloat(float *data, const uint8* packet=0);

            uint8 dmpProcessFIFOPacket(const unsigned char *dmpData);
            uint8 dmpReadAndProcessFIFOPacket(uint8 numPackets, uint8 *processed=NULL);

            uint8 dmpSetFIFOProcessedCallback(void (*func) (void));

            uint8 dmpInitFIFOParam();
            uint8 dmpCloseFIFO();
            uint8 dmpSetGyroDataSource(uint8 source);
            uint8 dmpDecodeQuantizedAccel();
            uint32 dmpGetGyroSumOfSquare();
            uint32 dmpGetAccelSumOfSquare();
            void dmpOverrideQuaternion(long *q);
            uint16 dmpGetFIFOPacketSize();
        #endif

    private:
        uint8 devAddr;
        uint8 buffer[14];
};

#ifdef MPU9250_INCLUDE_DMP_MOTIONAPPS41
/* ================================================================================================ *
 | Default MotionApps v4.1 48-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][MAG X ][MAG Y ][MAG Z ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ] |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41  42  43  44  45  46  47  |
 * ================================================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
const unsigned char dmpMemory[MPU9250_DMP_CODE_SIZE]  = {
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,
    
    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x78, 0xA2,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    
    // bank 3, 256 bytes
    0xD8, 0xDC, 0xF4, 0xD8, 0xB9, 0xAB, 0xF3, 0xF8, 0xFA, 0xF1, 0xBA, 0xA2, 0xDE, 0xB2, 0xB8, 0xB4,
    0xA8, 0x81, 0x98, 0xF7, 0x4A, 0x90, 0x7F, 0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA,
    0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2, 0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80,
    0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF, 0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0,
    0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C, 0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1,
    0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1, 0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3,
    0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01, 0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88,
    0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80, 0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF,
    0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89,
    0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80, 0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9,
    0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E, 0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A,
    0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9, 0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11,
    0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55,
    0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xF0, 0x00, 0x28, 0x50, 0xF5, 0xBA, 0xAD, 0x8F, 0x9F, 0x28, 0x54,
    0x7C, 0xB9, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xDB, 0xB2, 0xB6, 0x8E, 0x9D,
    0xAE, 0xF5, 0x60, 0x68, 0x70, 0xB1, 0xB5, 0xF1, 0xDA, 0xA6, 0xDF, 0xD9, 0xA6, 0xFA, 0xA3, 0x86,
    
    // bank 4, 256 bytes
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    
    // bank 5, 256 bytes
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0x97, 0x86,
    0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97, 0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40,
    0xB9, 0xA3, 0x8A, 0xC3, 0xC5, 0xC7, 0x9A, 0xA3, 0x28, 0x50, 0x78, 0xF1, 0xB5, 0x93, 0x01, 0xD9,
    0xDF, 0xDF, 0xDF, 0xD8, 0xB8, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04, 0x28, 0x51, 0x79, 0x1D, 0x30,
    0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78, 0x78, 0x9B, 0xF1, 0x1A, 0xB0,
    0xF0, 0xB1, 0x83, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0xB0, 0x8B, 0x29, 0x51, 0x79, 0xB1, 0x83, 0x24,

    // bank 6, 256 bytes
    0x70, 0x59, 0xB0, 0x8B, 0x20, 0x58, 0x71, 0xB1, 0x83, 0x44, 0x69, 0x38, 0xB0, 0x8B, 0x39, 0x40,
    0x68, 0xB1, 0x83, 0x64, 0x48, 0x31, 0xB0, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71,
    0x58, 0x44, 0x68, 0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0,
    0x8C, 0xA8, 0x04, 0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02,
    0x26, 0x46, 0x66, 0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38,
    0x64, 0x48, 0x31, 0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19,
    0x31, 0x48, 0x60, 0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86,
    0xA8, 0x6E, 0x76, 0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A,
    0x6E, 0x8A, 0x56, 0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E,
    0x9D, 0xB8, 0xAD, 0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55,
    0x7D, 0x81, 0x91, 0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D,
    0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51,
    0xD9, 0x04, 0xAE, 0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19,
    0x81, 0xAD, 0xD9, 0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9,
    0xAD, 0xAD, 0xAD, 0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76,
    0xF3, 0xAC, 0x2E, 0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC,
    
    // bank 7, 170 bytes (remainder)
    0x30, 0x18, 0xA8, 0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24,
    0xF2, 0xB0, 0x89, 0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9,
    0xD8, 0xD8, 0x79, 0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D,
    0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D,
    0x80, 0x25, 0xDA, 0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34,
    0x3C, 0xF3, 0xAB, 0x8B, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xB0, 0x87, 0x9C, 0xB9,
    0xA3, 0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3,
    0xA3, 0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3,
    0xA3, 0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3,
    0xDC, 0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

const unsigned char dmpConfig[MPU9250_DMP_CONFIG_SIZE]  = {
//  BANK    OFFSET  LENGTH  [DATA]
    0x02,   0xEC,   0x04,   0x00, 0x47, 0x7D, 0x1A,   // ?
    0x03,   0x82,   0x03,   0x4C, 0xCD, 0x6C,         // FCFG_1 inv_set_gyro_calibration
    0x03,   0xB2,   0x03,   0x36, 0x56, 0x76,         // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCA, 0xE3, 0x09,   // D_0_104 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // D_1_152 inv_set_accel_calibration
    0x03,   0x86,   0x03,   0x0C, 0xC9, 0x2C,         // FCFG_2 inv_set_accel_calibration
    0x03,   0x90,   0x03,   0x26, 0x46, 0x66,         //   (continued)...FCFG_2 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x40, 0x00,               // D_0_108 inv_set_accel_calibration

    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x40, 0x00, 0x00, 0x00,   // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x40, 0x00, 0x00, 0x00,   // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0xC0, 0x00, 0x00, 0x00,   // CPASS_MTX_22

    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,   // D_1_236 inv_apply_endian_accel
    0x03,   0x86,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04,   0x22,   0x03,   0x0D, 0x35, 0x5D,         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x00,   0xA3,   0x01,   0x00,                     // ?
    0x04,   0x29,   0x04,   0x87, 0x2D, 0x35, 0x3D,   // FCFG_5 inv_set_bias_update
    0x07,   0x62,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
    0x07,   0x9F,   0x01,   0x30,                     // CFG_16 inv_set_footer
    0x07,   0x67,   0x01,   0x9A,                     // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x68,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x62,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // ?
    0x02,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // ?
    0x07,   0x83,   0x06,   0xC2, 0xCA, 0xC4, 0xA3, 0xA3, 0xA3, // ?
                 // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01, // SET INT_ENABLE, SPECIAL INSTRUCTION
    0x07,   0xA7,   0x01,   0xFE,                     // ?
    0x07,   0x62,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // ?
    0x07,   0x67,   0x01,   0x9A,                     // ?
    0x07,   0x68,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x07,   0x8D,   0x04,   0xF1, 0x28, 0x30, 0x38,   // ??? CFG_12 inv_send_mag -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x04                // D_0_22 inv_set_fifo_rate

    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};


const unsigned char dmpUpdates[MPU9250_DMP_UPDATES_SIZE] = {
    0x01,   0xB2,   0x02,   0xFF, 0xF5,
    0x01,   0x90,   0x04,   0x0A, 0x0D, 0x97, 0xC0,
    0x00,   0xA3,   0x01,   0x00,
    0x04,   0x29,   0x04,   0x87, 0x2D, 0x35, 0x3D,
    0x01,   0x6A,   0x02,   0x06, 0x00,
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x02,   0x60,   0x0C,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x01,   0x08,   0x02,   0x01, 0x20,
    0x01,   0x0A,   0x02,   0x00, 0x4E,
    0x01,   0x02,   0x02,   0xFE, 0xB3,
    0x02,   0x6C,   0x04,   0x00, 0x00, 0x00, 0x00, // READ
    0x02,   0x6C,   0x04,   0xFA, 0xFE, 0x00, 0x00,
    0x02,   0x60,   0x0C,   0xFF, 0xFF, 0xCB, 0x4D, 0x00, 0x01, 0x08, 0xC1, 0xFF, 0xFF, 0xBC, 0x2C,
    0x02,   0xF4,   0x04,   0x00, 0x00, 0x00, 0x00,
    0x02,   0xF8,   0x04,   0x00, 0x00, 0x00, 0x00,
    0x02,   0xFC,   0x04,   0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
};
#endif


#endif /* _MPU9250_H_ */

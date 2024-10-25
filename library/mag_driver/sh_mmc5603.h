//
// Created by yueluo on 3/4/2022.
//

#ifndef MOTION_SH_MMC5603NJ_H
#define MOTION_SH_MMC5603NJ_H

#include <stdint-gcc.h>

#define MMC5603NJ_I2C_ADDRESS      0x30

#define MMC5603NJ_XOUT_0           0x00
#define MMC5603NJ_XOUT_1           0x01
#define MMC5603NJ_YOUT_0           0x02
#define MMC5603NJ_YOUT_1           0x03
#define MMC5603NJ_ZOUT_0           0x04
#define MMC5603NJ_ZOUT_1           0x05
#define MMC5603NJ_XOUT_2           0x06
#define MMC5603NJ_YOUT_2           0x07
#define MMC5603NJ_ZOUT_2           0x08

#define MMC5603NJ_TOUT             0x09

#define MMC5603NJ_ODR              0x1A
#define MMC5603NJ_STATUS           0x18
#define MMC5603NJ_CONTROL_0        0x1B
#define MMC5603NJ_CONTROL_1        0x1C
#define MMC5603NJ_CONTROL_2        0x1D

#define MMC5603NJ_ST_X_TH          0x1E
#define MMC5603NJ_ST_Y_TH          0x1F
#define MMC5603NJ_ST_Z_TH          0x20
#define MMC5603NJ_ST_X             0x27
#define MMC5603NJ_ST_Y             0x28
#define MMC5603NJ_ST_Z             0x29

#define MMC5603NJ_PRODUCT_ID       0x39

/* Bit definition for control register ODR 0x1A */
#define MMC5603NJ_CMD_ODR_1HZ		    0x01
#define MMC5603NJ_CMD_ODR_5HZ		    0x05
#define MMC5603NJ_CMD_ODR_10HZ	        0x0A
#define MMC5603NJ_CMD_ODR_50HZ	        0x32
#define MMC5603NJ_CMD_ODR_100HZ	        0x64
#define MMC5603NJ_CMD_ODR_200HZ	        0xC8
#define MMC5603NJ_CMD_ODR_255HZ	        0xFF

/* Bit definition for control register 0 0x1B */
#define MMC5603NJ_CMD_TMM				0x01
#define MMC5603NJ_CMD_TMT         	    0x02
#define MMC5603NJ_CMD_START_MDT		    0x04
#define MMC5603NJ_CMD_SET				0x08
#define MMC5603NJ_CMD_RESET			    0x10
#define MMC5603NJ_CMD_AUTO_SR_EN		0x20
#define MMC5603NJ_CMD_AUTO_ST_EN		0x40
#define MMC5603NJ_CMD_CMM_FREQ_EN		0x80

/* Bit definition for control register 1 0x1C */
#define MMC5603NJ_CMD_BW00			    0x00
#define MMC5603NJ_CMD_BW01			    0x01
#define MMC5603NJ_CMD_BW10			    0x02
#define MMC5603NJ_CMD_BW11			    0x03
#define MMC5603NJ_CMD_ST_ENP			0x20
#define MMC5603NJ_CMD_ST_ENM			0x40
#define MMC5603NJ_CMD_SW_RST			0x80

/* Bit definition for control register 2 0x1D */
#define MMC5603NJ_CMD_PART_SET1		    0x00
#define MMC5603NJ_CMD_PART_SET25		0x01
#define MMC5603NJ_CMD_PART_SET75		0x02
#define MMC5603NJ_CMD_PART_SET100		0x03
#define MMC5603NJ_CMD_PART_SET250		0x04
#define MMC5603NJ_CMD_PART_SET500		0x05
#define MMC5603NJ_CMD_PART_SET1000	    0x06
#define MMC5603NJ_CMD_PART_SET2000	    0x07
#define MMC5603NJ_CMD_EN_PART_SET		0x08
#define MMC5603NJ_CMD_CMM_EN			0x10
#define MMC5603NJ_CMD_INT_MDT_EN		0x20
#define MMC5603NJ_CMD_INT_MD_EN		    0x40
#define MMC5603NJ_CMD_HPOWER			0x80

#define MMC5603NJ_PRODUCT_ID_VAL		0x10
#define MMC5603NJ_MM_DONE_INT			0x01
#define MMC5603NJ_MT_DONE_INT			0x02
#define MMC5603NJ_MDT_FLAG_INT		    0x04
#define MMC5603NJ_ST_FAIL_INT			0x08
#define MMC5603NJ_OTP_READ_DONE		    0x10
#define MMC5603NJ_SAT_SENSOR			0x20
#define MMC5603NJ_MM_DONE				0x40
#define MMC5603NJ_MT_DONE				0x80

// 16-bit mode, null field output (32768)
#define	MMC5603NJ_16BIT_OFFSET		    32768
#define	MMC5603NJ_16BIT_SENSITIVITY	    1024

#endif //MOTION_SH_MMC5603NJ_H

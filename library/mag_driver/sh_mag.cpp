//
// Created by yueluo on 4/2/2022.
//

#include "sh_mag.h"
#include "stm32g4xx_hal.h"
//RTOS
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

int32_t Lsm6d_sh_mag::getMagDevId() {
#if defined(USE_MAG_AKM09918C)
    uint8_t buf[2];
    uint16_t id = 0;
    SH_IO_Read(buf, AK09918_WIA1, 2);
    id = lwutil_ld_u16_le(buf);
    return id;
#elif defined(USE_MAG_MMC5603NJ)
    uint8_t buf[1];
    uint8_t id = 0;
    SH_IO_Read(buf, MMC5603NJ_PRODUCT_ID, 1);
    id = (uint8_t)buf[0];
    return id;
#elif defined(USE_MAG_RM3100)
    uint8_t buf[1];
    uint8_t id = 0;
    SH_IO_Read(buf, RM3100_REVID, 1);
    id = (uint8_t)buf[0];
    return id;
#endif
}

void Lsm6d_sh_mag::setupMag() {
#if defined(USE_MAG_AKM09918C)
    /* soft reset akm */
    SH_IO_Write((uint8_t *) AK09918_SRST_BIT, AK09918_CNTL3, 1);

    // The power on time is 50ms.
    vTaskDelay(600);

    /* power down */
    // SH_IO_Write((uint8_t *) AK09918_POWER_DOWN, AK09918_CNTL2, 1); // after reset, it should always be power down

    /* change odr to 100hz */
    SH_IO_Write((uint8_t *) AK09918_CONTINUOUS_100HZ, AK09918_CNTL2, 1);

#elif defined(USE_MAG_MMC5603NJ)
    /* soft reset mmc */
    SH_IO_Write((uint8_t *) MMC5603NJ_CMD_SW_RST, MMC5603NJ_CONTROL_1, 1);

    // The power on time is 20ms.
    vTaskDelay(250);

    // TODO: set bandwidth?

    SH_IO_Write((uint8_t *) MMC5603NJ_CMD_ODR_100HZ, MMC5603NJ_ODR, 1);

    /* Set Auto_SR_en bit '1', Enable the function of automatic set/reset */
    /* Set Cmm_freq_en bit '1', Start the calculation of the measurement period according to the ODR*/
    SH_IO_Write((uint8_t *)(MMC5603NJ_CMD_CMM_FREQ_EN | MMC5603NJ_CMD_AUTO_SR_EN), MMC5603NJ_CONTROL_0, 1);

    /* Set Cmm_en bit '1', Enter continuous mode */
    SH_IO_Write((uint8_t *) MMC5603NJ_CMD_CMM_EN, MMC5603NJ_CONTROL_2, 1);

#elif defined(USE_MAG_RM3100)

#endif
}

float Lsm6d_sh_mag::normalizeRawMag(int16_t rawMag) {

#if defined(USE_MAG_AKM09918C)
    return ((float)rawMag * 1.5f);
#elif defined(USE_MAG_MMC5603NJ)
    return ((float)rawMag - MMC5603NJ_16BIT_OFFSET) / MMC5603NJ_16BIT_SENSITIVITY;
#elif defined(USE_MAG_RM3100)

#endif
}

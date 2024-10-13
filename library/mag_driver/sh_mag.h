//
// Created by yueluo on 4/2/2022.
//

#ifndef MOTION_F411_LSM6D_SH_MAG_H
#define MOTION_F411_LSM6D_SH_MAG_H

#define USE_MAG_MMC5603NJ

// #include <cstdint>

// #include "../conf.h"
// #include "../Util/lwutil.h"

#if defined(USE_MAG_AKM09918C)
#include "sh_akm00918.h"
#elif defined(USE_MAG_MMC5603NJ)
#include "sh_mmc5603.h"
#elif defined(USE_MAG_RM3100)
#include "sh_rm3100.h"
#endif

#ifdef __cplusplus

class Lsm6d_sh_mag {
public:
    ~Lsm6d_sh_mag() = default;

    int32_t getMagDevId();
    void setupMag();
    float normalizeRawMag(int16_t rawMag);

    virtual uint8_t SH_IO_Write(uint8_t *buf, uint8_t reg, uint16_t num) = 0;
    virtual uint8_t SH_IO_Read(uint8_t *buf, uint8_t reg, uint16_t num) = 0;

protected:
#if defined(USE_MAG_AKM09918C)
    constexpr static int32_t SH_IIC_ADDRESS = AK09918_I2C_ADDR;
    constexpr static int32_t SH_SENSOR_ID = AK09918_WIA_VAL;
    /* trick to read out XL YL ZL in a onetime sh IO */
    constexpr static int32_t SH_MAG_XOUT_ADDR = AK09918_HXL;
    constexpr static int32_t SH_MAG_XOUT_LEN = 6;
#elif defined(USE_MAG_MMC5603NJ)
    constexpr static int32_t SH_IIC_ADDRESS = MMC5603NJ_I2C_ADDRESS;
    constexpr static int32_t SH_SENSOR_ID = MMC5603NJ_PRODUCT_ID_VAL;
    /* trick to read out XL YL ZL in a single time IO */
    constexpr static int32_t SH_MAG_XOUT_ADDR = MMC5603NJ_XOUT_0;
    constexpr static int32_t SH_MAG_XOUT_LEN = 6;
#elif defined(USE_MAG_RM3100)
    constexpr static int32_t SH_IIC_ADDRESS = RM3100_I2C_ADDRESS;
    constexpr static int32_t SH_SENSOR_ID = RM3100_REVID;
#endif

};

#endif //__cplusplus

#endif //MOTION_F411_LSM6D_SH_MAG_H

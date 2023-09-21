#ifndef INC_MPU60X0_H
#define INC_MPU60X0_H

#include "core/common-defines.h"
// Edit the define below to access either SPI or I2C functions depending on your configuration
//#define MPU_USE_SPI 1
#define MPU_USE_I2C 1
/*
 * 			Registers
 */

#define MPU60X0_REG_SELF_TEST_X				(0x0D)
#define MPU60X0_REG_SELF_TEST_Y				(0x0E)
#define MPU60X0_REG_SELF_TEST_Z				(0x0F)
#define MPU60X0_REG_SELF_TEST_A				(0x10)
#define MPU60X0_REG_SMPLRT_DIV				(0x19)
#define MPU60X0_REG_CONFIG					(0x1A)
#define MPU60X0_REG_GYRO_CONFIG				(0x1B)
#define MPU60X0_REG_ACCEL_CONFIG			(0x1C)

#define MPU60X0_REG_FIFO_EN					(0x23)

#define MPU60X0_REG_INT_PIN_CFG				(0x37)
#define MPU60X0_REG_INT_ENABLE				(0x38)
#define MPU60X0_REG_INT_STATUS				(0x3A)
#define MPU60X0_REG_ACCEL_XOUT_H			(0x3B)
#define MPU60X0_REG_ACCEL_XOUT_L			(0x3C)
#define MPU60X0_REG_ACCEL_YOUT_H			(0x3D)
#define MPU60X0_REG_ACCEL_YOUT_L			(0x3E)
#define MPU60X0_REG_ACCEL_ZOUT_H			(0x3F)
#define MPU60X0_REG_ACCEL_ZOUT_L			(0x40)
#define MPU60X0_REG_TEMP_OUT_H				(0x41)
#define MPU60X0_REG_TEMP_OUT_L				(0x42)
#define MPU60X0_REG_GYRO_XOUT_H				(0x43)
#define MPU60X0_REG_GYRO_XOUT_L				(0x44)
#define MPU60X0_REG_GYRO_YOUT_H				(0x45)
#define MPU60X0_REG_GYRO_YOUT_L				(0x46)
#define MPU60X0_REG_GYRO_ZOUT_H				(0x47)
#define MPU60X0_REG_GYRO_ZOUT_L				(0x48)

#define MPU60X0_REG_SIGNAL_PATH_RESET		(0x68)
#define MPU60X0_REG_MOT_DETECT_CTRL			(0x69)
#define MPU60X0_REG_USER_CTRL				(0x6A)
#define MPU60X0_REG_PWR_MGMT_1				(0x6B)
#define MPU60X0_REG_PWR_MGMT_2				(0x6C)
#define MPU60X0_REG_FIFO_COUNTH				(0x72)
#define MPU60X0_REG_FIFO_COUNTL				(0x73)
#define MPU60X0_REG_FIFO_R_W				(0x74)
#define MPU60X0_REG_WHO_AM_I				(0x75)

/*
 * 		Default register configurations
 */

#define MPU60X0_REG_WHO_AM_I_DEFAULT		(0x68)	// Default value stored in WHO_AM_I register

#define GYRO_SELF_TEST                      (0x07U << 5)
#define ACCEL_SELF_TEST                     (0x08U << 4)
#define MPU60X0_ENABLE                      (0x00U << 6)
#define MPU60X0_DISABLE                     (0x01U << 6)

#define MPU60X0_SPI_READ_MASK               (0x80)
#define MPU60X0_SPI_WRITE_MASK              (0x00)

#define DEFAULT_MPU_TIMEOUT                 (50)

typedef struct mpu60X0_config_t {
        uint8_t sample_rate;
        uint8_t low_pass_filter;
        uint8_t accel_high_pass_filter;
        uint8_t fifo_enable;
        uint8_t interrupt_mode;
        uint8_t interrupt_enable;
        uint8_t timeout_ms;  
} mpu60X0_config_t;

typedef enum MPU60X0_Status {
        MPU_OK,
        MPU_TIMED_OUT,
        WHO_AM_I_FAIL,
        SELF_TEST,


} MPU60X0_Status;

#ifdef MPU_USE_SPI
    typedef struct mpu60X0_t    {
        spi_handle_t* spi;
        mpu60X0_config_t config;
        simple_timer_t timeout_timer;
        MPU60X0_Status status;
        uint16_t accel_x_raw;
        uint16_t accel_y_raw;
        uint16_t accel_z_raw;
        uint16_t gyro_x_raw;
        uint16_t gyro_y_raw;
        uint16_t gyro_z_raw;
    } mpu60X0_t;

    void mpu60X0_setup(mpu60X0_t* mpu60X0,spi_handle_t* spi);
    void mpu60X0_teardown(mpu60X0_t* mpu60X0);
    void mpu60X0_update(mpu60X0_t* mpu60X0);
    uint8_t mpu60X0_get_fifo_count(mpu60X0_t* mpu60X0);
    bool mpu60X0_data_available(mpu60X0_t* mpu60X0);
    MPU60X0_Status mpu60X0_read_register(mpu60X0_t* mpu60X0, uint8_t address, uint8_t* data);
    MPU60X0_Status mpu60X0_write_register(mpu60X0_t* mpu60X0, uint8_t address, uint8_t data);
#endif
#ifdef MPU_USE_I2C

    typedef struct mpu60X0_t    {
        i2c_handle_t* i2c;
        mpu60X0_config_t config;
        simple_timer_t timeout_timer;
        MPU60X0_Status status;
        uint16_t accel_x_raw;
        uint16_t accel_y_raw;
        uint16_t accel_z_raw;
        uint16_t gyro_x_raw;
        uint16_t gyro_y_raw;
        uint16_t gyro_z_raw;
    } mpu60X0_t;

    void mpu60X0_setup(mpu60X0_t* mpu60X0,i2c_handle_t* i2c);
    void mpu60X0_teardown(mpu60X0_t* mpu60X0);
    void mpu60X0_update(mpu60X0_t* mpu60X0);
    uint8_t mpu60X0_get_fifo_count(mpu60X0_t* mpu60X0);
    bool mpu60X0_data_available(mpu60X0_t* mpu60X0);
    MPU60X0_Status mpu60X0_read_register(mpu60X0_t* mpu60X0, uint8_t address, uint8_t* data);
    MPU60X0_Status mpu60X0_write_register(mpu60X0_t* mpu60X0, uint8_t address, uint8_t data);

#endif

#endif/* INC_MPU60X0_H */

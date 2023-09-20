#ifndef INC_MPU6050_H
#define INC_MPU6050_H

#include "core/common-defines.h"
/*
 * 			Registers
 */

#define MPU6050_REG_SELF_TEST_X				(0x0D)
#define MPU6050_REG_SELF_TEST_Y				(0x0E)
#define MPU6050_REG_SELF_TEST_Z				(0x0F)
#define MPU6050_REG_SELF_TEST_A				(0x10)
#define MPU6050_REG_SMPLRT_DIV				(0x19)
#define MPU6050_REG_CONFIG					(0x1A)
#define MPU6050_REG_GYRO_CONFIG				(0x1B)
#define MPU6050_REG_ACCEL_CONFIG			(0x1C)

#define MPU6050_REG_FIFO_EN					(0x23)

#define MPU6050_REG_INT_PIN_CFG				(0x37)
#define MPU6050_REG_INT_ENABLE				(0x38)
#define MPU6050_REG_INT_STATUS				(0x3A)
#define MPU6050_REG_ACCEL_XOUT_H			(0x3B)
#define MPU6050_REG_ACCEL_XOUT_L			(0x3C)
#define MPU6050_REG_ACCEL_YOUT_H			(0x3D)
#define MPU6050_REG_ACCEL_YOUT_L			(0x3E)
#define MPU6050_REG_ACCEL_ZOUT_H			(0x3F)
#define MPU6050_REG_ACCEL_ZOUT_L			(0x40)
#define MPU6050_REG_TEMP_OUT_H				(0x41)
#define MPU6050_REG_TEMP_OUT_L				(0x42)
#define MPU6050_REG_GYRO_XOUT_H				(0x43)
#define MPU6050_REG_GYRO_XOUT_L				(0x44)
#define MPU6050_REG_GYRO_YOUT_H				(0x45)
#define MPU6050_REG_GYRO_YOUT_L				(0x46)
#define MPU6050_REG_GYRO_ZOUT_H				(0x47)
#define MPU6050_REG_GYRO_ZOUT_L				(0x48)

#define MPU6050_REG_SIGNAL_PATH_RESET		(0x68)
#define MPU6050_REG_MOT_DETECT_CTRL			(0x69)
#define MPU6050_REG_USER_CTRL				(0x6A)
#define MPU6050_REG_PWR_MGMT_1				(0x6B)
#define MPU6050_REG_PWR_MGMT_2				(0x6C)
#define MPU6050_REG_FIFO_COUNTH				(0x72)
#define MPU6050_REG_FIFO_COUNTL				(0x73)
#define MPU6050_REG_FIFO_R_W				(0x74)
#define MPU6050_REG_WHO_AM_I				(0x75)

/*
 * 		Default register configurations
 */

#define MPU6050_REG_WHO_AM_I_DEFAULT		(0x68)	// Default value stored in WHO_AM_I register

#define GYRO_SELF_TEST                      (0x07U << 5)
#define ACCEL_SELF_TEST                     (0x08U << 4)

#define MPU6050_READ_MASK                   (0x80)
#define MPU6050_WRITE_MASK                  (0x00)

#define DEFAULT_MPU_TIMEOUT                 (50)

typedef struct mpu6050_config_t {
    uint8_t sample_rate;
    uint8_t low_pass_filter;
    uint8_t acc_high_pass_filter;
    uint8_t fifo_enable;
    uint8_t interrupt_mode;
    uint8_t interrupt_enable;
    uint8_t timeout_ms;  
} mpu6050_config_t;

typedef struct mpu6050_t    {
    spi_handle_t* spi;
    mpu6050_config_t config;
    simple_timer_t timeout_timer;
    MPU6050_Status status;
    uint16_t acc_x_raw;
    uint16_t acc_y_raw;
    uint16_t acc_z_raw;
    uint16_t gyro_x_raw;
    uint16_t gyro_y_raw;
    uint16_t gyro_z_raw;
} mpu6050_t;

typedef enum MPU6050_Status {
    MPU_OK,
    MPU_TIMED_OUT,
    WHO_AM_I_FAIL,
    SELF_TEST,

} MPU6050_Status;

void mpu6050_setup(mpu6050_t* mpu6050,spi_handle_t* spi);
void mpu6050_teardown(mpu6050_t* mpu6050);
void mpu6050_read(mpu6050_t* mpu6050);
uint8_t mpu6050_get_fifo_count(mpu6050_t* mpu6050);
bool mpu6050_data_available(mpu6050_t* mpu6050);
MPU6050_Status mpu6050_read_register(mpu6050_t* mpu6050, uint8_t address, uint8_t* data);
MPU6050_Status mpu6050_write_register(mpu6050_t* mpu6050, uint8_t address, uint8_t* data);



#endif/* INC_MPU6050_H */

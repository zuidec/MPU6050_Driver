#include "mpu6050.h"
#include "core/spi.h"
#include "core/simple-timer.h"



static void mpu6050_self_test(mpu6050_t* mpu6050) {
    // Gyro self test
    spi_write_byte(mpu6050->spi, MPU6050_REG_GYRO_CONFIG);
    spi_write_byte(mpu6050->spi, GYRO_SELF_TEST);
    // Accelerometer self test
    spi_write_byte(mpu6050->spi, MPU6050_REG_ACCEL_CONFIG);
    spi_write_byte(mpu6050->spi, ACCEL_SELF_TEST);
    
    mpu6050->status = MPU_OK;
}

void mpu6050_setup(mpu6050_t* mpu6050, spi_handle_t* spi)    {
    
    // Attach informatioon from SPI to our MPU6050 and initialize readings
    mpu6050->acc_x_raw = 0;
    mpu6050->acc_y_raw = 0;
    mpu6050->acc_z_raw = 0;
    mpu6050->gyro_x_raw = 0;
    mpu6050->gyro_y_raw = 0;
    mpu6050->gyro_z_raw = 0;
    mpu6050->spi = spi;
    
    // Check to see if device really is MPU6050
    uint8_t response = 0x00;
    
 
    if(mpu6050_read_register(mpu6050, MPU6050_REG_WHO_AM_I, &response) != MPU6050_REG_WHO_AM_I_DEFAULT)  {
        mpu6050_teardown(mpu6050);
        mpu6050->status = WHO_AM_I_FAIL;
        return;
    }
    mpu6050->status = SELF_TEST;
    
    mpu6050_self_test(mpu6050);

    // Set config once self test is complete
    mpu6050->config.sample_rate = ;
    mpu6050->config.low_pass_filter = ;
    mpu6050->config.acc_high_pass_filter = ;
    mpu6050->config.fifo_enable = ;
    mpu6050->config.interrupt_mode = ;
    mpu6050->config.interrupt_enable = ;
    mpu6050->config.timeout_ms = DEFAULT_MPU_TIMEOUT;
    simple_timer_setup(&mpu6050->timeout_timer, mpu6050->config.timeout_ms, false);
}

void mpu6050_teardown(mpu6050_t* mpu6050) {


}

void mpu6050_read(mpu6050_t* mpu6050)   {


}
MPU6050_Status mpu6050_read_register(mpu6050_t* mpu6050, uint8_t address, uint8_t* data)   {
    
    spi_write_byte(mpu6050->spi, address | MPU6050_READ_MASK);
    simple_timer_reset(&mpu6050->timeout_timer);
    
    while(!spi1_data_available()) {
        if(simple_timer_has_elapsed(&mpu6050->timeout_timer)) {
            
            return MPU_TIMED_OUT;
        }
    }

    data = spi_read_byte(mpu6050->spi);
    return MPU_OK;
}
MPU6050_Status mpu6050_write_register(mpu6050_t* mpu6050, uint8_t address, uint8_t* data)   {
    
    spi_write_byte(mpu6050->spi, address | MPU6050_READ_MASK);
    spi_write_byte(mpu6050->spi, data);
    return MPU_OK;

}

uint8_t mpu6050_get_fifo_count(mpu6050_t* mpu6050)  {


}

bool mpu6050_data_available(mpu6050_t* mpu6050)   {
    return spi_data_available(mpu6050->spi);
}

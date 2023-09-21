#ifdef MPU_USE_I2C
    #include "core/i2c.h"
#elif   MPU_USE_SPI
    #include "core/spi.h"
#endif

#include "mpu60X0.h"
#include "core/simple-timer.h"


///
///         Common functions
///

void mpu60X0_update(mpu60X0_t* mpu60X0)   {
    uint8_t response[12] = {0x00};
    // Read x, then y, then z registers H->L for the gyro and then the accelerometer
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_GYRO_XOUT_H, response[0]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_GYRO_XOUT_L, response[1]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_GYRO_YOUT_H, response[2]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_GYRO_YOUT_L, response[3]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_GYRO_ZOUT_H, response[4]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_GYRO_ZOUT_L, response[5]);

    mpu60X0_read_register(mpu60X0, MPU60X0_REG_ACCEL_XOUT_H, response[6]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_ACCEL_XOUT_L, response[7]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_ACCEL_YOUT_H, response[8]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_ACCEL_YOUT_L, response[9]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_ACCEL_ZOUT_H, response[10]);
    mpu60X0_read_register(mpu60X0, MPU60X0_REG_ACCEL_ZOUT_L, response[11]);

    // Work through the data, shifting the high byte to the top
    mpu60X0->gyro_x_raw =  (uint16_t)((response[0] << 8) | response[1]);
    mpu60X0->gyro_y_raw =  (uint16_t)((response[2] << 8) | response[3]);
    mpu60X0->gyro_z_raw =  (uint16_t)((response[4] << 8) | response[5]);
    mpu60X0->accel_x_raw = (uint16_t)((response[0] << 8) | response[1]);
    mpu60X0->accel_y_raw = (uint16_t)((response[2] << 8) | response[3]);
    mpu60X0->accel_z_raw = (uint16_t)((response[4] << 8) | response[5]);

}
void mpu60X0_enable(mpu60X0_t* mpu60X0) {
    mpu60X0_write_register(mpu60X0, MPU60X0_REG_PWR_MGMT_1, MPU60X0_ENABLE);
}

void mpu60X0_disable(mpu60X0_t* mpu60X0) {
    mpu60X0_write_register(mpu60X0, MPU60X0_REG_PWR_MGMT_1, MPU60X0_DISABLE);
}

///
///         Functions configured to use with I2C interface
///

#ifdef MPU_USE_I2C


#endif



///
///         Functions configured to use with SPI interface (MPU6000 only)
///

#ifdef MPU_USE_SPI
void mpu60X0_setup(mpu60X0_t* mpu60X0, spi_handle_t* spi)    {
    
    // Attach informatioon from SPI to our MPU60X0 and initialize readings
    mpu60X0->accel_x_raw = 0;
    mpu60X0->accel_y_raw = 0;
    mpu60X0->accel_z_raw = 0;
    mpu60X0->gyro_x_raw = 0;
    mpu60X0->gyro_y_raw = 0;
    mpu60X0->gyro_z_raw = 0;
    mpu60X0->spi = spi;

    // Configure timeout timer before any reads in case things don't go well
    mpu60X0->config.timeout_ms = DEFAULT_MPU_TIMEOUT;
    simple_timer_setup(&mpu60X0->timeout_timer, mpu60X0->config.timeout_ms, false);

    // Reset device in case there was a bad shutdown/powerup sequence.
    // According to datasheet the sequence should be: 
    //      set DEVICE_RESET = 1; wait 100ms; 
    //      set GYRO_RESET = ACCEL_RESET = TEMP_RESET = 1; wait 100ms
    // 
    // The manual also recommends using a gyro clock as reference, 
    // even though external oscillator is default
    //



    // Check to see if device really is MPU60X0 by checking WHO_AM_I register
    uint8_t response = 0x00;
    
    if(mpu60X0_read_register(mpu60X0, MPU60X0_REG_WHO_AM_I, &response) != MPU_OK)  {
        mpu60X0_teardown(mpu60X0);
        mpu60X0->status = WHO_AM_I_FAIL;
        return;
    }
    else if( response != MPU60X0_REG_WHO_AM_I_DEFAULT)  {
        mpu60X0_teardown(mpu60X0);
        mpu60X0->status = WHO_AM_I_FAIL;
        return;
    }

    mpu60X0->status = MPU_OK;
    // Set config once self test is complete
    mpu60X0->config.sample_rate = ;
    mpu60X0->config.low_pass_filter = ;
    mpu60X0->config.accel_high_pass_filter = ;
    mpu60X0->config.fifo_enable = ;
    mpu60X0->config.interrupt_mode = ;
    mpu60X0->config.interrupt_enable = ;

    // Finally, enable the device to bring it out of sleep
    mpu60X0_enable(mpu60X0);
    
}

void mpu60X0_teardown(mpu60X0_t* mpu60X0) {
    // Disable interrupts

    // Sleep the device
    mpu60X0_disable(mpu60X0);
}


MPU60X0_Status mpu60X0_read_register(mpu60X0_t* mpu60X0, uint8_t address, uint8_t* data)   {
    
    // Send register to be read with the read mask and reset timeout
    spi_write_byte(mpu60X0->spi, address | MPU60X0_SPI_READ_MASK);
    simple_timer_reset(&mpu60X0->timeout_timer);
    
    // Wait until data available or timed out
    while(!spi1_data_available()) {
        if(simple_timer_has_elapsed(&mpu60X0->timeout_timer)) {
            
            return MPU_TIMED_OUT;
        }
    }

    // Collect and store register contents
    data = spi_read_byte(mpu60X0->spi);
    return MPU_OK;
}
MPU60X0_Status mpu60X0_write_register(mpu60X0_t* mpu60X0, uint8_t address, uint8_t data)   {
    
    // Write one byte with the register address and write mask, then send second byte with data to be written
    spi_write_byte(mpu60X0->spi, address | MPU60X0_SPI_WRITE_MASK);
    spi_write_byte(mpu60X0->spi, data);
    return MPU_OK;

}

uint8_t mpu60X0_get_fifo_count(mpu60X0_t* mpu60X0)  {


}

bool mpu60X0_data_available(mpu60X0_t* mpu60X0)   {
    return spi_data_available(mpu60X0->spi);
}


#endif


#include "CodalConfig.h"
#include "MPU6500.h"
#include "ErrorNo.h"
#include "CodalCompat.h"
#include "CodalFiber.h"

#include "CodalDmesg.h"

using namespace codal;


MPU6500::MPU6500(I2C& _i2c, Pin &_int1, CoordinateSpace &coordinateSpace, uint16_t address,  uint16_t id) : Accelerometer(coordinateSpace, id), i2c(_i2c), int1(_int1)
{
    // Store our identifiers.
    this->id = id;
    this->status = 0;
    this->address = address<<1;

    // Update our internal state for 50Hz at +/- 2g (50Hz has a period af 20ms).
    this->samplePeriod = 20;
    this->sampleRange = 2;

    // Configure and enable the accelerometer.
    configure();
}

int MPU6500::configure()
{
    i2c.writeRegister(address, 0x6B, 0x80);
    fiber_sleep(20);
    i2c.writeRegister(address, 0x6B, 0x00);  /* PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference) */
    
    // Configure Gyro and Thermometer
    // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively; 
    // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
    // be higher than 1 / 0.0059 = 170 Hz
    // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
    i2c.writeRegister(address, 0x1A, 0x03);
    
    i2c.writeRegister(address, 0x1B, 0x18);  /* GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec */
    i2c.writeRegister(address, 0x19, 32);

    i2c.writeRegister(address, 0x37, 0x30); // enable interrupt latch; also enable clear of pin by any read
    i2c.writeRegister(address, 0x38, 0x01); // enable raw data interrupt

    DMESG("MPU6500 init %x", whoAmI());
    return DEVICE_OK;
}

int MPU6500::whoAmI()
{
    uint8_t data;
    int result;
    // the default whoami should return 0x68
    result = i2c.readRegister(address, MPU6500_WHOAMI, &data, 1);
    if (result !=0)
        return 0xffff;

    return (data>>1) & 0x3f;
}

int MPU6500::requestUpdate()
{
    updateSample();
    return DEVICE_OK;
}

int MPU6500::updateSample()
{
    int result;
    uint8_t i2cData[16];

    status |= DEVICE_COMPONENT_STATUS_IDLE_TICK;

    if(int1.getDigitalValue() == 1) {
        result = i2c.readRegister(address, 0x3B, (uint8_t *) i2cData, 14);

        if (result != 0)
            return DEVICE_I2C_ERROR;

        sample.x = ((i2cData[0] << 8) | i2cData[1]);
        sample.y = ((i2cData[2] << 8) | i2cData[3]);
        sample.z = ((i2cData[4] << 8) | i2cData[5]);

        gyro.x = (((i2cData[8] << 8) | i2cData[9]));
        gyro.y = (((i2cData[10] << 8) | i2cData[11]));
        gyro.z = (((i2cData[12] << 8) | i2cData[13]));

        int16_t t = (((i2cData[6] << 8) | i2cData[7]));
        temp = t * 10 / 34 + 3653;

        sample.x /= 16;
        sample.y /= 16;
        sample.z /= 16;

        update(sample);
    }
    return DEVICE_OK;
};

void MPU6500::idleCallback()
{
    requestUpdate();
}

int MPU6500::setSleep(bool sleepMode)
{
    if (sleepMode)
        return i2c.writeRegister(address, 0x6B, 0x40);
    else
        return configure();
}

#ifndef i2c_util_H
#define i2c_util_H

#include <LSM6.h> // IMU
#include <LIS3MDL.h> // Magnetometer
#include <NMEAGPS.h>
#include <NeoSWSerial.h>

// Each slave device will be an i2c_device which provides 2 core functions:
// 1. read()
// 2. publish()
// For specific implementation of these virtual function, we can define classes
// derived from the i2c_device base class.
//
class i2c_device
{
    protected:
        // time (in ms) to wait when requesting data from sensor
        uint16_t timeout;
        // The i2c address of the slave device
        int addr;

        // The i2c Wire library can only read/write a single byte at a time.
        // This is meant to be a wrapper function to read any arbitrary
        // data type.
        int read_bytes(void *data, int bytes_to_read);

    public:
        i2c_device (uint16_t timeout, int addr)
            : timeout(timeout),
              addr(addr) {}

        // The function to read sensor data
        virtual void read() = 0;
        // The function to publish the sensor data via custom_rosserial
        virtual void publish() = 0;
};

class ultrasonic : public i2c_device
{
    private:
        uint16_t distance;
        uint8_t error_code;
        uint8_t msg[4];

    public:
        ultrasonic(uint16_t timeout, int addr)
            : i2c_device(timeout, addr)
        {
            error_code = 0;
        }

        void read();
        void publish();
};

class imu : public i2c_device
{
    private:
        LSM6 imu_data;
        LIS3MDL mag;
        int16_t msg[9];

    public:
        imu(uint16_t timeout, int addr)
            : i2c_device(timeout, addr)
        {
            // initialization of magnatometer and imu
            imu_data.init();
            mag.init();
            imu_data.enableDefault();
            mag.enableDefault();
            imu_data.setTimeout(timeout);
            mag.setTimeout(timeout);
        }

        void read();
        void publish();
};

class gps : public i2c_device
{
    private:
        int32_t msg[5];
        NeoSWSerial *gps_port;
        NMEAGPS *gps_data;

    public:
        gps(uint16_t timeout, int addr, NeoSWSerial *port, NMEAGPS *gps_data)
            : i2c_device(timeout, addr)
        {
            this->gps_port = gps_port;
            this->gps_data = gps_data;
        }

        void read();
        void publish();
};

class propeller : public i2c_device
{
    private:
        int16_t left_pwm;
        int16_t right_pwm;
        int8_t left_enable;
        int8_t right_enable;
        int8_t mode;
        int8_t error_code;
        uint8_t msg[4];

        void read_from_computer();
        void write_to_computer();
        void read_from_propeller_mcu();
        void write_to_propeller_mcu();

    public:
        propeller(uint16_t timeout, int addr)
            : i2c_device(timeout, addr)
        {

        }

        void read();
        void publish();
};

read_from_propeller_mcu()
{
    uint8_t buf[7];
    int recv = i2c_device::read_bytes(buf, sizeof buf);
    left_pwm = buf[0] + (buf[1] << 8);
    right_pwm = buf[2] + (buf[3] << 8);
    left_enable = buf[4];
    right_enable = buf[5];
    mode = buf[6];
    error_code = recv == 0;
}

write_to_propeller_mcu()
{
    Wire.beginTransmission(addr); // transmit to device #8
    Wire.write('A');        // sends five bytes
    Wire.write('B');              // sends one byte
    Wire.write('C');              // sends one byte
    Wire.write('\n');              // sends one byte
    Wire.write('\r');              // sends one byte
    Wire.endTransmission();    // stop transmitting
}

#endif

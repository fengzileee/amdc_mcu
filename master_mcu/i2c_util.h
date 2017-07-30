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

        void read_from_computer();
        void write_to_computer();
        void read_from_propeller_mcu();
        void write_to_propeller_mcu();

    public:

        struct {
            int16_t left_pwm;
            int16_t right_pwm;
            int8_t left_enable;
            int8_t right_enable;
        } cmd, feedback;
        int8_t mode;
        int8_t error_code;
        uint8_t msg[8];

        propeller(uint16_t timeout, int addr)
            : cmd({0,0,0,0}),
              feedback({0,0,0,0}),
              mode(0),
              error_code(0),
              i2c_device(timeout, addr)
        {

        }

        void read();
        void publish();
};

class servo : public i2c_device
{
    private:

        const int LEFT_CLOSE_ANGLE = 60;
        const int LEFT_OPEN_ANGLE = 112;
        const int RIGHT_CLOSE_ANGLE = 170;
        const int RIGHT_OPEN_ANGLE = 112;

    public:

        bool open;
        int16_t left_angle;
        int16_t right_angle;
        int8_t error_code;
        uint8_t msg[4];

        servo(uint16_t timeout, int addr):
          open(false),
          left_angle(LEFT_CLOSE_ANGLE),
          right_angle(RIGHT_CLOSE_ANGLE),
          error_code(0),
          i2c_device(timeout, addr)
        {

        }

        void read();
        void publish();
};

class BufferedSerial
{
    private:
        uint8_t start_idx;
        uint8_t end_idx;
        uint8_t diff;
        // buffer size must be greater than or equal to 256
        uint8_t buffer[256];
    public:
        BufferedSerial():
          start_idx(0),
          end_idx(0),
          diff(0) {}

        int available()
        {
            while (Serial.available())
            {
                buffer[end_idx++] = Serial.read();
                diff++;
            }
            return diff;
        }

        void fetch()
        {
            while (Serial.available())
            {
                buffer[end_idx++] = Serial.read();
                diff++;
            }
        }

        int read()
        {
            diff--;
            return buffer[start_idx++];
        }

        int read(void *buf, int sz)
        {
            uint8_t *c_buf = (uint8_t *) buf;
            int i;
            for (i = 0; i < sz && i < diff; ++i)
                c_buf[i] = buffer[start_idx++];
            diff -= i;
            return i;
        }

};

extern BufferedSerial bufserial;

#endif

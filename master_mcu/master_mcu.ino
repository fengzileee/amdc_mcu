#include <Wire.h>
#include <NeoSWSerial.h>
#include "i2c_util.h"

const int 
ultrasonic_addr[] =  // clockwise from btm left
{ 2,                 // btm left sensor
  3,                 // left sensor
  4,                 // top left sensor
  5,                 // top sensor
  6,                 // top right sensor
  7,                 // right sensor
  8 };               // btm right sensor

// time (in ms) to wait when requesting data from sensor
const uint16_t i2c_timeout = 2;

i2c_device *devices[9];
i2c_device *propeller_dev;

NeoSWSerial gps_port(8, 9);
NMEAGPS gps_data;

BufferedSerial bufserial;

void gps_isr(uint8_t c)
{
    gps_data.handle(c);
}

void setup()
{
    // join I2C bus as master
    Wire.begin();

    Serial.begin(115200);

    for (int i = 0; i < 7; ++i)
    {
        devices[i] = new ultrasonic(i2c_timeout, ultrasonic_addr[i]);
    }

    devices[7] = new imu(i2c_timeout, 0);

    // gps
    gps_port.attachInterrupt(gps_isr);
    gps_port.begin(9600);
    devices[8] = new gps(i2c_timeout, 0, &gps_port, &gps_data);

    propeller_dev = new propeller(i2c_timeout, 9);
}

void loop()
{
    for (auto dev : devices)
    {
        // read sensor data via i2c
        dev->read();
        // publish sensor data to computer
        dev->publish();

        // need to run this in every loop to clear up serial buffer
        propeller_dev->read();
        propeller_dev->publish();
    }
}


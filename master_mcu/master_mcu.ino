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
const uint16_t i2c_timeout = 10;

i2c_device *devices[9];

NeoSWSerial gps_port(8, 9);
NMEAGPS gps_data;

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
}

void handle_callback()
{

}

void loop()
{
    for (auto dev : devices)
    {
        // read sensor data via i2c
        dev->read();
        // publish sensor data via rosserial
        dev->publish();
    }

    // handle callback
    handle_callback();

    // just slowing things down.. not necessary
    delay(10);
}


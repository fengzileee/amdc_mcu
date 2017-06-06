#include <Wire.h>
#include "custom_ros.h"
#include "i2c_util.h"

const struct
{
    int         i2c_addr;
    const char *topic_name;
} ultrasonic_info[] =       // clockwise from left
{{ 2, "u1" },               // left sensor
 { 3, "u2" },               // top left sensor
 { 4, "u3" },               // top right sensor
 { 5, "u4" },               // right sensor
 { 6, "u5" },               // btm right sensor
 { 7, "u6" },               // btm sensor
 { 8, "u7" }};              // btm left sensor

const char *IMU_TOPIC_NAME = "im";

// time (in ms) to wait when requesting data from sensor
const uint16_t i2c_timeout = 10;

ros::NodeHandle nh;

i2c_device *devices[8];

void setup()
{
    // join I2C bus as master
    Wire.begin();

    nh.initNode();

    for (int i = 0; i < 7; ++i)
    {
        devices[i] = new ultrasonic(ultrasonic_info[i].topic_name,
                                    i2c_timeout, 
                                    ultrasonic_info[i].i2c_addr);
        devices[i]->advertise(nh);
    }

    devices[7] = new imu(IMU_TOPIC_NAME, i2c_timeout, 0);
    devices[7]->advertise(nh);

}

void loop()
{
    for (auto dev : devices)
    {
        // read sensor data via i2c
        dev->read();
        // publish sensor data via rosserial
        dev->publish(nh);
    }

    // handle callback
    nh.spinOnce();

    // just slowing things down.. not necessary
    delay(10);
}


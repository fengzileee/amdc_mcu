#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include "i2c_util.h"

void i2c_device::read_bytes(void *data, int bytes_to_read)
{
    Wire.requestFrom(addr, bytes_to_read);

    auto start = millis();
    while (Wire.available() < bytes_to_read)
    {
        if (timeout > 0 && (millis() - start) > timeout)
            return;
    }

    for (int i = 0; i < bytes_to_read; ++i)
    {
        unsigned char *recv = (unsigned char *) data;
        recv[i] = Wire.read();
    }
}

void ultrasonic::read()
{
    i2c_device::read_bytes(&distance, sizeof distance);
}

void ultrasonic::publish(ros::NodeHandle &nh)
{
    msg.header.stamp = nh.now();
    msg.range = (float) distance / 100;
    pub->publish(&msg);
}

void imu::read()
{
    // TODO
}

void imu::publish(ros::NodeHandle &nh)
{
    msg.header.stamp = nh.now();
    // TODO

    pub->publish(&msg);
}

void gps::read()
{
    // TODO
}

void gps::publish(ros::NodeHandle &nh)
{
    // TODO
}

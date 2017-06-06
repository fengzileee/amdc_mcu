#include <Wire.h>
#include "custom_ros.h"
#include "i2c_util.h"

void i2c_device::read_bytes(void *data, int bytes_to_read)
{
    Wire.requestFrom(addr, bytes_to_read);

    uint16_t start = millis();
    while (Wire.available() < bytes_to_read)
    {
        if (timeout > 0 && ((uint16_t) millis() - start) > timeout)
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
    msg.data = distance;
    pub->publish(&msg);
}

void imu::read()
{
    /* msg.data sequence: accelerations of xyz, 
       angular velocities around xyz, 
       magnetometer readings around xyz
    */
    imu_data.read();
    msg.data[0] = imu_data.a.x;
    msg.data[1] = imu_data.a.y;
    msg.data[2] = imu_data.a.z;
    msg.data[3] = imu_data.g.x;
    msg.data[4] = imu_data.g.y;
    msg.data[5] = imu_data.g.z;

    mag.read();
    msg.data[6] = mag.m.x;
    msg.data[7] = mag.m.y;
    msg.data[8] = mag.m.z;
}

void imu::publish(ros::NodeHandle &nh)
{
    pub -> publish(&msg);
}

void gps::read()
{
    if (gps_data.available(port))
    {
        gps_fix fix = gps_data.read();
        msg.data[0] = fix.latitudeL(); // scaled by 10,000,000
        msg.data[1] = fix.longitudeL(); // scaled by 10,000,000
        msg.data[2] = fix.altitude_cm();
        msg.data[3] = 0;
        msg.data[4] = 1;
    }
    else 
    {
        msg.data[3] = -1;
    }
}

void gps::publish(ros::NodeHandle &nh)
{
    pub -> publish(&msg);
}

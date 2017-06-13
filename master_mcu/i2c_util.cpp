#include <Wire.h>
#include "i2c_util.h"

void publish_msg(void *msg, uint8_t sz)
{
    if (sz >= 255)
        return;

    uint8_t lrc = sz + 1;
    uint8_t *buf = (uint8_t *)msg;

    // LRC checksum
    for (int i = 0; i < sz; ++i)
        lrc += buf[i];
    lrc = -lrc;

    Serial.write('A');     // byte 0: header
    Serial.write('z');     // byte 1: header
    Serial.write(sz + 1);  // byte 2: msg size (including checksum)
    for (int i = 0; i < sz; ++i)
        Serial.write(buf[i]);
    Serial.write(lrc);     // byte 3+sz: checksum
                           // total bytes sent: 4 + sz
}

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

void ultrasonic::publish()
{
    msg[0] = addr;
    msg[1] = distance;
    publish_msg(msg, sizeof msg);
}

void imu::read()
{
    /* msg.data sequence: accelerations of xyz, 
       angular velocities around xyz, 
       magnetometer readings around xyz
    */
     imu_data.read();
     msg[0] = imu_data.a.x;
     msg[1] = imu_data.a.y;
     msg[2] = imu_data.a.z;
     msg[3] = imu_data.g.x;
     msg[4] = imu_data.g.y;
     msg[5] = imu_data.g.z;
 
     mag.read();
     msg[6] = mag.m.x;
     msg[7] = mag.m.y;
     msg[8] = mag.m.z;
}

void imu::publish()
{
    publish_msg(msg, sizeof msg);
}

void gps::read()
{
    if (gps_data.available(port))
    {
        gps_fix fix = gps_data.read();
        msg[0] = fix.latitudeL(); // scaled by 10,000,000
        msg[1] = fix.longitudeL(); // scaled by 10,000,000
        msg[2] = fix.altitude_cm();
        msg[3] = 0;
        msg[4] = 1;

    }
    else 
    {
        msg[3] = -1;
    }
}

void gps::publish()
{
    publish_msg(msg, sizeof msg);
}

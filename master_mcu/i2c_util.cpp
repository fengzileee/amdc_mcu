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

int i2c_device::read_bytes(void *data, int bytes_to_read)
{
    Wire.requestFrom(addr, bytes_to_read);

    uint16_t start = millis();
    while (Wire.available() < bytes_to_read)
    {
        if (timeout > 0 && ((uint16_t) millis() - start) > timeout)
            return 0;
    }

    for (int i = 0; i < bytes_to_read; ++i)
    {
        unsigned char *recv = (unsigned char *) data;
        recv[i] = Wire.read();
    }
    return bytes_to_read;
}

void ultrasonic::read()
{
    uint8_t buf[3];
    int recv = i2c_device::read_bytes(buf, sizeof buf);
    distance = buf[0] + (buf[1] << 8);
    error_code = recv != 0 ? buf[2] : 5;
}

void ultrasonic::publish()
{
    msg[0] = addr;
    msg[1] = distance & 0xff;
    msg[2] = (distance >> 8) & 0xff;
    msg[3] = error_code;
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
    if (gps_data->available(*gps_port))
    {
        gps_fix fix = gps_data->read();
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


void propeller::read_from_computer()
{
    uint8_t buf[6];
    uint8_t lrc = 0;

    if (bufserial.available() >= 8)
    {
        if (bufserial.read() != 'A')
        {
            // message not received in correct sequence
            error_code = 2;
            return;
        }

        bufserial.read(buf, sizeof buf);
        for (int i = 0; i < sizeof buf; ++i)
        {
            lrc += buf[i];
        }
        lrc = -lrc;

        if (lrc != bufserial.read())
        {
            // incorrect checksum
            error_code = 4;
            return;
        }

        cmd.left_pwm = buf[0];
        cmd.left_pwm += buf[1] << 8;

        cmd.right_pwm = buf[2];
        cmd.right_pwm += buf[3] << 8;

        cmd.left_enable = buf[4];
        cmd.right_enable = buf[5];
        
        error_code = 0;
    }
    else if (bufserial.available() > 0)
    {
        // not receiving full message
        error_code = 6;
    }
}

void propeller::write_to_computer()
{
    msg[0] = feedback.left_pwm & 0xff;
    msg[1] = (feedback.left_pwm >> 8) & 0xff;
    msg[2] = feedback.right_pwm & 0xff;
    msg[3] = (feedback.right_pwm >> 8) & 0xff;
    msg[4] = feedback.left_enable;
    msg[5] = feedback.right_enable;
    msg[6] = mode;
    msg[7] = error_code;
    publish_msg(msg, sizeof msg);
}

void propeller::read_from_propeller_mcu()
{
    uint8_t buf[7];
    int recv = i2c_device::read_bytes(buf, sizeof buf);
    if (recv == sizeof buf)
    {
        feedback.left_pwm = buf[0] + (buf[1] << 8);
        feedback.right_pwm = buf[2] + (buf[3] << 8);
        feedback.left_enable = buf[4];
        feedback.right_enable = buf[5];
        mode = buf[6];
        error_code &= 0xfe;
    }
    else
    {
        error_code |= 1;
    }
}

void propeller::write_to_propeller_mcu()
{
    Wire.beginTransmission(addr);
    Wire.write(cmd.left_pwm & 0xff);         // left pwm lower byte
    Wire.write((cmd.left_pwm >> 8) & 0xff);  // left pwm upper byte
    Wire.write(cmd.right_pwm & 0xff);        // right pwm lower byte
    Wire.write((cmd.right_pwm >> 8) & 0xff); // right pwm upper byte
    Wire.write(cmd.left_enable);
    Wire.write(cmd.right_enable);
    Wire.endTransmission();
}

void propeller::read()
{
    read_from_computer();
    write_to_propeller_mcu();
}

void propeller::publish()
{
    read_from_propeller_mcu();
    write_to_computer();
}

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
int dev_counter = 0;

propeller *propeller_dev;
servo *servo_dev;

NeoSWSerial gps_port(8, 9);
NMEAGPS gps_data;

BufferedSerial bufserial;

void gps_isr(uint8_t c)
{
    gps_data.handle(c);
}

void add_device(i2c_device *dev)
{
    devices[dev_counter++] = dev;
}

void get_actuator_command()
{
    uint8_t buf[9];
    uint8_t lrc = 0;

    if (bufserial.available() < 11)
    {
        // this can happen often if the arduino update frequency is
        // faster than the computer
        propeller_dev->error_code = 6;
        return;
    }

    // check header
    if (bufserial.read() != 'A')
    {
        propeller_dev->error_code = 2;
        return;
    }

    // checksum
    bufserial.read(buf, sizeof buf);
    for (int i = 0; i < sizeof buf; ++i)
    {
        lrc += buf[i];
    }
    lrc = -lrc;
    if (lrc != bufserial.read())
    {
        propeller_dev->error_code = 4;
        return;
    }

    // update actuator commands
    bool update_propeller = buf[6];
    bool update_servo = buf[8];

    propeller_dev->error_code = 0;
    if (update_propeller)
    {
        propeller_dev->cmd.left_pwm = buf[0];
        propeller_dev->cmd.left_pwm += buf[1] << 8;
        propeller_dev->cmd.right_pwm = buf[2];
        propeller_dev->cmd.right_pwm += buf[3] << 8;
        propeller_dev->cmd.left_enable = buf[4];
        propeller_dev->cmd.right_enable = buf[5];
    }

    if (update_servo)
    {
        servo_dev->open = buf[7];
    }
}

void setup()
{
    // join I2C bus as master
    Wire.begin();

    Serial.begin(115200);

    for (int i = 0; i < 7; ++i)
    {
        add_device(new ultrasonic(i2c_timeout, ultrasonic_addr[i]));
    }

    add_device(new imu(i2c_timeout, 0));

    // gps
    gps_port.attachInterrupt(gps_isr);
    gps_port.begin(9600);
    add_device(new gps(i2c_timeout, 0, &gps_port, &gps_data));

    propeller_dev = new propeller(i2c_timeout, 9);
    servo_dev = new servo(i2c_timeout, 10);
}

void loop()
{
    for (int i = 0; i < dev_counter; ++i)
    {
        // read sensor data via i2c
        devices[i]->read();
        // publish sensor data to computer
        devices[i]->publish();

        // need to run this in every loop to clear up serial buffer
        get_actuator_command();
        propeller_dev->read();
        propeller_dev->publish();
        servo_dev->publish();
    }
}


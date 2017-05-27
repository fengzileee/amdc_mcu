#include <Wire.h>
#include <ros.h>
#include "i2c_util.h"

ros::NodeHandle nh;

i2c_device *devices[2];

void setup()
{
    // join I2C bus as device #1
    Wire.begin();

    nh.initNode();
    devices[0] = new ultrasonic("u1_pub", 50, 8, "base_frame");
    devices[1] = new ultrasonic("u2_pub", 50, 7, "asd_frame");
    devices[0]->advertise(nh);
    devices[1]->advertise(nh);
}

void loop()
{
    for (auto dev : devices)
    {
        dev->read();
        dev->publish(nh);
    }

    nh.spinOnce();
    delay(50);
}


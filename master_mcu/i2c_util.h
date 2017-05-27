#ifndef i2c_util_H
#define i2c_util_H

#include <ros.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>

class i2c_device
{
    protected:
        ros::Publisher *pub;
        const char *topic_name;
        int timeout;
        int addr;

        void read_bytes(void *, int);

    public:
        i2c_device (const char *topic_name, int timeout, int addr)
            : topic_name(topic_name), 
              timeout(timeout),
              addr(addr) {}
        virtual void read() = 0;
        virtual void publish(ros::NodeHandle &nh) = 0;
        void advertise(ros::NodeHandle &nh)
        {
            nh.advertise(*pub);
        }
};

class ultrasonic : public i2c_device
{
    private:
        sensor_msgs::Range msg;
        uint8_t distance;

    public:
        ultrasonic(const char *topic_name, int timeout, int addr, const char *frame_id)
            : i2c_device(topic_name, timeout, addr)
        {
            pub = new ros::Publisher(topic_name, &msg);

            msg.header.frame_id = frame_id;
            msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
            msg.field_of_view = 0.2;
            msg.min_range = 0.05;
            msg.max_range = 7.00;
        }
        void read();
        void publish(ros::NodeHandle &);
};

class imu : public i2c_device
{
    private:
        sensor_msgs::Imu msg;
        // TODO
        //LSM6 imu_data;
        //LIS3MDL mag;
        //LPS ps;

        //https://github.com/pololu/lsm6-arduino/blob/master/examples/Serial/Serial.ino
        //https://github.com/pololu/lis3mdl-arduino/blob/master/examples/Serial/Serial.ino
        //https://github.com/pololu/lps-arduino/blob/master/examples/SerialMetric/SerialMetric.ino

    public:
        imu(const char *topic_name, int timeout, int addr, const char *frame_id)
            : i2c_device(topic_name, timeout, addr)
        {
            pub = new ros::Publisher(topic_name, &msg);

            msg.header.frame_id = frame_id;
        }
        void read();
        void publish(ros::NodeHandle &);
};

class gps : public i2c_device
{
    // TODO
    private:

    public:
        gps(const char *topic_name, int timeout, int addr)
            : i2c_device(topic_name, timeout, addr)
        {
            //pub = new ros::Publisher(topic_name, &msg);

            //msg.header.frame_id = frame_id;
        }
        void read();
        void publish(ros::NodeHandle &);
};

#endif

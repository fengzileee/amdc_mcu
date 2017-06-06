#ifndef i2c_util_H
#define i2c_util_H

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <LSM6.h> // IMU
#include <LIS3MDL.h> // Magnetometer
#include <std_msgs/Int32MultiArray.h>
#include <NMEAGPS.h>
#include <AltSoftSerial.h>

// Each slave device will be an i2c_device which provides 2 core functions:
// 1. read()
// 2. publish()
// For specific implementation of these virtual function, we can define classes
// derived from the i2c_device base class.
//
class i2c_device
{
    protected:
        ros::Publisher *pub;
        const char *topic_name;
        // time (in ms) to wait when requesting data from sensor
        uint16_t timeout;
        // The i2c address of the slave device
        int addr;

        // The i2c Wire library can only read/write a single byte at a time.
        // This is meant to be a wrapper function to read any arbitrary
        // data type.
        void read_bytes(void *data, int bytes_to_read);

    public:
        i2c_device (const char *topic_name, uint16_t timeout, int addr)
            : topic_name(topic_name), 
              timeout(timeout),
              addr(addr) {}

        // The function to read sensor data via the i2c bus
        virtual void read() = 0;
        // The function to publish the sensor data via rosserial
        virtual void publish(ros::NodeHandle &nh) = 0;
        void advertise(ros::NodeHandle &nh)
        {
            nh.advertise(*pub);
        }
};

class ultrasonic : public i2c_device
{
    private:
        std_msgs::Int16 msg;
        uint16_t distance;

    public:
        ultrasonic(const char *topic_name, uint16_t timeout, int addr)
            : i2c_device(topic_name, timeout, addr)
        {
            pub = new ros::Publisher(topic_name, &msg);
        }

        void read();
        void publish(ros::NodeHandle &);
};

class imu : public i2c_device
{
    private:
        // TODO
        LSM6 imu_data;
        LIS3MDL mag;
        //LPS ps;
        std_msgs::Int16MultiArray msg;

        //https://github.com/pololu/lsm6-arduino/blob/master/examples/Serial/Serial.ino
        //https://github.com/pololu/lis3mdl-arduino/blob/master/examples/Serial/Serial.ino
        //https://github.com/pololu/lps-arduino/blob/master/examples/SerialMetric/SerialMetric.ino

    public:
        imu(const char *topic_name, uint16_t timeout, int addr)
            : i2c_device(topic_name, timeout, addr)
        {
            // initialization of msg
            msg.layout.dim = (std_msgs::MultiArrayDimension *)
            malloc(sizeof(std_msgs::MultiArrayDimension));
            msg.layout.dim[0].label = "height";
            msg.layout.dim[0].size = 9;
            msg.layout.dim[0].stride = 1; 
            msg.layout.data_offset = 0;
            msg.data = (int *)malloc(sizeof(int)*9);
            msg.data_length = 9;

            // initialization of magnatometer and imu
            imu_data.init();
            mag.init();
            imu_data.enableDefault();
            mag.enableDefault();

            // ROS publisher
            pub = new ros::Publisher(topic_name, &msg);
        }

        void read();
        void publish(ros::NodeHandle &);
};

class gps : public i2c_device
{
    private:
        AltSoftSerial port;
        NMEAGPS gps_data;
        std_msgs::Int32MultiArray msg;

    public:
        gps(const char *topic_name, uint16_t timeout, int addr, AltSoftSerial gps_port)
            : i2c_device(topic_name, timeout, addr)
        {
            // ROS publisher
            pub = new ros::Publisher(topic_name, &msg);

            // Soft Serial port for gps
            port = gps_port;
            port.begin(9600); // default baudrate of our GPS device

            // initialize ROS msg
            // latitude, longitude, altitude, status, service
            msg.layout.dim = (std_msgs::MultiArrayDimension *) 
                malloc(sizeof(std_msgs::MultiArrayDimension));
            msg.layout.dim[0].label = "height"; 
            msg.layout.dim[0].size = 5;
            msg.layout.dim[0].stride = 1;
            msg.layout.data_offset = 0; 
            msg.data = (int32_t *) malloc(sizeof(int32_t)*5);
            msg.data_length = 5;
        }

        void read();
        void publish(ros::NodeHandle &);
};

#endif

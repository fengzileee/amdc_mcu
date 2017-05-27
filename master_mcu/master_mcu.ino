#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Range.h>

ros::NodeHandle nh;

sensor_msgs::Range ultrasonic_msg;
ros::Publisher ultrasonic_publisher("ultrasonic_range", &ultrasonic_msg);

namespace sensor_type
{
    typedef int ultrasonic;

    struct IMU
    {
        float orientation_covariance;
        float angular_velocity_covariance;
        float linear_acceleration_covariance;
        struct 
        {
            float x;
            float y;
            float z;
            float w;
        } quaternion;
        struct 
        {
            float x;
            float y;
            float z;
        } angular_velocity;
        struct 
        {
            float x;
            float y;
            float z;
        } linear_acceleration;
    };

    struct GPS
    {
        float lat;
        float lng;
    };
}

float dist;

//class Device
//{
    //int id;
    //void *data;
    //void publish();
    //void read();
//};

int device_type = 1;
int bytes_to_read[] = {1, 2, 3};
void *data;

void read()
{
    int total_bytes = bytes_to_read[device_type];
    while (Wire.available() < total_bytes)
    {
    
    }

    for (int i = 0; i < total_bytes; ++i)
    {
        unsigned char *recv = (unsigned char *) data;
        recv[i] = Wire.read();
    }
}

void setup()
{
    // join I2C bus as device #1
    Wire.begin();

    nh.initNode();
    nh.advertise(ultrasonic_publisher);
    ultrasonic_msg.header.frame_id = "base_frame";
    ultrasonic_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    ultrasonic_msg.field_of_view = 0.2;
    ultrasonic_msg.min_range = 0.05;
    ultrasonic_msg.max_range = 7.00;
}

void loop()
{
    // TODO 
    // make request into a single function?
    // maybe define a protocol, eg:
    // int data[] = {device_id, value}
    Wire.requestFrom(8, 1); // request int data from device #8
    if (Wire.available())
    {
        dist = Wire.read(); // dist in cm
    }

    ultrasonic_msg.header.stamp = nh.now();
    ultrasonic_msg.range = dist / 100;
    ultrasonic_publisher.publish(&ultrasonic_msg);

    nh.spinOnce();
    delay(50);
}


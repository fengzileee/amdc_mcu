#include <Wire.h>

unsigned char dist = 240;

void callback()
{
    //Wire.beginTransmission(1);
    Wire.write(dist);
    //Wire.endTransmission();
}

void setup()
{
    // join I2C bus as device #8
    Wire.begin(8);
    Wire.onRequest(callback);
}

void loop()
{

}


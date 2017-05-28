// code adapted from:
// http://www.pcserviceselectronics.co.uk/arduino/Ultrasonic/betterecho.php
#include <Wire.h>

#define ECHO_PIN 4
#define TRIGGER_PIN 5

// ratio based on speed of sound
const unsigned long TIME_DISTANCE_RATIO = 58;
// default distance (in case of error)
const uint16_t DEFAULT_DIST = 0;

// max time to wait for echo pin to become high
const unsigned long START_ECHO_TIMEOUT = 1000;
// max distance that we care about (longer distance means we need to
// wait longer for sensor to reply)
const unsigned long MAX_DISTANCE = 600;
const unsigned long MAX_ECHO_LEN = TIME_DISTANCE_RATIO * MAX_DISTANCE;

// distance in cm, default to 0
volatile uint16_t dist = DEFAULT_DIST;

// used for creating delay for each reading interval
unsigned long time, next_time;
// time between each sensor reading (in ms)
const unsigned long INTERVAL = 30;

void callback()
{
    for (int i = 0; i < sizeof dist; ++i)
    {
        unsigned char *data = (unsigned char *) &dist;
        Wire.write(data[i]);
    }
}

uint16_t get_proximity()
{
    unsigned long start, end, duration;

    // error if echo pin is high before we even set it
    if (digitalRead(ECHO_PIN) == HIGH)
        return DEFAULT_DIST;

    start = micros();

    // send 10us trigger pulse
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    // error if echo pin remains low after timeout
    end = start + START_ECHO_TIMEOUT;
    while (digitalRead(ECHO_PIN) == LOW)
        if (micros() > end)
            return DEFAULT_DIST;

    // measure length of echo (capped to MAX_ECHO_LEN)
    start = micros();
    end = start + MAX_ECHO_LEN;
    while (digitalRead(ECHO_PIN) == HIGH)
        if (micros() > end)
            break;
    end = micros();

    // set distance to default in case end < start
    duration = end - start;
    return end > start ? duration/TIME_DISTANCE_RATIO : DEFAULT_DIST;
}

void setup()
{
    // setup pins
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);

    // join I2C bus as device #8
    Wire.begin(8);
    Wire.onRequest(callback);
}

void loop()
{
    time = millis();
    if (time >= next_time)
    {
        dist = get_proximity();
        next_time = time + INTERVAL;
    }
}


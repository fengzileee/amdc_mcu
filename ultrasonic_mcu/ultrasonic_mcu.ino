// code adapted from:
// http://www.pcserviceselectronics.co.uk/arduino/Ultrasonic/betterecho.php
#include <Wire.h>

#define handle_error(N) \
do {                    \
    error_code = (N);   \
    return;             \
} while (0);

#define ECHO_PIN 5
#define TRIGGER_PIN 4

// ratio based on speed of sound
const unsigned long TIME_DISTANCE_RATIO = 58;
// default distance (in case of error)
const uint16_t DEFAULT_DIST = 500;

// max time to wait for echo pin to become high
const unsigned long START_ECHO_TIMEOUT = 1000;
// max distance that we care about (longer distance means we need to
// wait longer for sensor to reply)
const unsigned long MAX_DISTANCE = 500;
const unsigned long MAX_ECHO_LEN = TIME_DISTANCE_RATIO * MAX_DISTANCE;

// distance in cm, default to 0
volatile uint16_t dist = DEFAULT_DIST;
volatile uint8_t error_code = 0;

// used for creating delay for each reading interval
unsigned long time, next_time;
// time between each sensor reading (in ms)
const unsigned long INTERVAL = 100;

#ifndef I2C_ADDRESS
#define I2C_ADDRESS 2
#endif

void callback()
{
    unsigned char *data = (unsigned char *) &dist;
    Wire.write(data[0]);
    Wire.write(data[1]);
    Wire.write(error_code);
}

void get_proximity()
{
    unsigned long start, end, duration;

    // error if echo pin is high before we even set it
    if (digitalRead(ECHO_PIN) == HIGH)
        handle_error(1);

    start = micros();

    // send 10us trigger pulse
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    // error if echo pin remains low after timeout
    end = start + START_ECHO_TIMEOUT;
    while (digitalRead(ECHO_PIN) == LOW)
        if (micros() > end)
            handle_error(2);

    // measure length of echo (capped to MAX_ECHO_LEN)
    start = micros();
    end = start + MAX_ECHO_LEN;
    while (digitalRead(ECHO_PIN) == HIGH)
        if (micros() > end)
            handle_error(3);
    end = micros();

    // set distance to default in case end < start
    duration = end - start;
    if (end < start)
        handle_error(4);

    error_code = 0;
    dist = duration/TIME_DISTANCE_RATIO;
}

void setup()
{
    // setup pins
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);

    // join I2C bus as device #8
    Wire.begin(I2C_ADDRESS);
    Wire.onRequest(callback);
}

void loop()
{
    time = millis();
    if (time >= next_time)
    {
        get_proximity();
        next_time = time + INTERVAL;
    }
}


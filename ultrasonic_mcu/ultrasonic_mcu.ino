// code adapted from:
// http://www.pcserviceselectronics.co.uk/arduino/Ultrasonic/betterecho.php
#include <Wire.h>

#define ECHO_PIN 5
#define TRIGGER_PIN 4

// ratio based on speed of sound
const unsigned long TIME_DISTANCE_RATIO = 58;

// max time to wait for echo pin to become high
const unsigned long START_ECHO_TIMEOUT = 1000;
// MAX_DISTANCE corresponse to how long we are willing to wait for the
// sensor to read the bounced back wave. If we set this too low, then
// the echo pin may remain high in the next interval (unless we set a
// relatively large interval like 200ms)
const unsigned long MAX_DISTANCE = 5000;
const unsigned long MAX_ECHO_LEN = TIME_DISTANCE_RATIO * MAX_DISTANCE;
const uint16_t DISTANCE_CAP = 600;

// distance in cm, default to 600
volatile uint16_t dist = DISTANCE_CAP;
volatile uint8_t error_code = 0;

volatile uint16_t old_dist = DISTANCE_CAP;
volatile uint8_t old_error_code = 0;

// used for creating delay for each reading interval
unsigned long time, next_time;
// time between each sensor reading (in ms)
// We use 200ms because experiment shows that the echo pin can be high
// for roughly 190ms.
const unsigned long INTERVAL = 200;

// Lock to ensure atomicity of get_proximity (since I2C callback is
// handled by interrupt)
volatile int lock = 0;
#define LOCKED   1
#define RELEASED 0

#ifndef I2C_ADDRESS
#define I2C_ADDRESS 2
#endif

// error handling
#define  NO_ERROR             0
#define  ECHO_HIGH_B4_SET     1
#define  ECHO_REMAINS_LOW     2
#define  ECHO_EXCEED_MAX      3
#define  END_LESS_THAN_START  4

#define handle_error_default(N) \
do {                            \
    error_code = (N);           \
    dist = DISTANCE_CAP;        \
    lock = RELEASED;            \
    return;                     \
} while (0);

#define handle_error_prev(N)    \
do {                            \
    error_code = (N);           \
    lock = RELEASED;            \
    return;                     \
} while (0);

void callback()
{
    unsigned char *data = (unsigned char *) &dist;
    uint8_t *error = (uint8_t *) &error_code;

    if (lock == LOCKED)
    {
        data = (unsigned char *) &old_dist;
        error = (uint8_t *) &old_error_code;
    }

    Wire.write(data[0]);
    Wire.write(data[1]);
    Wire.write(*error);
}

void get_proximity()
{
    unsigned long start, end, duration;

    lock = LOCKED;

    // error if echo pin is high before we even set it
    // Possible reason is that the echo pin is still high from previous
    // reading, suggesting that the range is large.
    if (digitalRead(ECHO_PIN) == HIGH)
        handle_error_default(ECHO_HIGH_B4_SET);

    start = micros();

    // send 10us trigger pulse
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    // error if echo pin remains low after timeout
    end = start + START_ECHO_TIMEOUT;
    while (digitalRead(ECHO_PIN) == LOW)
        if (micros() > end)
            handle_error_prev(ECHO_REMAINS_LOW);

    // measure length of echo (capped to MAX_ECHO_LEN)
    // Length greater than MAX_ECHO_LEN means range is large.
    start = micros();
    end = start + MAX_ECHO_LEN;
    while (digitalRead(ECHO_PIN) == HIGH)
        if (micros() > end)
            handle_error_default(ECHO_EXCEED_MAX);
    end = micros();

    // set distance to default in case end < start
    // Possible due to integer overflow.
    duration = end - start;
    if (end < start)
        handle_error_prev(END_LESS_THAN_START);

    error_code = NO_ERROR;
    dist = duration/TIME_DISTANCE_RATIO;
    dist = dist <= DISTANCE_CAP ? dist : DISTANCE_CAP;

    lock = RELEASED;
}

void setup()
{
    // setup pins
    pinMode(TRIGGER_PIN, OUTPUT);
    digitalWrite(TRIGGER_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);

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


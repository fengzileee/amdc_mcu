#include <Wire.h>
#include <AltSoftSerial.h>

#define PWM_LEFT_FORWARD  5
#define PWM_LEFT_REVERSE  6
#define ENABLE_LEFT       7
#define PWM_RIGHT_FORWARD 3
#define PWM_RIGHT_REVERSE 11
#define ENABLE_RIGHT      12

// For bluetooth control of individual propeller
#define LEFT_FORWARD  'w'
#define LEFT_REVERSE  's'
#define LEFT_TOGGLE   'x' // toggle between enable/disable
#define RIGHT_FORWARD 'e'
#define RIGHT_REVERSE 'd'
#define RIGHT_TOGGLE  'c' // toggle between enable/disable
#define INTERVAL 10

#define MAX_SPD_LIMIT 230
#define MIN_SPD_LIMIT -230

// TODO
// 1. Remove Serial and add I2C support

int mode;

// Range of speed is [-230, 230]
volatile int left_spd;
volatile int right_spd;

volatile int left_enable;
volatile int right_enable;

// Range of pwm control is [0, 230]
// Forward is activated when speed is positive
// Reverse is activated when speed is negative
int left_forward_pwm_control;
int left_reverse_pwm_control;
int right_forward_pwm_control;
int right_reverse_pwm_control;

volatile int update;

// AltSoftSerial always uses these pins:
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Arduino Uno        9         8         10
AltSoftSerial bt_serial;

void receive_callback(int c)
{
    if (Wire.available() == 6)
    {
        left_spd = Wire.read();
        left_spd += Wire.read() << 8;
        right_spd = Wire.read();
        right_spd += Wire.read() << 8;
        left_enable = Wire.read();
        right_enable = Wire.read();
        update = 1;
    }
}

void request_callback()
{
    Wire.write(left_spd & 0xff);         // left spd lower byte
    Wire.write((left_spd >> 8) & 0xff);  // left spd upper byte
    Wire.write(right_spd & 0xff);        // right spd lower byte
    Wire.write((right_spd >> 8) & 0xff); // right spd upper byte
    Wire.write(left_enable);
    Wire.write(right_enable);
    Wire.write(mode);
}

void setup()
{
    // Initialise everything and set to 0 speed
    pinMode(ENABLE_LEFT, OUTPUT);
    pinMode(ENABLE_RIGHT, OUTPUT);
    pinMode(PWM_LEFT_FORWARD, OUTPUT);
    pinMode(PWM_LEFT_REVERSE, OUTPUT);
    pinMode(PWM_RIGHT_FORWARD, OUTPUT);
    pinMode(PWM_RIGHT_REVERSE, OUTPUT);

    digitalWrite(PWM_LEFT_FORWARD, LOW);
    digitalWrite(PWM_LEFT_REVERSE, LOW);
    digitalWrite(PWM_RIGHT_FORWARD, LOW);
    digitalWrite(PWM_RIGHT_FORWARD, LOW);

    digitalWrite(ENABLE_LEFT, HIGH);
    digitalWrite(ENABLE_RIGHT, HIGH);

    bt_serial.begin(9600);
    Wire.begin(9);
    Wire.onRequest(request_callback);
    Wire.onReceive(receive_callback);

#ifdef DEBUG
    Serial.begin(9600);
#endif
}

void loop()
{
    while (bt_serial.available())
    {
        int recv = bt_serial.read();
        switch (recv)
        {
        case LEFT_FORWARD:
            left_spd += INTERVAL;
            left_spd = left_spd >= MAX_SPD_LIMIT ? MAX_SPD_LIMIT : left_spd;
            break;
        case LEFT_REVERSE:
            left_spd -= INTERVAL;
            left_spd = left_spd <= MIN_SPD_LIMIT ? MIN_SPD_LIMIT : left_spd;
            break;
        case LEFT_TOGGLE:
            left_enable = !left_enable;
            break;
        case RIGHT_FORWARD:
            right_spd += INTERVAL;
            right_spd = right_spd >= MAX_SPD_LIMIT ? MAX_SPD_LIMIT : right_spd;
            break;
        case RIGHT_REVERSE:
            right_spd -= INTERVAL;
            right_spd = right_spd <= MIN_SPD_LIMIT ? MIN_SPD_LIMIT : right_spd;
            break;
        case RIGHT_TOGGLE:
            right_enable = !right_enable;
            break;
        }

        mode = 1;
        update = 1;
    }

    if (update == 1)
    {
        left_forward_pwm_control = left_spd > 0 ? left_spd : 0;
        left_reverse_pwm_control = left_spd < 0 ? -left_spd : 0;
        right_forward_pwm_control = right_spd > 0 ? right_spd : 0;
        right_reverse_pwm_control = right_spd < 0 ? -right_spd : 0;

        analogWrite(PWM_LEFT_FORWARD, left_forward_pwm_control);
        analogWrite(PWM_LEFT_REVERSE, left_reverse_pwm_control);
        analogWrite(PWM_RIGHT_FORWARD, right_forward_pwm_control);
        analogWrite(PWM_RIGHT_REVERSE, right_reverse_pwm_control);

        digitalWrite(ENABLE_LEFT, left_enable);
        digitalWrite(ENABLE_RIGHT, right_enable);

        update = 0;
    
#ifdef DEBUG
        Serial.print("left propeller ");
        Serial.println(left_spd);
        Serial.print("left_forward_pwm_control ");
        Serial.println(left_forward_pwm_control);
        Serial.print("left_reverse_pwm_control ");
        Serial.println(left_reverse_pwm_control);
        
        Serial.print("right propeller ");
        Serial.println(right_spd);
        Serial.print("right_forward_pwm_control ");
        Serial.println(right_forward_pwm_control);
        Serial.print("right_reverse_pwm_control ");
        Serial.println(right_reverse_pwm_control);
        delay(100);
#endif
    }
}


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

// Range of speed is [-230, 230]
int left_spd;
int right_spd;

// Range of pwm control is [0, 230]
// Forward is activated when speed is positive
// Reverse is activated when speed is negative
int left_forward_pwm_control;
int left_reverse_pwm_control;
int right_forward_pwm_control;
int right_reverse_pwm_control;

int left_enable = 1;
int right_enable = 1;

// AltSoftSerial always uses these pins:
//
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Teensy 3.0 & 3.1  21        20         22
// Teensy 2.0         9        10       (none)
// Teensy++ 2.0      25         4       26, 27
// Arduino Uno        9         8         10
// Arduino Leonardo   5        13       (none)
// Arduino Mega      46        48       44, 45
// Wiring-S           5         6          4
// Sanguino          13        14         12
AltSoftSerial bt_serial;
//HardwareSerial &bt_serial = Serial;

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

#ifdef DEBUG
    Serial.begin(9600);
#endif
}

void loop()
{
    if (bt_serial.available() >= 8)
    {
        if (bt_serial.read() != 'A') // check header
        {
#ifdef DEBUG
            Serial.println("Wrong header");
#endif
            return;
        }
        left_forward_pwm_control = bt_serial.read();
        left_reverse_pwm_control = bt_serial.read();
        right_forward_pwm_control = bt_serial.read();
        right_reverse_pwm_control = bt_serial.read();
        left_enable = bt_serial.read();
        right_enable = bt_serial.read();
        int recv_lrc = bt_serial.read();

        uint8_t lrc = left_forward_pwm_control +
                      left_reverse_pwm_control +
                      right_forward_pwm_control +
                      right_reverse_pwm_control +
                      left_enable +
                      right_enable;
        lrc = -lrc;
        if (lrc != recv_lrc)
        {
#ifdef DEBUG
            Serial.println("wrong checksum");
#endif
            return;
        }

        analogWrite(PWM_LEFT_FORWARD, left_forward_pwm_control);
        analogWrite(PWM_LEFT_REVERSE, left_reverse_pwm_control);
        analogWrite(PWM_RIGHT_FORWARD, right_forward_pwm_control);
        analogWrite(PWM_RIGHT_REVERSE, right_reverse_pwm_control);
        digitalWrite(ENABLE_LEFT, left_enable);
        digitalWrite(ENABLE_RIGHT, right_enable);

#ifdef DEBUG
        Serial.print("left_forward_pwm_control ");
        Serial.println(left_forward_pwm_control);
        Serial.print("left_reverse_pwm_control ");
        Serial.println(left_reverse_pwm_control);
        Serial.print("left_enable ");
        Serial.println(left_enable);
        
        Serial.print("right_forward_pwm_control ");
        Serial.println(right_forward_pwm_control);
        Serial.print("right_reverse_pwm_control ");
        Serial.println(right_reverse_pwm_control);
        Serial.print("right_enable ");
        Serial.println(right_enable);
#endif
    }

    delay(1);
}


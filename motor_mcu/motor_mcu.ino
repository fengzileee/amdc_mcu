#include <Wire.h>
#include <AltSoftSerial.h>

#define PWM_LEFT_FORWARD  5
#define PWM_LEFT_REVERSE  6
#define ENABLE_LEFT       7
#define PWM_RIGHT_FORWARD 3
#define PWM_RIGHT_REVERSE 11
#define ENABLE_RIGHT      12

#define MAX_SPD_LIMIT 230
#define MIN_SPD_LIMIT -230

#ifdef DEBUG
  #define debug_println(X) Serial.println(X);
  #define debug_print(X) Serial.print(X);
#else
  #define debug_println(X)
  #define debug_print(X)
#endif

// AltSoftSerial always uses these pins:
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Arduino Uno        9         8         10
AltSoftSerial bt_serial;

// Range of speed is [-230, 230]
volatile int current_left_spd;
volatile int target_left_spd;
volatile int current_right_spd;
volatile int target_right_spd;

int left_delta;
int right_delta;

volatile int left_enable;
volatile int right_enable;

// 1 means controlled via bluetooth
// 0 means controlled from computer
volatile int mode;

// Range of pwm control is [0, 230]
// Forward is activated when speed is positive
// Reverse is activated when speed is negative
int left_forward_pwm_control;
int left_reverse_pwm_control;
int right_forward_pwm_control;
int right_reverse_pwm_control;

// variables to implement smooth transition
unsigned long time, next_time;
const unsigned long INTERVAL = 10;

void receive_callback(int c)
{
    uint8_t buf[6];
    if (Wire.available() == sizeof buf)
    {
        for (int i = 0; i < sizeof buf; ++i)
            buf[i] = Wire.read();

        // don't update using motor command from computer if it is 
        // currently controlled via bluetooth
        if (mode == 0)
        {
            target_left_spd = buf[0];
            target_left_spd += buf[1] << 8;
            target_right_spd = buf[2];
            target_right_spd += buf[3] << 8;
            left_enable = buf[4];
            right_enable = buf[5];
        }
    }
}

void request_callback()
{
    Wire.write(current_left_spd & 0xff);         // left spd lower byte
    Wire.write((current_left_spd >> 8) & 0xff);  // left spd upper byte
    Wire.write(current_right_spd & 0xff);        // right spd lower byte
    Wire.write((current_right_spd >> 8) & 0xff); // right spd upper byte
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

    digitalWrite(ENABLE_LEFT, LOW);
    digitalWrite(ENABLE_RIGHT, LOW);

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
    uint8_t buf[6];
    uint8_t lrc = 0;

    time = millis();

    if (bt_serial.available() >= sizeof buf + 2)
    {
        if (bt_serial.read() != 'A') // check header
        {
            // TODO report error to master mcu
            debug_println("Wrong header");
            return;
        }

        for (int i = 0; i < sizeof buf - 1; ++i)
        {
            buf[i] = bt_serial.read();
            lrc += buf[i];
        }
        lrc = -lrc;

        if (lrc != bt_serial.read())
        {
            debug_println("wrong checksum");
            return;
        }

        target_left_spd = buf[0] - buf[1];
        target_right_spd = buf[2] - buf[3];
        left_enable = buf[4];
        right_enable = buf[5];

        // TODO allow remote controller to set mode back to auto
        mode = 1;
    }

    digitalWrite(ENABLE_LEFT, left_enable);
    digitalWrite(ENABLE_RIGHT, right_enable);

    if (left_enable == 1 && current_left_spd != target_left_spd)
    {
        left_delta = target_left_spd > current_left_spd ? 1 : -1;
        if (time >= next_time)
            current_left_spd += left_delta;
    }
    else if (left_enable == 0)
    {
        current_left_spd = 0;
    }

    if (right_enable == 1 && current_right_spd != target_right_spd)
    {
        right_delta = target_right_spd > current_right_spd ? 1 : -1;
        if (time >= next_time)
            current_right_spd += right_delta;
    }
    else if (right_enable == 0)
    {
        current_right_spd = 0;
    }

    left_forward_pwm_control = current_left_spd > 0 ? current_left_spd : 0;
    left_reverse_pwm_control = current_left_spd < 0 ? -current_left_spd : 0;
    right_forward_pwm_control = current_right_spd > 0 ? current_right_spd : 0;
    right_reverse_pwm_control = current_right_spd < 0 ? -current_right_spd : 0;
    analogWrite(PWM_LEFT_FORWARD, left_forward_pwm_control);
    analogWrite(PWM_LEFT_REVERSE, left_reverse_pwm_control);
    analogWrite(PWM_RIGHT_FORWARD, right_forward_pwm_control);
    analogWrite(PWM_RIGHT_REVERSE, right_reverse_pwm_control);

    debug_print("left_forward_pwm_control ");
    debug_println(left_forward_pwm_control);
    debug_print("left_reverse_pwm_control ");
    debug_println(left_reverse_pwm_control);
    debug_print("left_enable ");
    debug_println(left_enable);
    debug_print("right_forward_pwm_control ");
    debug_println(right_forward_pwm_control);
    debug_print("right_reverse_pwm_control ");
    debug_println(right_reverse_pwm_control);
    debug_print("right_enable ");
    debug_println(right_enable);

    if (time >= next_time)
        next_time = time + INTERVAL;
}


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
#define INTERVAL 5

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

    Serial.begin(9600);
}

void loop()
{
    while (Serial.available())
    {
        int recv = Serial.read();
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
            digitalWrite(ENABLE_LEFT, !digitalRead(ENABLE_LEFT));
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
            digitalWrite(ENABLE_RIGHT, !digitalRead(ENABLE_RIGHT));
            break;
        }

        left_forward_pwm_control = left_spd > 0 ? left_spd : 0;
        left_reverse_pwm_control = left_spd < 0 ? -left_spd : 0;
        right_forward_pwm_control = right_spd > 0 ? right_spd : 0;
        right_reverse_pwm_control = right_spd < 0 ? -right_spd : 0;

        analogWrite(PWM_LEFT_FORWARD, left_forward_pwm_control);
        analogWrite(PWM_LEFT_REVERSE, left_reverse_pwm_control);
        analogWrite(PWM_RIGHT_FORWARD, right_forward_pwm_control);
        analogWrite(PWM_RIGHT_REVERSE, right_reverse_pwm_control);
        
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
#endif
    }
    delay(1);
}


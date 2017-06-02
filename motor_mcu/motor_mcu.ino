#define LPWM 9
#define RPWM 10

#define UP_BTM 'k'
#define DOWN_BTM 'j'

#define INTERVAL 5
#define MAX_SPD_LIMIT 230
#define MIN_SPD_LIMIT -230

// TODO
// 1. Remove Serial and add I2C support
// 2. Implement a feature to smoothly transit from one spd to another?

// Range of control input is [-230, 230]
int control_inp = 0;

// Range of left/right_spd is [0, 230]
// left_spd is abs(control_inp) if control_inp <= 0
// right_spd is abs(control_inp) if control_inp >= 0
int left_spd;
int right_spd;

void setup()
{
    // Initialise everything and set to 0 speed
    pinMode(LPWM, OUTPUT);
    pinMode(RPWM, OUTPUT);
    digitalWrite(LPWM, LOW);
    digitalWrite(RPWM, LOW);

    Serial.begin(9600);
}

void loop()
{
    while (Serial.available())
    {
        int recv = Serial.read();
        if (recv == UP_BTM)
        {
            control_inp += INTERVAL;
            control_inp = control_inp >= MAX_SPD_LIMIT ? MAX_SPD_LIMIT : control_inp;
        }
        else if (recv == DOWN_BTM)
        {
            control_inp -= INTERVAL;
            control_inp = control_inp <= MIN_SPD_LIMIT ? MIN_SPD_LIMIT : control_inp;
        }

        left_spd = control_inp < 0 ? -control_inp : 0;
        right_spd = control_inp > 0 ? control_inp : 0;

        analogWrite(LPWM, left_spd);
        analogWrite(RPWM, right_spd);
    }
    delay(1);
}


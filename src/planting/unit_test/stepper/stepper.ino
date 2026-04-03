// Code for iCL57-23 Stepper motor test, this is the motor used to drive the ball screw for horizontal movement
// Uses Timer1 hardware PWM (pin 9, OC1A) to generate step pulses — frees CPU from bit-banging
// Datasheet ref: https://www.omc-stepperonline.com/icl-series-nema-23-integrated-closed-loop-stepper-motor-3-1nm-439oz-in-20-50vdc-w-14-bit-encoder-icld57-31

// FIXME: verify motor speed MAX?
const int PUL = 9; // OC1A — Timer1 PWM output
const int DIR = 6; // controls stepper direction

const int STEPS_PER_REV = 1600; // 200 base steps * 8 microsteps

/*Pulse and Direction Connection:
(1) Optically isolated, high level 4.5-5V, low voltage 0-0.5V.
(2) Max 200 KHz input frequency.
(3) The width of PUL signal is at least 2.5us, duty cycle is recommended 50%.
(4) Single pulse (step & direction), iCL-23xx and iCL-24xx support double pulse
(CW&CCW), while iCL-17xx do not support.
(5) DIR signal requires advance PUL signal minimum 5 us. */

// Timer1 CTC mode + OC1A toggle = 50% duty cycle square wave on pin 9.
// Prescaler 64: timer_clock = 16MHz / 64 = 250kHz
// freq = 125000 / (OCR1A + 1)  ->  OCR1A = 125000 / freq - 1

void setRPM(int rpm)
{
    if (rpm == 0)
    {
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // stop timer clock
        digitalWrite(PUL, LOW);
        return;
    }

    long freq = (long)rpm * STEPS_PER_REV / 60; // step pulses per second
    long ocrVal = 125000L / freq - 1;
    ocrVal = constrain(ocrVal, 0L, 65535L);
    TCNT1 = 0; // reset counter so new OCR1A takes effect immediately
    OCR1A = (unsigned int)ocrVal;

    // enable prescaler 64 (CS11 | CS10)
    TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11) | (1 << CS10))) | (1 << CS11) | (1 << CS10);
}

void setup()
{
    Serial.begin(9600);
    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    digitalWrite(DIR, HIGH); // HIGH = CW, LOW = CCW

    // Timer1: CTC mode (WGM12), toggle OC1A on compare match (COM1A0), no clock yet
    TCCR1A = (1 << COM1A0);
    TCCR1B = (1 << WGM12);
    TCNT1 = 0;
    OCR1A = 65535;
    // positive is to the right, negative is to the left
    Serial.println("Enter RPM (-150 to 150, negative = CCW):");
}

void loop()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n'); // reads whole line, consumes the \n
        input.trim();
        if (input.length() > 0)
        {
            int rpm = input.toInt();
            // DIR must be set at least 5us before PUL — digitalWrite is ~4us, setRPM() handles the rest
            digitalWrite(DIR, rpm >= 0 ? HIGH : LOW); // HIGH = CW, LOW = CCW
            delayMicroseconds(5);                     // satisfy DIR setup time (datasheet: min 5us before PUL)
            int constrainedRPM = constrain(abs(rpm), 0, 150);
            setRPM(constrainedRPM);
            Serial.print("Direction: ");
            Serial.println(rpm >= 0 ? "CW" : "CCW");
            Serial.print("Setting to RPM: ");
            Serial.println(constrainedRPM);
            Serial.println("Enter RPM (-150 to 150, negative = CCW):");
        }
    }
}
// Code for iCL57-23 Stepper motor test, this is the motor used to drive the ball screw for horizontal movement
// Uses Timer1 hardware PWM (pin 9, OC1A) to generate step pulses — frees CPU from bit-banging
// Datasheet ref: https://www.omc-stepperonline.com/icl-series-nema-23-integrated-closed-loop-stepper-motor-3-1nm-439oz-in-20-50vdc-w-14-bit-encoder-icld57-31

// Speed locked at 130 RPM
// User input: L <mm>  →  CCW   |   R <mm>  →  CW
// Example:  R 50   moves ball screw 50 mm to the right (CW)

const int PUL = 9;           // OC1A — Timer1 PWM output
const int DIR = 6;           // controls stepper direction

const int  STEPS_PER_REV = 800;   // 200 base steps * 4 microsteps
const float MM_PER_REV   = 5.0;   // SFU 1605 ball screw lead
const float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV;  // 160 steps/mm
// const float STEPS_PER_MM = 102.0;  // pre-calculated for efficiency
const int  LOCKED_RPM    = 270;

/*Pulse and Direction Connection:
(1) Optically isolated, high level 4.5-5V, low voltage 0-0.5V.
(2) Max 200 KHz input frequency.
(3) The width of PUL signal is at least 2.5us, duty cycle is recommended 50%.
(4) Single pulse (step & direction), iCL-23xx and iCL-24xx support double pulse
(CW&CCW), while iCL-17xx do not support.
(5) DIR signal requires advance PUL signal minimum 5 us. */

// Timer1 CTC mode + OC1A toggle = 50% duty cycle square wave on pin 9.
// Prescaler 64: timer_clock = 16MHz / 64 = 250kHz
// freq = 125000 / (OCR1A + 1)  →  OCR1A = 125000 / freq - 1
// Each compare match toggles pin → 2 interrupts per complete step pulse.

volatile long stepsRemaining = 0;
volatile bool motorDone      = false;

ISR(TIMER1_COMPA_vect)
{
    // Pin toggles on every interrupt; one full step pulse = 2 toggles.
    static bool halfPulse = false;
    halfPulse = !halfPulse;
    if (halfPulse) return;  // only count on the falling edge of each pulse

    if (stepsRemaining > 0)
    {
        stepsRemaining--;
        if (stepsRemaining == 0)
        {
            // Stop timer clock, disable interrupt
            TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
            TIMSK1 &= ~(1 << OCIE1A);
            motorDone = true;
        }
    }
}

void startMove(long steps)
{
    long freq   = (long)LOCKED_RPM * STEPS_PER_REV / 60;
    long ocrVal = constrain(125000L / freq - 1, 0L, 65535L);

    TCNT1  = 0;
    OCR1A  = (unsigned int)ocrVal;

    stepsRemaining = steps;
    motorDone      = false;

    TIMSK1 |= (1 << OCIE1A);  // enable compare match interrupt
    // start timer with prescaler 64 (CS11 | CS10)
    TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11) | (1 << CS10))) | (1 << CS11) | (1 << CS10);
}

void setup()
{
    Serial.begin(9600);
    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    digitalWrite(PUL, LOW);
    digitalWrite(DIR, HIGH);

    // Timer1: CTC mode (WGM12), toggle OC1A on compare match (COM1A0), no clock yet
    TCCR1A = (1 << COM1A0);
    TCCR1B = (1 << WGM12);
    TCNT1  = 0;
    OCR1A  = 65535;

    Serial.println("Speed locked at 130 RPM | SFU1605: 5mm/rev | 160 steps/mm");
    Serial.println("Enter: L <mm>  or  R <mm>");
}

void loop()
{
    if (motorDone)
    {
        motorDone = false;
        Serial.println("Done.");
        Serial.println("Enter: L <mm>  or  R <mm>");
    }

    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0) return;

        char dir = toupper(input.charAt(0));
        if (dir != 'L' && dir != 'R')
        {
            Serial.println("Invalid input. Use  L <mm>  or  R <mm>");
            return;
        }

        float mm = input.substring(1).toFloat();
        if (mm <= 0)
        {
            Serial.println("Distance must be > 0.");
            return;
        }

        // Stop any ongoing move before starting a new one
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
        TIMSK1 &= ~(1 << OCIE1A);

        digitalWrite(DIR, dir == 'R' ? HIGH : LOW);  // HIGH = CW, LOW = CCW
        delayMicroseconds(5);                         // DIR setup time (datasheet: min 5us before PUL)

        long steps = (long)(mm * STEPS_PER_MM + 0.5f);  // round to nearest step

        Serial.print("Direction: ");
        Serial.println(dir == 'R' ? "CW (R)" : "CCW (L)");
        Serial.print("Distance:  ");
        Serial.print(mm, 2);
        Serial.println(" mm");
        Serial.print("Steps:     ");
        Serial.println(steps);

        startMove(steps);
    }
}

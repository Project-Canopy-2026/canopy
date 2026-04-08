// ============================================================
//  Combined Motor Controller — BLDC (BLD-530S) + Stepper (iCL57-23)
//  Serial protocol @ 115200 baud for ROS2 via /dev/ttyUSB0
//
//  BLDC datasheet:    https://www.omc-stepperonline.com/digital-brushless-dc-motor-driver-24-48vdc-max-30a-500w-bld-530s
//  Stepper datasheet: https://www.omc-stepperonline.com/icl-series-nema-23-integrated-closed-loop-stepper-motor-3-1nm-439oz-in-20-50vdc-w-14-bit-encoder-icld57-31
//
//  Command protocol (newline-terminated strings, case-insensitive):
//    "bldc,in,<rpm>"          → bldc_spin("in", rpm)  *in = CW in auger perspective*
//    "bldc,out,<rpm>"         → bldc_spin("out", rpm)
//    "bldc,stop"              → bldc_stop()
//    "stepper,left,<mm>"      → stepper_move("left",  mm)   *CCW, speed locked at 270 RPM*
//    "stepper,right,<mm>"     → stepper_move("right", mm)   *CW,  speed locked at 270 RPM*
//    "stepper,stop"           → stepper_stop()
//    "stop"                   → bldc_stop() + stepper_stop() immediately
//
//  Replies:
//    "ACK:<cmd>"    – command received and dispatched
//    "DONE:STEPPER" – step-counted stepper move completed
//    "ERR:<reason>" – unknown or malformed command
// ============================================================

// ============================================================
//  CONSTANTS
// ============================================================
// BLDC (BLD-530S)
const int SV = 3;
const int FR = 7; // LOW = CW = IN, HIGH = CCW = OUT in auger perspective
const int EN = 8; // LOW = driver enabled
const int BLDC_RATED_RPM = 75;

// Stepper (iCL57-23) — SFU 1605 ball screw
const int PUL = 9;
const int DIR = 6;               // HIGH = CW = RIGHT, LOW = CCW = LEFT
const int  STEPS_PER_REV = 800;  // 200 base steps × 4 microsteps
const float MM_PER_REV   = 5.0;  // SFU 1605 lead
const float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV; // 160 steps/mm
const int  STEPPER_RPM   = 270;  // locked speed

volatile long stepsRemaining = 0;
volatile bool stepperDone    = false;

// ============================================================
//  HELPERS
// ============================================================
int speedRPM(int rpm)
{
    return map(rpm, 0, BLDC_RATED_RPM, 0, 255);
}

// Timer1 ISR: counts step pulses. Each compare match toggles pin → 2 interrupts per step.
ISR(TIMER1_COMPA_vect)
{
    static bool halfPulse = false;
    halfPulse = !halfPulse;
    if (halfPulse) return;

    if (stepsRemaining > 0)
    {
        stepsRemaining--;
        if (stepsRemaining == 0)
        {
            TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10)); // stop clock
            TIMSK1 &= ~(1 << OCIE1A);                              // disable interrupt
            stepperDone = true;
        }
    }
}

// Starts Timer1 at STEPPER_RPM using CTC + OC1A toggle (prescaler 64).
static void startTimer()
{
    long freq   = (long)STEPPER_RPM * STEPS_PER_REV / 60;
    long ocrVal = constrain(125000L / freq - 1, 0L, 65535L);
    TCNT1 = 0;
    OCR1A = (unsigned int)ocrVal;
    TIMSK1 |= (1 << OCIE1A);
    TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11) | (1 << CS10))) | (1 << CS11) | (1 << CS10);
}

static void stopTimer()
{
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
    TIMSK1 &= ~(1 << OCIE1A);
}

// ============================================================
//  SERIAL REPLY to ROS2
// ============================================================

void send_ack(String cmd_name) { Serial.println("ACK:" + cmd_name); }
void send_done(String cmd_name) { Serial.println("DONE:" + cmd_name); }
void send_error(String reason) { Serial.println("ERR:" + reason); }

// ============================================================
//  BLDC FUNCTIONS
// ============================================================
void bldc_spin(String direction, int rpm)
{
    digitalWrite(FR, (direction == "in") ? LOW : HIGH);
    delayMicroseconds(5); // wait 5 µs DIR-before-PUL gap

    int constrainedRPM = constrain(rpm, 0, BLDC_RATED_RPM);
    analogWrite(SV, speedRPM(constrainedRPM));
}

void bldc_stop()
{
    analogWrite(SV, 0);
}

// ============================================================
//  STEPPER FUNCTIONS
// ============================================================

// Starts a non-blocking step-counted stepper move (speed locked at STEPPER_RPM).
//   direction "left"  → CCW (LOW on DIR)
//   direction "right" → CW  (HIGH on DIR)
void stepper_move(String direction, float mm)
{
    stopTimer();
    digitalWrite(DIR, (direction == "right") ? HIGH : LOW);
    delayMicroseconds(5); // DIR setup time (datasheet: min 5 µs before PUL)

    stepsRemaining = (long)(mm * STEPS_PER_MM + 0.5f);
    stepperDone    = false;
    startTimer();
}

void stepper_stop()
{
    stopTimer();
    stepsRemaining = 0;
    stepperDone    = false;
}

// ============================================================
//  SERIAL COMMAND HANDLER
// ============================================================

// Parses a comma-delimited command string, send to
// appropriate motor function, then sends ACK immediately.
void handle_command(String cmd)
{
    cmd.toLowerCase();

    // Tokenise on commas into a small fixed array
    String tokens[5];
    int count = 0;
    int start = 0;

    for (int i = 0; i <= (int)cmd.length() && count < 5; i++)
    {
        if (i == (int)cmd.length() || cmd.charAt(i) == ',')
        {
            tokens[count++] = cmd.substring(start, i);
            start = i + 1;
        }
    }

    // Panic stop — halt all motors immediately, no further parsing needed
    if (tokens[0] == "stop")
    {
        bldc_stop();
        stepper_stop();
        send_ack("stop");
        return;
    }

    if (count < 2)
    {
        send_error("INCORRECT FORMAT:" + cmd);
        return;
    }

    String motor = tokens[0];
    String action = tokens[1];

    if (motor == "bldc")
    {
        if ((action == "in" || action == "out") && count >= 3)
        {
            int rpm = tokens[2].toInt();
            bldc_spin(action, rpm);
            send_ack("bldc," + action);
        }
        else if (action == "stop")
        {
            bldc_stop();
            send_ack("bldc,stop");
        }
        else
        {
            send_error("UNKNOWN_BLDC_CMD:" + action);
        }
    }
    else if (motor == "stepper")
    {
        if ((action == "left" || action == "right") && count >= 3)
        {
            float mm = tokens[2].toFloat();
            stepper_move(action, mm);
            send_ack("stepper," + action);
        }
        else if (action == "stop")
        {
            stepper_stop();
            send_ack("stepper,stop");
        }
        else
        {
            send_error("UNKNOWN_STEPPER_CMD:" + action);
        }
    }
    else
    {
        send_error("UNKNOWN_MOTOR:" + motor);
    }
}

// ============================================================
//  ARDUINO SETUP
// ============================================================

void setup()
{
    // 115200 baud for ROS2 communication on /dev/ttyUSB0
    Serial.begin(115200);

    // ── BLDC pins ────────────────────────────────────────────
    pinMode(SV, OUTPUT);
    pinMode(FR, OUTPUT);
    pinMode(EN, OUTPUT);
    TCCR2B = (TCCR2B & 0b11111000) | 0x03; // Timer2 prescaler = 32 → ~1953 Hz on pin 3 (within BLD-530S 1kHz–2kHz spec)
    digitalWrite(FR, LOW);                 // default IN (CW)
    digitalWrite(EN, LOW);                 // LOW = driver enabled
    analogWrite(SV, 0);                    // motor stopped at startup

    // ── Stepper pins ─────────────────────────────────────────
    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    digitalWrite(DIR, HIGH); // default CW
    TCCR1A = (1 << COM1A0);  // Timer1: CTC mode (WGM12), toggle OC1A on compare match (COM1A0), no clock yet
    TCCR1B = (1 << WGM12);
    TCNT1 = 0;
    OCR1A = 65535;

    Serial.println("READY");
}

// ============================================================
//  ARDUINO LOOP
// ============================================================

void loop()
{
    if (Serial.available() > 0)
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0)
        {
            handle_command(cmd);
        }
    }
    // Check if the step-counted stepper move has completed
    if (stepperDone)
    {
        stepperDone = false;
        send_done("STEPPER");
    }
}

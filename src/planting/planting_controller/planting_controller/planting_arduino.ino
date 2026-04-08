// ============================================================
//  Combined Motor Controller — BLDC (BLD-530S) + Stepper (iCL57-23)
//  Serial protocol @ 115200 baud for ROS2 via /dev/ttyUSB0
//
//  BLDC datasheet:    https://www.omc-stepperonline.com/digital-brushless-dc-motor-driver-24-48vdc-max-30a-500w-bld-530s
//  Stepper datasheet: https://www.omc-stepperonline.com/icl-series-nema-23-integrated-closed-loop-stepper-motor-3-1nm-439oz-in-20-50vdc-w-14-bit-encoder-icld57-31
//
//  Command protocol (newline-terminated strings, case-insensitive):
//    "bldc,in,<rpm>"              → bldc_spin("in", rpm)  *in = CW in auger perspective*
//    "bldc,out,<rpm>"             → bldc_spin("out", rpm)
//    "bldc,stop"                  → bldc_stop()
//    "stepper,left,<rpm>,<s>"     → stepper_move("left",  rpm, s*1000)
//    "stepper,right,<rpm>,<s>"    → stepper_move("right", rpm, s*1000)
//    "stepper,stop"               → stepper_stop()
//    "stop"                       → bldc_stop() + stepper_stop() immediately
//
//  Replies:
//    "ACK:<cmd>"    – command received and dispatched
//    "DONE:STEPPER" – timed stepper move completed
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
const int STEPS_PER_REV = 1600; // 200 base steps × 8 microsteps

// Stepper (iCL57-23)
const int PUL = 9;
const int DIR = 6; // HIGH = CW = LEFT, LOW = CCW = RIGHT
unsigned long move_start_time = 0;
unsigned long move_duration_ms = 0;
bool stepper_running = false;

// ============================================================
//  HELPERS
// ============================================================
int speedRPM(int rpm)
{
    return map(rpm, 0, BLDC_RATED_RPM, 0, 255);
}

// Sets Timer1 CTC frequency for stepper RPM.
void setRPM(int rpm)
{
    if (rpm == 0)
    {
        TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
        digitalWrite(PUL, LOW);
        return;
    }

    long freq = (long)rpm * STEPS_PER_REV / 60; // step pulses per second
    long ocrVal = 125000L / freq - 1;
    ocrVal = constrain(ocrVal, 0L, 65535L);

    TCNT1 = 0;
    OCR1A = (unsigned int)ocrVal;
    TCCR1B = (TCCR1B & ~((1 << CS12) | (1 << CS11) | (1 << CS10))) | (1 << CS11) | (1 << CS10);
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

// Starts a non-blocking timed stepper move.
//   direction "left"  → HIGH on DIR
//   direction "right" → LOW  on DIR
void stepper_move(String direction, int rpm, unsigned long duration_ms)
{
    digitalWrite(DIR, (direction == "left") ? HIGH : LOW);
    delayMicroseconds(5); // wait 5 µs before the first PUL edge (datasheet)

    int constrainedRPM = constrain(rpm, 0, 150);
    setRPM(constrainedRPM);

    move_start_time = millis();
    move_duration_ms = duration_ms;
    stepper_running = true;
}

void stepper_stop()
{
    setRPM(0);
    stepper_running = false;
}

// Call every loop(). Returns true and sends DONE:STEPPER when the
// timed move has elapsed; returns false otherwise.
bool stepper_check_done()
{
    if (!stepper_running)
        return false;

    if (millis() - move_start_time >= move_duration_ms)
    {
        stepper_stop();
        send_done("STEPPER");
        return true;
    }
    return false;
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
        if ((action == "left" || action == "right") && count >= 4)
        {
            int rpm = tokens[2].toInt();
            unsigned long dur = (unsigned long)tokens[3].toInt() * 1000UL; // s → ms
            stepper_move(action, rpm, dur);
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
    // Check if the timed stepper move has completed
    stepper_check_done();
}

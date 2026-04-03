// testing BLDC motor driver BLD 5305
// datasheet ref: https://www.omc-stepperonline.com/digital-brushless-dc-motor-driver-24-48vdc-max-30a-500w-bld-530s
// SV input: DC 0–5V (or PWM 5V, 1kHz–2kHz) → 0 to 75 RPM (rated speed)
// Pin 3 uses Timer2 on Arduino Uno (PWM capable)

const int SV = 3; // speed control terminal (wired to pin 3)
const int FR = 7; // CW/CCW control terminal
const int EN = 8; // enable control terminal

const int RATED_RPM = 75;

int speedRPM(int rpm)
{
    return map(rpm, 0, RATED_RPM, 0, 255);
}

void setup()
{
    Serial.begin(9600);
    pinMode(SV, OUTPUT);
    pinMode(FR, OUTPUT);
    pinMode(EN, OUTPUT);

    // Timer2 prescaler = 32 → ~1953 Hz on pin 3 (within BLD-530S 1kHz–2kHz spec)
    TCCR2B = (TCCR2B & 0b11111000) | 0x03;

    digitalWrite(FR, HIGH); // LOW is CW in auger perspective
    digitalWrite(EN, LOW);  // enable driver (LOW = enabled)

    Serial.println("Enter RPM (-75 to 75):");
}

int currentRPM = 0;

void loop()
{
    int targetRPM = Serial.parseInt();
    if (Serial.available() > 0 || targetRPM != 0)
    {
        digitalWrite(FR, targetRPM >= 0 ? LOW : HIGH);
        delayMicroseconds(5);
        currentRPM = constrain(abs(targetRPM), 0, RATED_RPM);
        analogWrite(SV, speedRPM(currentRPM));
        Serial.print("Setting to RPM: ");
        Serial.println(currentRPM);
    }
}
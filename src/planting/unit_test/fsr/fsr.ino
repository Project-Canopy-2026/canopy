// Testing FSR 400 Series force sensor (Interlink 34-00065, Model 404 Single Zone Donut)
// Datasheet ref: src/planting/unit_test/Datasheet_FSR.pdf
// Measures vertical ground-reaction force on the auger while drilling.
//
// Wiring (voltage divider, output rises with force):
//   5V --[FSR]--+--[10k RM]-- GND
//               |
//               A0

const int FSR_PIN = A0;
const float RM = 10000.0;   // measuring resistor, ohms
const float VCC = 5.0;
const float ADC_MAX = 1023.0;

// Averages a few ADC reads to smooth sensor/ADC noise, returns volts at A0.
float readFSRVoltage()
{
    long sum = 0;
    const int samples = 8;
    for (int i = 0; i < samples; i++)
    {
        sum += analogRead(FSR_PIN);
    }
    float raw = sum / (float)samples;
    return raw * VCC / ADC_MAX;
}

// Vout = RM*VCC/(RM+Rfsr)  =>  Rfsr = RM*(VCC/Vout - 1)
// Returns -1 to signal "open" (no measurable force) when voltage is ~0.
float voltageToResistance(float voltage)
{
    if (voltage < 0.005) return -1;
    return RM * (VCC / voltage - 1.0);
}

// ROUGH, UNCALIBRATED placeholder: fit to two points eyeballed off the
// datasheet's generic resistance-vs-force curve (~100g->10k ohm, ~1000g->1k ohm),
// which gives Force(g) ~= 1e6 / Rfsr. That curve is for a generic FSR in this
// circuit, not this exact part/mount, so treat this as relative-pressure only
// until it's calibrated against known weights on the actual mounted sensor.
float resistanceToForceGrams(float rfsr)
{
    if (rfsr <= 0) return 0;
    return 1000000.0 / rfsr;
}

void setup()
{
    Serial.begin(9600);
    Serial.println("FSR test — reading A0 (uncalibrated grams estimate)");
}

void loop()
{
    float voltage = readFSRVoltage();
    float rfsr = voltageToResistance(voltage);

    Serial.print("V=");
    Serial.print(voltage, 3);
    Serial.print("  R=");
    if (rfsr < 0)
    {
        Serial.print("OPEN");
        Serial.print("  grams~=0");
    }
    else
    {
        Serial.print(rfsr, 0);
        Serial.print("  grams~=");
        Serial.print(resistanceToForceGrams(rfsr), 1);
    }
    Serial.println();

    delay(200);
}

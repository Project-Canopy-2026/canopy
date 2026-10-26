// Testing FSR UX force sensor (Interlink 34-00153, strip form factor)
// Datasheet ref: src/planting/unit_test/fsr/DataSheet_FSR UX.pdf
// Sensing range 0.5N-150N (no-load resistance >10 Mohm). No Force-vs-
// Resistance/Vout curve is published for this sensor, so this reports
// raw voltage/resistance only — do not convert to force without first
// building a calibration table from known weights on the actual mount.
// Measures vertical ground-reaction force on the auger while drilling.
//
// Wiring (voltage divider, output rises with force):
//   5V --[FSR]--+--[5k RM]-- GND
//               |
//               A0

const int FSR_PIN = A0;
const float RM = 5000.0;   // measuring resistor, ohms
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

const unsigned long LOG_INTERVAL_MS = 100;

void setup()
{
    Serial.begin(9600);
    // CSV log: time_ms,voltage_V,resistance_ohm (resistance is -1 when OPEN,
    // i.e. no measurable force, so the column stays numeric for plotting).
    // Compatible with Arduino IDE's Serial Plotter (Tools > Serial Plotter),
    // or capture straight to a file with plotting/log_fsr.py.
    Serial.println("time_ms,voltage_V,resistance_ohm");
}

void loop()
{
    float voltage = readFSRVoltage();
    float rfsr = voltageToResistance(voltage);

    Serial.print(millis());
    Serial.print(",");
    Serial.print(voltage, 3);
    Serial.print(",");
    Serial.println(rfsr, 0);

    delay(LOG_INTERVAL_MS);
}

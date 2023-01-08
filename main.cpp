#include <Arduino.h>

#include "main.hpp"
#include "printf.h"
#include "serial_cmd.hpp"
#include "timer.hpp"

void _putchar(char character) { serial_writec(character); }

//===== Pin configuration
#define TOUT_PIN A0          // Outgoing air temperature sense
#define TIN_PIN A1           // Incoming air temperature sense
#define CABINET_TEMP_PIN A2  // Cabinet temperature sense
#define CABINET_HEAT_PIN 8   // Cabinet heater pin
#define ROOM_HEAT_PIN 7      // Room heater enable pin
#define BUTTON_PIN 6         // Test button

//===== Heating configuration
#define CABINET_HEAT_MINTIME_S 30     // minimum on/off time
#define ROOM_HEAT_MINTIME_S 60        // minimum on/off time
#define CABINET_HEAT_THRESHOLD_C 5.0  // Keep over +5 deg
#define ROOM_HEAT_TRESHOLD_C 15.0     // Heat only if temperature is less than this
#define TEMP_DELTA_C 2.0              // Celcius
#define ROOM_MAX_C 80.0               // maximum possible room temperature
#define ROOM_MIN_C (-50.0)            // minimum possible room temperature

//===== ADC configuration
#define ADC_MAX ((1 << 10) - 1)  // 10bits ADC
#define ADC_VREF 5.0             // Voltage reference
#define ADC_TO_V(a) (ADC_VREF * float(a) / ADC_MAX)

//===== ADC conversion parameters
#define ADC_TEMP_ZERO_POINT_V 1.840  // Zero 0 degrees voltage (defined by the bias led)
#define ADC_TEMP_VTOD_MUL 0.02       // 20mV/1 Celcius degree
#define ADC_TO_C(a) ((ADC_TO_V(a) - ADC_TEMP_ZERO_POINT_V) / ADC_TEMP_VTOD_MUL)

//===== Cabinet NTC temperature sensor
#define CAB_TEMP_R 10e3        // Reference resistor
#define CAB_TEMP_NTC_22 5e3    // NTC resistance at +22C
#define CAB_TEMP_NTC_0 14.5e3  // NTC resistance at +0C

#define NTC_R(v) ((ADC_VREF - (v)) * CAB_TEMP_R / (v))  // NTC effective resistance
#define NTC_TEMP_C(v) ((NTC_R(v) - CAB_TEMP_NTC_0) * 22 / (CAB_TEMP_NTC_22 - CAB_TEMP_NTC_0))

#define LP_BUFFER_SIZE 16  // Keep as power of 2 for arithmetic performance.

#define REPORT_INTERVAL_S 5

// Low pass filter to smooth out noisy ADC readings
struct LowPassFilter {
    LowPassFilter(int apin)
        : pin(apin)
    {
        reset();
    }

    void reset()
    {
        memset(&buffer[0], 0, sizeof(buffer));
        idx = 0;
        acc = 0;
    }

    void update() { update(analogRead(pin)); }

    int value() const { return acc / LP_BUFFER_SIZE; }

protected:
    int buffer[LP_BUFFER_SIZE];
    uint8_t pin;
    uint8_t idx;
    long acc;

    void update(int val)
    {
        // Update rolling average over last samples
        acc = acc - buffer[idx] + val;
        buffer[idx++] = val;
        idx %= LP_BUFFER_SIZE;
    }
};

static LowPassFilter roomTinADC(TIN_PIN);
static LowPassFilter roomToutADC(TOUT_PIN);
static LowPassFilter cabinetADC(CABINET_TEMP_PIN);

void update_adc()
{
    roomTinADC.update();
    roomToutADC.update();
    cabinetADC.update();
}

void report()
{
    const float Tin = ADC_TO_C(roomTinADC.value());
    const float Tout = ADC_TO_C(roomToutADC.value());
    int a = cabinetADC.value();
    const float v = ADC_TO_V(a);
    const float Tcab = NTC_TEMP_C(v);
    // serial_printfln("Tcab raw %d V %.1f", a, v);
    serial_printfln("T(in): %.1fC° T(out): %.1fC° T(cabinet): %.1fC°", Tin, Tout, Tcab);
}

void setup()
{
    Serial.begin(115200);

    analogReference(DEFAULT);

    pinMode(ROOM_HEAT_PIN, OUTPUT);
    pinMode(CABINET_HEAT_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    serial_printfln("%s %s", APP_NAME, APP_VERSION);
    serial_printfln("AC on/off time: %ds", ROOM_HEAT_MINTIME_S);
    serial_printfln("Tdelta: %.1fC°", TEMP_DELTA_C);
    serial_printfln("T(in) max: %.1fC°", ROOM_HEAT_TRESHOLD_C);
    serial_printfln("Cabinet heater on/off time: %ds", CABINET_HEAT_MINTIME_S);
    serial_printfln("T(cabinet) min: %.1fC°", CABINET_HEAT_THRESHOLD_C);

    serial_writeln(F("Ready"));

    for (int i = 0; i < LP_BUFFER_SIZE; i++) {
        update_adc();
    }

    report();
}

void test_loop()
{
    // Test button pressed. Put heat on
    serial_writeln(F("Test"));
    report();
    digitalWrite(ROOM_HEAT_PIN, HIGH);
    digitalWrite(CABINET_HEAT_PIN, HIGH);
    delay(1000);

    // Wait until button released
    while (!digitalRead(BUTTON_PIN)) update_adc();

    report();
    // Reset state
    digitalWrite(ROOM_HEAT_PIN, LOW);
    digitalWrite(CABINET_HEAT_PIN, LOW);
    delay(1000);
    serial_writeln(F("Resume"));
}

void loop()
{
    static bool errorCondition = false;
    static bool roomHeatOn = false;
    static bool cabinetHeatOn = false;
    static Timer2 roomTimer(true, ROOM_HEAT_MINTIME_S * 1000ul, true);
    static Timer2 cabinetTimer(true, CABINET_HEAT_MINTIME_S * 1000ul, true);
    static Timer2 reportTimer(true, REPORT_INTERVAL_S * 1000ul);
    static Timer2 blinkTimer(true, 500);

    update_adc();

    const uint32_t ts = millis();
    if (cabinetTimer.update(ts)) {
        // Timer expired, check status and turn internal heater on/off as needed
        const float v = ADC_TO_V(cabinetADC.value());
        const float Tcab = NTC_TEMP_C(v);
        const bool on = Tcab <= CABINET_HEAT_THRESHOLD_C;
        digitalWrite(CABINET_HEAT_PIN, on);
        if (roomHeatOn != on) {
            serial_printfln("T(cabinet) %.1fC° Cabinet Heater: %s", Tcab, on ? "ON" : "OFF");
        }
        roomHeatOn = on;
    }

    if (roomTimer.update(ts)) {
        // Room timer expired. Check temperature delta and turn heater on/off as needed
        const float Tin = ADC_TO_C(roomTinADC.value());
        const float Tout = ADC_TO_C(roomToutADC.value());
        bool on = Tin - Tout < TEMP_DELTA_C;

        if (Tin > ROOM_HEAT_TRESHOLD_C) {
            // don't enable heat if temperature is high enough
            on = false;
        }

        // check for non-sensical temperature values. These could mean failed sensor or
        // disconnected cable.
        errorCondition = Tin < ROOM_MIN_C || Tout < ROOM_MIN_C || Tin > ROOM_MAX_C || Tout > ROOM_MAX_C;
        if (errorCondition) {
            on = false;
            serial_printfln("Sensor Error: T(in): %.1fC° T(out): %.1fC°", Tin, Tout);
        }

        digitalWrite(ROOM_HEAT_PIN, on);
        if (cabinetHeatOn != on) {
            serial_printfln("T(in): %.1fC° T(out): %.1fC° AC: %d", Tin, Tout, on ? "ON" : "OFF");
        }
        cabinetHeatOn = on;
    }

    if (blinkTimer.update(ts) && errorCondition) {
        // toggle heater pin to blink the indicator led and alert the user
        digitalWrite(CABINET_HEAT_PIN, blinkTimer.flipflop());
    }

    if (reportTimer.update(ts)) {
        report();
    }

    bool button = !digitalRead(BUTTON_PIN);
    if (button) {
        test_loop();
        reportTimer.reset(millis());

        // Set the timers so that these will be immediately checked on
        // the next pass
        cabinetTimer.set();
        roomTimer.set();
    }

    if (const char* cmd = serial_read_line()) {
    }
}

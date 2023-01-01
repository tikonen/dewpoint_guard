#include <Arduino.h>

#include "main.hpp"
#include "printf.h"
#include "serial_cmd.hpp"
#include "timer.hpp"
#include "filter.hpp"

void _putchar(char character) { serial_write_char(character); }

#define TOUT_PIN A0          // Outgoing air temperature sense
#define TIN_PIN A1           // Incoming air temperature sense
#define CABINET_TEMP_PIN A2  // Cabinet temperature sense

#define ROOM_HEAT_PIN 9     // Room heater enable pin
#define CABINET_HEAT_PIN 8  // Cabinet heater pin

#define BUTTON_PIN 6

#define CABINET_HEAT_MINTIME_S 30     // minimum on/off time
#define ROOM_HEAT_MINTIME_S 60        // minimum on/off time
#define CABINET_HEAT_THRESHOLD_C 5.0  // Keep over +5 deg

#define TEMP_DELTA_C 2.0  // Celcius

#define ADC_ZERO_POINT_V 1.61  // Zero 0 degrees voltage
#define ADC_VTOD_MUL 0.02      // 20mV/1 Celcius degree
#define ADC_MAX (1 << 10 - 1)  // 10bits ADC
#define ADC_VREF 5.0
#define ADC_TO_V(a) (ADC_VREF * float(a) / ADC_MAX)
#define ADC_TO_TEMP(a) ((ADC_TO_V(a) - ADC_ZERO_POINT_V) / ADC_VTOD_MUL)

// Cabinet NTC temperature sensor
#define CAB_TEMP_R 10e3
#define CAB_TEMP_NTC_22 5e3    // +22C
#define CAB_TEMP_NTC_0 14.5e3  // +0C
#define NTC_R(v) ((ADC_VREF - (v)) * CAB_TEMP_R / (v))
#define NTC_TEMP_C(v) ((NTC_R(v) - CAB_TEMP_NTC_0) * 22 / (CAB_TEMP_NTC_22 - CAB_TEMP_NTC_0))

// IIR low pass filter to smooth out noisy ADC readings
class ADCFilter : public LowPassFilter<int>
{
public:
    const float filterRatio = 0.05;

    ADCFilter(int apin)
        : LowPassFilter(analogRead(apin), filterRatio)
        , pin(apin)
    {
    }

    int read() { return update(analogRead(pin)); }

protected:
    int pin;
};

ADCFilter roomTinADC(TIN_PIN);
ADCFilter roomToutADC(TOUT_PIN);
ADCFilter cabinetADC(CABINET_TEMP_PIN);

void update_adc()
{
    roomTinADC.read();
    roomToutADC.read();
    cabinetADC.read();
}

void report()
{
    const float Tin = ADC_TO_TEMP(roomTinADC.read());
    const float Tout = ADC_TO_TEMP(roomToutADC.read());
    const int v = ADC_TO_V(cabinetADC.read());
    const float Tcab = NTC_TEMP_C(v);
    serial_printfln("T(in): %.1C T(out): %.1C T(cabinet): %.1C", Tin, Tout, Tcab);
}

void setup()
{
    Serial.begin(115200);

    analogReference(EXTERNAL);  // Aref pin

    pinMode(ROOM_HEAT_PIN, OUTPUT);
    pinMode(CABINET_HEAT_PIN, OUTPUT);

    pinMode(BUTTON_PIN, INPUT_PULLUP);

    serial_printfln("%s %s", APP_NAME, APP_VERSION);

    serial_print(F("Initializing"));

    // Let ADC readings stabilize
    Timer2 initial(false, 3000);
    while (initial.update()) {
        update_adc();
    }

    report();
}

void loop()
{
    static bool roomHeatOn = false;
    static bool cabinetHeatOn = false;
    static Timer2 roomTimer(true, ROOM_HEAT_MINTIME_S * 1000ul);
    static Timer2 cabinetTimer(true, CABINET_HEAT_MINTIME_S * 1000ul);
    static Timer2 reportTimer(true, 60 * 1000ul);

    update_adc();

    uint32_t ts = millis();
    if (cabinetTimer.update(ts)) {
        // Timer expired, check status and turn internal heater on/off as needed
        const int v = ADC_TO_V(cabinetADC.read());
        const float Tcab = NTC_TEMP_C(v);
        const bool on = Tcab <= CABINET_HEAT_THRESHOLD_C;
        digitalWrite(CABINET_TEMP_PIN, on);
        if (roomHeatOn != on) {
            serial_printfln("T(cabinet) %.1C Heater: %d", Tcab, on);
        }
        roomHeatOn = on;
    }

    if (roomTimer.update(ts)) {
        // Room timer expired. Check temperature delta and turn heater on/off as needed
        const float Tin = ADC_TO_TEMP(roomTinADC.read());
        const float Tout = ADC_TO_TEMP(roomToutADC.read());
        const bool on = Tin - Tout < TEMP_DELTA_C;
        digitalWrite(ROOM_HEAT_PIN, on);
        if (cabinetHeatOn != on) {
            serial_printfln("T(in): %.1C T(out): %.1C Heater: %d", Tin, Tout, on);
        }
        cabinetHeatOn = on;
    }

    if (reportTimer.update(ts)) {
        report();
    }

    bool button = !digitalRead(BUTTON_PIN);
    if (button) {
        // Test button pressed. Put heat on and
        serial_println(F("Test"));
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

        ts = millis();
        cabinetTimer.reset(ts);
        roomTimer.reset(ts);
        reportTimer.reset(ts);
        serial_println(F("Resume"));
    }
}

#include "MPL3115A2.h"

MPL3115A2 baro;

void setup()
{
    Wire.begin();
    Serial.begin(115200);
    while (!Serial) {} 

    delay(2000);

    Serial.println("MPL3115A2 test!");

    if (!baro.init()) {
        Serial.println("Could not find sensor. Check wiring.");
    }

    Serial.print("Default Seapressure: ");
    Serial.println(baro.getSeaPressure());
    Serial.print("Default AltitudeOffset: ");
    Serial.println(baro.getAltitudeOffset());
    Serial.print("Default PressureOffset: ");
    Serial.println(baro.getPressureOffset());
    Serial.print("Default TemperatureOffset: ");
    Serial.println(baro.getTemperatureOffset());

    uint32_t sp = baro.getSeaPressure();
    baro.setSeaPressure(100);
    Serial.print("Seapressure (should be 100): ");
    Serial.println(baro.getSeaPressure());
    baro.setSeaPressure(sp);

    int8_t ao = baro.getAltitudeOffset();
    baro.setAltitudeOffset(10);
    Serial.print("AltitudeOffset (should be 10): ");
    Serial.println(baro.getAltitudeOffset());
    baro.setAltitudeOffset(ao);

    int16_t po = baro.getPressureOffset();
    baro.setPressureOffset(108);
    Serial.print("PressureOffset (should be 108): ");
    Serial.println(baro.getPressureOffset());
    baro.setPressureOffset(po);

    float to = baro.getTemperatureOffset();
    baro.setTemperatureOffset(2.0f);
    Serial.print("TemperatureOffset (should be 2.0): ");
    Serial.println(baro.getTemperatureOffset());
    baro.setTemperatureOffset(to);

    ao = baro.getAltitudeOffset();
    baro.setAltitudeOffset(-10);
    Serial.print("AltitudeOffset (should be -10): ");
    Serial.println(baro.getAltitudeOffset());
    baro.setAltitudeOffset(ao);

    po = baro.getPressureOffset();
    baro.setPressureOffset(-108);
    Serial.print("PressureOffset (should be -108): ");
    Serial.println(baro.getPressureOffset());
    baro.setPressureOffset(po);

    to = baro.getTemperatureOffset();
    baro.setTemperatureOffset(-2.0f);
    Serial.print("TemperatureOffset (should be -2.0f): ");
    Serial.println(baro.getTemperatureOffset());
    baro.setTemperatureOffset(to);

    Serial.print("Default Seapressure: ");
    Serial.println(baro.getSeaPressure());
    Serial.print("Default AltitudeOffset: ");
    Serial.println(baro.getAltitudeOffset());
    Serial.print("Default PressureOffset: ");
    Serial.println(baro.getPressureOffset());
    Serial.print("Default TemperatureOffset: ");
    Serial.println(baro.getTemperatureOffset());
}

float pressure;
float altitude;
float temperature;

void loop()
{
    Serial.println("-----------------------");

    if (baro.getPressure(pressure, temperature)) {
        Serial.print("Pressure: ");
        Serial.print(pressure);
        Serial.println(" Pa");
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.println(" °C");
    }
    // if (baro.getAltitude(altitude, temperature)) {
    //     Serial.print("Altitude: ");
    //     Serial.print(altitude);
    //     Serial.println(" m");
    //     Serial.print("Temperature: ");
    //     Serial.print(temperature);
    //     Serial.println(" °C");
    // }

    delay(500);
}

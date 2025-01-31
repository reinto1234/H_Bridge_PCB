#include <Arduino.h>
#include "Input_meas.h"

unsigned long delayTime = 1000;

void setup() {
    Serial.begin(19200);
    Serial.println(F("INA228 Test"));

    InputMeasurement::init();
}

void loop() {
    Serial.print("Spannung = ");
    Serial.print(InputMeasurement::getVoltage());
    Serial.println(" V");

    Serial.print("Strom = ");
    Serial.print(InputMeasurement::getCurrent());
    Serial.println(" A");

    Serial.print("Leistung = ");
    Serial.print(InputMeasurement::getPower());
    Serial.println(" W");

    Serial.println();
    delay(delayTime);
}

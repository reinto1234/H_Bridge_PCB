/*************************************************************************
 * @file Input_meas.cpp
 * @brief Input measurements for the H-Bridge Inverter System
 *
 * This file contains the implementation of input measurements, including
 * voltage, current, and power.
 ************************************************************************/

#include "Input_meas.h"
#include "mutexdefinitions.h"
#include "I2C.h"
#include "safety.h"

#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>


Adafruit_INA228 InputMeasurement::ina228;

#define ShuntResistor 0.015

float InputMeasurement::measurementBufferin[3] = {0.0, 0.0, 0.0}; // [Voltage, Current, Power]



// Call this once after ina228.begin(...)
void InputMeasurement::configure_overcurrent_alert_only(float I_limit_A, float R_shunt_ohm) {

  /**********************************************************************
   * INA228 SHUNT-OVER-VOLTAGE LIMIT (SOVL) CALCULATION
   *
   * Goal:
   *   Trigger ALERT when Shunt Voltage >= I_limit_A * R_shunt_ohm
   *
   * Datasheet facts (INA228 SOVL register):
   *   - SOVL has a fixed LSB depending on ADCRANGE:
   *
   *       ADCRANGE = 0 → ±163.84 mV full-scale
   *         Raw shunt conversion LSB = 312.5 nV (0.3125 µV)
   *         SOVL_LSB = Raw_LSB * 16 = 312.5 nV * 16 = 5 µV
   *
   *       ADCRANGE = 1 → ±40.96 mV full-scale
   *         Raw shunt conversion LSB = 78.125 nV (0.078125 µV)
   *         SOVL_LSB = 78.125 nV * 16 = 1.25 µV
   *
   *   Therefore:
   *       SOVL_LSB = (ADCRANGE==0 ? 5 µV : 1.25 µV)
   *
   * Conversion formula:
   *       Desired_Shunt_Voltage = I_limit_A * R_shunt_ohm
   *
   *       sovl_raw = Desired_Shunt_Voltage / SOVL_LSB
   *
   * Example (your setup):
   *       R_shunt = 0.015 Ω
   *       I_limit = 5 A
   *       V_limit = 5 * 0.015 = 0.075 V = 75 mV
   *
   *       BUT with ADCRANGE = 0 (default):
   *           full-scale = 163.84 mV
   *           → 75 mV is within ADC range
   *
   *       ADCRANGE = 0 → SOVL_LSB = 5 µV
   *       Max measurable shunt voltage = 163.84 mV
   *
   *       sovl_raw_max = 163.84 mV / 5 µV = 32768
   *       limited_sovl_raw = min(32768, 240 mV / 5 µV)
   *
   * Result:
   *       If I_limit * R_shunt exceeds ADC range → clamp to max
   *
   **********************************************************************/

  // --- Transparent ALERT (non-latching) ---
  //ina228.setAlertLatch();

  // 1) Desired voltage across shunt
  float v_limit_V = I_limit_A * R_shunt_ohm;  // V

  // 2) Full-scale shunt voltage based on ADCRANGE
  float vshunt_fullscale =
      ina228.getADCRange() ? 40.96e-3f : 163.84e-3f;  // V (datasheet)

  // Clamp if user sets a too-large current limit
  if (v_limit_V > vshunt_fullscale) {
    v_limit_V = vshunt_fullscale;
  }

  // 3) SOVL LSB (datasheet)
  float sovl_lsb_V = ina228.getADCRange() ? 1.25e-6f : 5.0e-6f;

  // 4) Compute raw register value
  long sovl_raw = lroundf(v_limit_V / sovl_lsb_V);

  // 5) clamp to 16-bit positive
  if (sovl_raw > 0x7FFF) sovl_raw = 0x7FFF;
  if (sovl_raw < 0)      sovl_raw = 0;

  // 6) Write SOVL register
  Adafruit_I2CDevice diag_dev(0x40, &I2CINA);
  Adafruit_I2CRegister SOVL(&diag_dev, INA228_REG_SOVL, 2, MSBFIRST);
  SOVL.write((uint16_t)sovl_raw);

  // 7) Enable ONLY SHNTOL alert source
  Adafruit_I2CRegister DIAG(&diag_dev, INA228_REG_DIAGALRT, 2, MSBFIRST);
  uint16_t v = DIAG.read();

  v &= ~0x4FFFu;       // disable all alert sources
  v |=  (1u << 6);     // enable SHNTOL (bit 6)
  v &= ~(1u << 15);    // transparent / non-latching

  DIAG.write(v);

  // 8) Clear stale flags
  (void)ina228.alertFunctionFlags();
}




void InputMeasurement::init() {

    I2CINA.begin(SDA_PIN, SCL_PIN, I2CSpeed);
    sleep(1);   // Wait for the sensor to initialize
    scanI2C();
    Serial.println("I2CINA scanned");


    if (!ina228.begin(0x40, &I2CINA)) {
        while (1); // Halt execution if sensor is not found
    }
    Serial.println("INA228 found!");

    configure_overcurrent_alert_only(I_Shutdown, ShuntResistor); // Configure overcurrent alert at 2A

    

    // Set the shunt resistor value
    ina228.setShunt(ShuntResistor,10);
    ina228.setCurrentConversionTime(INA228_TIME_1052_us);
    ina228.setVoltageConversionTime(INA228_TIME_1052_us);
    ina228.setAveragingCount(INA228_COUNT_64);
    ina228.setAlertPolarity(INA228_ALERT_POLARITY_INVERTED);
    

    delay(5);
    uint16_t flags = ina228.alertFunctionFlags();

    Serial.print("Flags: 0x");
    Serial.println(flags, HEX);

    if (flags & (1 << 0)) Serial.println("MEMSTAT fault");
    if (flags & (1 << 1)) Serial.println("Conversion Ready");
    if (flags & (1 << 2)) Serial.println("Power over-limit");
    if (flags & (1 << 3)) Serial.println("Bus under-limit");
    if (flags & (1 << 4)) Serial.println("Bus over-limit");
    if (flags & (1 << 5)) Serial.println("Shunt under-limit");
    if (flags & (1 << 6)) Serial.println("Shunt over-limit");
    if (flags & (1 << 7)) Serial.println("Temp over-limit");
    if (flags & (1 << 9)) Serial.println("Math overflow");
    if (flags & (1 << 10)) Serial.println("Charge overflow");
    if (flags & (1 << 11)) Serial.println("Energy overflow");
}

void InputMeasurement::init1(){
    I2CINA.begin(SDA_PIN, SCL_PIN, I2CSpeed);
}

float InputMeasurement::getVoltage() {
    return ((int)((ina228.readBusVoltage() / 1000000) * 100 + 0.5) / 100.0);
}

float InputMeasurement::getCurrent() {
    return ((int)((ina228.readCurrent() / 1000) * 100 + 0.5) / 100.0);
}

float InputMeasurement::getPower() {
    float powertemp = ina228.readPower();
    if (powertemp == 262144000.00) {
        return getPower(); 
    }
    return powertemp / 1000;
}

float* InputMeasurement::measurementall() {
    
    float voltage = getVoltage();
    float current = getCurrent();
    float power = getPower();
    

    if (xSemaphoreTake(measurementinMutex, portMAX_DELAY) == pdTRUE) {
        measurementBufferin[0] = voltage;
        measurementBufferin[1] = current;
        measurementBufferin[2] = power;
        xSemaphoreGive(measurementinMutex);
    }
    return measurementBufferin;
    
}

void InputMeasurement::scanI2C() {
    Serial.println("Scanning I2C devices...");
    byte deviceCount = 0;

    // Scan for devices on the I2C bus
    for (byte address = 1; address < 127; address++) {
        I2CINA.beginTransmission(address);
        if (I2CINA.endTransmission() == 0) {  // If no error, device found
            deviceCount++;
            Serial.print("Found device at 0x");
            Serial.println(address, HEX);
        }
    }
    Serial.println("I2C scan complete.");

     //If fewer than one device is found, restart the I2C bus
     if (deviceCount < 1) {
         Serial.println("Less than one device found, restarting I2C...");
         I2CINA.end();  // End the current I2C bus communication
         delay(1000);    // Small delay before restarting the bus
         I2CINA.begin(SDA_PIN, SCL_PIN, I2CSpeed);  // Restart the I2C bus
         delay(1000);
         scanI2C();  // Scan again after restarting
     }
}

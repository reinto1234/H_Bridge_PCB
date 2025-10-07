/************************************************************************
 * @file PWM.cpp
 * @brief PWM control and H-Bridge Inverter implementation
 *
 * This file contains the implementation of the H-Bridge Inverter, including
 * the PI controller, sine wave generation, and PWM signal control.
 ************************************************************************/

#include "PWM.h"
#include "mutexdefinitions.h"

// Global variable for the inverter
// Ensures only one instance of the inverter is created
HBridgeInverter* inverter = nullptr;
hw_timer_t* spwmTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


//Interrupt Service Routine (ISR) for the SPWM timer
void IRAM_ATTR onSPWMTimer() {
    if (inverter != nullptr) {
        // Achtung: ISR → möglichst schnell! Kein Serial, keine Mutex!
        inverter->generateSPWM();
    }
}


// Function to start the inverter
void startInverter(float vrms) {
    // Check if an inverter instance already exists
    if (inverter == nullptr) {
        // Acquire mutex before modifying the inverter instance
        if (xSemaphoreTake(inverterMutex, portMAX_DELAY) == pdTRUE) {
            inverter = new HBridgeInverter(vrms);
            inverter->begin();
            Serial.println("H-Bridge Inverter started!");
            xSemaphoreGive(inverterMutex);
        }
    } else {
        Serial.println("Inverter is already running!");
    }
}

// Function to stop the inverter
void stopInverter() {
    if (inverter == nullptr) {
        Serial.println("No inverter is running!");
        return;
    }

    // Temporären Zeiger erstellen, um das Objekt später sicher löschen zu können
    HBridgeInverter* inverter_to_delete = inverter;

    // =================== KRITISCHER ABSCHNITT START ===================
    // Dies pausiert vorübergehend die Interrupts auf dem aktuellen CPU-Kern,
    // um die Race Condition mit der ISR absolut sicher zu verhindern.
    portENTER_CRITICAL(&timerMux);

    // 1. Den globalen Zeiger SOFORT auf nullptr setzen.
    //    Selbst wenn ein Interrupt jetzt noch irgendwie durchkäme, würde seine
    //    "if (inverter != nullptr)"-Prüfung fehlschlagen.
    inverter = nullptr;

    // 2. Den Timer stoppen, während die Interrupts pausiert sind.
    if (spwmTimer != nullptr) {
        timerAlarmDisable(spwmTimer);
        timerDetachInterrupt(spwmTimer);
        timerEnd(spwmTimer);
        spwmTimer = nullptr;
    }

    // =================== KRITISCHER ABSCHNITT ENDE ====================
    // Interrupts werden jetzt wieder zugelassen.
    portEXIT_CRITICAL(&timerMux);


    // --- Ab hier sind wir sicher vor der ISR ---

    // 3. Pins von der LEDC-Hardware lösen
    ledcDetachPin(HIN1);
    ledcDetachPin(HIN2);
    ledcDetachPin(LIN1);
    ledcDetachPin(LIN2);

    // 4. Hardware-Invertierung für alle Pins zurücksetzen
    GPIO.func_out_sel_cfg[HIN1].inv_sel = 0;
    GPIO.func_out_sel_cfg[HIN2].inv_sel = 0;
    GPIO.func_out_sel_cfg[LIN1].inv_sel = 0;
    GPIO.func_out_sel_cfg[LIN2].inv_sel = 0;

    // 5. Alle Ansteuerungs-Pins auf LOW setzen
    digitalWrite(HIN1, LOW);
    digitalWrite(HIN2, LOW);
    digitalWrite(LIN1, LOW);
    digitalWrite(LIN2, LOW);

    ledcWrite(PWM_CHANNEL_1, 0);
    ledcWrite(PWM_CHANNEL_2, 0);
    
    // 6. Das Objekt über den temporären Zeiger sicher löschen.
    //    Der Mutex ist hier nicht mehr zwingend nötig, da der globale Zeiger
    //    bereits nullptr ist und kein anderer Task mehr zugreifen wird.
    delete inverter_to_delete;

    Serial.println("H-Bridge Inverter safely stopped.");
}

// Constructor for HBridgeInverter class
HBridgeInverter::HBridgeInverter(float vrms)
    : pwmDutyCycle(0.5),
      stepIndex(0),
      controller(/*kp=*/0.03f, /*ki=*/0.02f, /*outputMin=*/0.2f, /*outputMax=*/1.0f, /*vrms=*/vrms)
{
    //Serial.println("Sine Table Size: " + String(SINE_STEPS));
    //sineTable.resize(SINE_STEPS); // Resize sine table to SINE_STEPS size

    // Precompute sine wave table based on modulation type
    for (int i = 0; i < SINE_STEPS; i++) {
        float sinValue = sin(2 * M_PI * i / SINE_STEPS);
        sineTable[i] = (int16_t)((sinValue) * 511);  // Bipolar: Full sinus range
    }
 
}

// Initialize the inverter
void HBridgeInverter::begin() {
    // Initialize measurement buffer
    for (int i = 0; i < 10; i++) {
        inverter->measurementBuffer[i] = 0.0f;
    }
    
    // PWM setup
    ledcSetup(PWM_CHANNEL_1, S_FREQ, RESOLUTION);

    ledcAttachPin(HIN1, PWM_CHANNEL_1);

    // BIPOLAR
    // Attach PWM to GPIOs
    // Initialize MCPWM GPIOs for complementary outputs
    ledcAttachPin(HIN2, PWM_CHANNEL_1);
    ledcAttachPin(LIN1, PWM_CHANNEL_1);
    ledcAttachPin(LIN2, PWM_CHANNEL_1);
    GPIO.func_out_sel_cfg[LIN1].inv_sel = 1;
    GPIO.func_out_sel_cfg[HIN2].inv_sel = 1;

    

    // Timer initialisieren
    spwmTimer = timerBegin(0, 80, true); // Timer 0, prescaler 80 → 1 µs Takt (80 MHz / 80 = 1 MHz)
    timerAttachInterrupt(spwmTimer, &onSPWMTimer, true);
    timerAlarmWrite(spwmTimer, TIMER_INTERVAL_US, true); // Alle x µs auslösen
    timerAlarmEnable(spwmTimer);



}


void HBridgeInverter::generateSPWM() {
    // In ISR-safe code
    // 0) read base sample (keep it unsigned)
    int16_t base = sineTable[stepIndex];          // -512..511
    __atomic_store_n(&controller.stepindexPWM, stepIndex , __ATOMIC_RELEASE);

    // 1) load amplitude (Q10: 0..1023)
    int16_t amp = __atomic_load_n(&PIController::g_amp_q10, __ATOMIC_ACQUIRE);

    // 2) fixed-point multiply (keep 32-bit), then scale back to 10 bits
    int32_t prod = (int16_t)base * amp;          // 0..~1,046,529
    int16_t scaled = prod >> 10;                  // -512..511

    uint16_t final_val = (uint16_t)(scaled + 512); // 0... 1023

    // 3) clamp without constrain() (ISR-safe)
    uint16_t pwmValue = (final_val > 1023u) ? 1023u : (uint16_t)final_val;

    // Bipolar modulation: both bridge arms switch simultaneously
    ledcWrite(PWM_CHANNEL_1, pwmValue);
        

    // Sinusschritt inkrementieren
    stepIndex = (stepIndex + 1) % SINE_STEPS;
}






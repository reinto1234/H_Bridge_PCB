# H-Bridge PCB Project

## Project Description
This project involves the design and implementation of an **educational single-phase H-Bridge inverter** controlled by an **Espressif ESP32**.  
The system demonstrates **bipolar sinusoidal PWM (SPWM)** operation with **closed-loop VRMS amplitude control** and a **browser-based interface** for monitoring and control.  
The H-Bridge can drive loads with the following ratings:

- **Input Voltage:** 24 V  
- **Input Current:** up to 10 A  
- **Power:** ≈ 240 W  

### Features
- Fixed **switching frequency:** 35 kHz  
- Fixed **output frequency:** 50 Hz  
- **Bipolar SPWM modulation** (only)  
- **Closed-loop VRMS control** using PI regulation  
- **Web-based UI** (HTTP + WebSocket) for control and telemetry  
- **Input/Output power measurement**  
- **Overcurrent protection** using INA228 (auto driver shutdown + UI fault latch)  

The project consists of two main components:  
1. **H_Bridge_PCB_Design** — KiCad PCB layout and schematic.  
2. **H_Bridge_PCB_Code** — Firmware for the ESP32 controller.

---

## Directory Structure

<h2>Directory Structure</h2>
<pre><code>H_Bridge_PCB_Project/
├── H_Bridge_PCB_Design/        # KiCad design files
├── H_Bridge_PCB_Code/          # ESP32 firmware
│   ├── include/                # Header files
│   │   ├── Controller.h
│   │   ├── I2C.h
│   │   ├── Input_meas.h
│   │   ├── mutexdefinitions.h
│   │   ├── Output_meas.h
│   │   ├── PWM.h
│   │   ├── spi_sampler.h
│   │   ├── Tasks.h
│   │   └── webserver.h
│   │
│   ├── src/                    # Source files
│   │   ├── Controller.cpp
│   │   ├── I2C.cpp
│   │   ├── Input_meas.cpp
│   │   ├── Output_meas.cpp
│   │   ├── PWM.cpp
│   │   ├── spi_sampler.cpp
│   │   ├── Tasks.cpp
│   │   ├── webserver.cpp
│   │   └── main.cpp
│   │
│   ├── data/                   # Webserver files (LittleFS)
│   │   ├── index.html
│   │   ├── script.js
│   │   └── style.css
│
├── lib/                        # Optional local libraries
├── platformio.ini              # PlatformIO configuration
├── images/                     # Flowchart and figures
└── README.md                   # This file
</code></pre>


---

## Requirements

### Hardware:
- PCB based on KiCad design  
- 24 V DC power supply (≥ 10 A)  
- Resistive or inductive load (max ≈ 240 W)  
- Gate drivers with integrated dead-time  
- INA228 current/voltage monitor for input sensing  

### Software:
- [KiCad](https://www.kicad.org/) — PCB design  
- [PlatformIO](https://platformio.org/) — firmware build  
- [VS Code](https://code.visualstudio.com/) — recommended IDE  

---

## Webserver
The ESP32 provides a **built-in Wi-Fi access point** and hosts a **web-based interface** for control and measurement visualization.

### Connect to WLAN Access Point
- **SSID:** `H_Bridge_Control`  
- **Password:** `12345678`

### Open the web interface:
👉 [http://192.168.4.1/](http://192.168.4.1/)

**Functions:**
- Start/Stop the inverter  
- Set VRMS target  
- Display live telemetry (Vin, Iin, VRMS, IRMS, PF, P, Q, f)  
- View and acknowledge overcurrent faults  

---

## Code
The firmware is implemented in **C++** (Arduino / FreeRTOS).  
The web interface is built with **HTML**, **JavaScript**, and **CSS**.

---

### Flowchart
![Flowchart of the program](images/Flowchart.png)

---

## Code Documentation
Brief overview of the most relevant components.

### Inverter Implementation (`PWM.cpp`)
Implements the real-time control of the full-bridge inverter:

- **startInverter()** — Initializes and enables PWM output.  
- **stopInverter()** — Stops PWM and resets inverter state.  
- **generateSPWM()** — Produces bipolar SPWM using a sine table.  
- **begin()** — Configures LEDC channels, frequency, and timers.  
- **getMeasurements()** — Retrieves latest measurement snapshot.  
- **computePI()** — Executes PI algorithm to maintain VRMS target.  
- **loop()** — Runs the inverter control logic at fixed timing intervals.  

### Controller Implementation (`Controller.cpp`)
Implements the PI controller used to regulate the inverter’s RMS output voltage:

- **PIController()** — Initializes PI gains, integrator state, limits, and VRMS target.  
- **ControlRMS()** — Computes RMS error and updates the global Q10 amplitude factor atomically.  
- **compute()** — Executes PI algorithm with anti-windup and output clamping to min/max limits.  
- **g_amp_q10** — Global Q10-formatted amplitude value used by the SPWM stage.  


### I²C Bus Setup (`I2C.cpp`)
Implements the I²C interface for the INA228 current/voltage sensor:

- **I2CINA** — Dedicated I²C bus instance (port 0) used by the input measurement subsystem.  


### Input Measurement (`Input_meas.cpp`)
Implements DC input voltage, current, and power measurement using INA228:

- **init()** — Initializes I²C bus, scans for devices, configures INA228, and sets averaging/timing.  
- **init1()** — Lightweight I²C initialization helper without full sensor setup.  
- **configure_overcurrent_alert_only()** — Configures INA228 alert registers for shunt overcurrent shutdown.  
- **getVoltage()** — Reads and returns the DC input voltage in volts.  
- **getCurrent()** — Reads and returns the shunt current in amperes.  
- **getPower()** — Reads and returns the input power in watts with simple error rejection.  
- **measurementall()** — Updates the shared input measurement buffer (voltage, current, power) in a mutex-protected way and returns it.  
- **scanI2C()** — Scans the I²C bus, prints found devices, and restarts the bus if no device is detected.  


### Main Application (`main.cpp`)
Implements system bring-up and FreeRTOS task orchestration:

- **setup()** — Initializes serial logging, mutexes, input measurement, Wi-Fi AP, webserver, SPI measurement engines, safety pins, and the overcurrent interrupt; then creates all application tasks.  
- **loop()** — Empty main loop; periodically yields since all logic is implemented in FreeRTOS tasks.  
- **Task handles** — Stores references to sampler, analyzer, WebSocket, controller, printer, and emergency stop tasks for debugging/management.  


### Mutex Definitions (`mutexdefinitions.cpp`)
Implements globally shared FreeRTOS mutexes for synchronization:

- **inverterMutex** — Protects access to the inverter state and control functions.  
- **measurementinMutex** — Protects shared input measurement buffer.  
- **measurementoutMutex** — Protects shared output measurement structures.  
- **measurementSpiMutex** — Protects SPI-related measurement buffers and state.  


### Output Measurement (`Output_meas.cpp`)
Implements decimation, CIC filtering, RMS, and frequency analysis for AMC1306 bitstreams:

- **buildByteCicLut()** — Precomputes CIC integrator/differentiator contributions for all 8-bit patterns.  
- **OutputMeasurements()** — Configures OSR, full-scale voltage, buffer sizes, and initial bit rate.  
- **init()** — Allocates decimation ring buffers, analysis windows, and period buffers (with PSRAM support) and creates a snapshot mutex.  
- **copyRecentWindowWithStats()** — Copies the newest segment from the decimation ring and computes mean, min, and max.  
- **findLastTwoRisingZcHyst()** — Detects the last two rising zero-crossings with hysteresis to define one AC period.  
- **computeRmsOnePeriodExact()** — Computes exact RMS over one period using piecewise linear interpolation.  
- **processRxBytesAndUpdateBitrate()** — Processes raw SPI bytes, runs CIC pipeline, generates decimated voltage samples, and updates effective bit rate.  
- **analyzeStep()** — Centers the signal, finds period boundaries, computes frequency, extracts a centered period, and calculates RMS.  
- **getSnapshot()** — Returns the latest snapshot (RMS, frequency, period length) in a thread-safe way.  
- **copyLastPeriod()** — Copies the last detected waveform period into a caller-provided buffer.  
- **copySinceSeq32()** — Provides incremental decimated samples since a given sequence index, for streaming or plotting.  


### Emergency Stop (`safety.cpp`)
Implements minimal emergency-stop signaling for overcurrent events:

- **g_emergency_stop** — Global volatile flag indicating an active emergency stop condition.  
- **onAlertISR()** — ISR that sets the emergency stop flag when the INA228 alert pin fires.  


### SPI Sampler (`spi_sampler.cpp`)
Implements DMA-driven SPI acquisition for dual AMC1306 isolated ADC channels:

- **g_ch1_om / g_ch2_om** — OutputMeasurements instances for both channels with hardware-specific scaling.  
- **g_sampler1 / g_sampler2** — Sampler context structures for SPI hosts, pins, DMA channels, and transactions.  
- **g_pack1 / g_pack2** — Pairs of sampler context and OutputMeasurements used by sampler tasks.  
- **spiSamplerInitMeasurements()** — Initializes both OutputMeasurements engines and validates buffer allocation.  
- **spiInitAndStart()** — Allocates DMA buffers, configures SPI bus and device, prepares transactions, and seeds the transaction queue.  


### Task Scheduler (`Tasks.cpp`)
Implements all FreeRTOS tasks for sampling, analysis, UI updates, control, and safety:

- **spiSamplerTask()** — Retrieves completed SPI DMA transactions, feeds data into OutputMeasurements, requeues transactions, and maintains sampling timing.  
- **analyzerTask()** — Periodically calls analyzeStep() on a given OutputMeasurements instance to update RMS and frequency.  
- **printerTask()** — Periodically prints analyzer timing, latest period samples, RMS, and frequency for both channels over serial.  
- **webSocketTask()** — Processes WebSocket events in a loop to keep client connections responsive.  
- **webSocketUpdate()** — Periodically collects input and output measurements and sends them as JSON via WebSocket to the UI.  
- **ControllerTask()** — Runs the VRMS control loop by feeding the measured RMS value into the PI controller of the inverter.  
- **EmergencyStopTask()** — Monitors the global emergency-stop flag, stops the inverter, signals the E-STOP output, and notifies the web UI when overcurrent is detected.  


### Webserver (`webserver.cpp`)
Implements Wi-Fi access point, HTTP endpoints, and WebSocket telemetry/UI control:

- **resetDefaults()** — Resets VRMS target and running state to their default values.  
- **initWiFi()** — Starts the Wi-Fi SoftAP for the H-Bridge control interface.  
- **initServer()** — Mounts LittleFS, serves static `/index.html`, `/style.css`, `/script.js`, and defines `/start`, `/stop`, and `/ack-trip` endpoints.  
- **updateMeasurements()** — Packs input/output data and frequency into JSON and broadcasts it over WebSocket.  
- **broadcastStatus()** — Sends current inverter running state to all connected clients via WebSocket.  
- **onWebSocketEvent()** — Handles client connect/disconnect events and sends initial status on connect.  
- **broadcastTrip()** — Notifies all clients of a trip condition and updates running state.  
- **_VRMS / _isRunning** — Static internal state for desired RMS value and inverter run/stop status.  


**HTTP Endpoints:**
- `/` – Main HTML page  
- `/start` – Start inverter  
- `/stop` – Stop inverter  
- `/update` – Update VRMS target  
- `/bipolar.png` – Waveform reference  

---

## Overcurrent Protection
Overcurrent protection is implemented through the **INA228** fault pin:  
1. When a current threshold is exceeded, **drivers are disabled immediately**.  
2. A **fault message** is sent to the web interface.  
3. After user acknowledgment, the inverter can be restarted safely.  

---

## Safety  
- Use **current-limited** power supplies during development.  
- Verify **MOSFET temperature and load** ratings.  
- For **educational/laboratory use only**.

---

## License
Released for **educational and research purposes**.  
Use responsibly.

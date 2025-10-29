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

H_Bridge_PCB_Project/
│-- H_Bridge_PCB_Design/ # KiCad design files
│-- H_Bridge_PCB_Code/ # ESP32 firmware
│ │-- include/ # Header files
│ │ │-- Controller.h
│ │ │-- I2C.h
│ │ │-- Input_meas.h
│ │ │-- mutexdefinitions.h
│ │ │-- Output_meas.h
│ │ │-- PWM.h
│ │ │-- spi_sampler.h
│ │ │-- Tasks.h
│ │ │-- webserver.h
│ │
│ │-- src/ # Source files
│ │ │-- Controller.cpp
│ │ │-- I2C.cpp
│ │ │-- Input_meas.cpp
│ │ │-- Output_meas.cpp
│ │ │-- PWM.cpp
│ │ │-- spi_sampler.cpp
│ │ │-- Tasks.cpp
│ │ │-- webserver.cpp
│ │ │-- main.cpp
│ │
│ │-- data/ # Webserver files (LittleFS)
│ │ │-- index.html
│ │ │-- script.js
│ │ │-- style.css
│
│-- lib/ # Optional local libraries
│-- platformio.ini # PlatformIO configuration
│-- images/ # Flowchart and figures
│-- README.md # This file

markdown
Code kopieren

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

### Structure
H_Bridge_PCB_Code/
│-- data/ # Web UI (LittleFS)
│ │-- index.html # User interface
│ │-- script.js # WebSocket control & live data
│ │-- style.css # UI styling
│
│-- src/ # Core application
│ │-- Controller.cpp # PI control loop for VRMS tracking
│ │-- I2C.cpp # Shared INA228 I²C communication
│ │-- Input_meas.cpp # Input voltage, current, power via INA228
│ │-- Output_meas.cpp # RMS, phase, PF, and frequency calculation
│ │-- PWM.cpp # SPWM generation (bipolar, 35 kHz)
│ │-- webserver.cpp # HTTP/WebSocket server
│ │-- spi_sampler.cpp # High-rate output sampling
│ │-- Tasks.cpp # Task scheduling and inter-task sync
│ │-- main.cpp # System initialization and control logic
│
│-- include/ # All header files
│-- platformio.ini # PlatformIO configuration

yaml
Code kopieren

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

---

### Controller and Measurement
- **Controller.cpp** — PI regulator adjusts modulation amplitude based on measured VRMS.  
- **Input_meas.cpp** — INA228 sensor readings (Vin, Iin, Pin).  
- **Output_meas.cpp / spi_sampler.cpp** — Sample and compute RMS, power, phase, PF, frequency.  
- **Tasks.cpp** — FreeRTOS task structure separating PWM, controller, measurement, and web updates.  

---

### Web Server and WebSocket Implementation (`webserver.cpp`)
Handles all network communication and UI updates:

- **initWiFi()** — Starts Wi-Fi access point.  
- **initServer()** — Serves static files and opens WebSocket connection.  
- **updateMeasurements()** — Periodically broadcasts input/output data.  
- **handleUserCommand()** — Parses user actions (Start/Stop/VRMS set).  
- **faultNotify()** — Sends overcurrent/fault messages to the UI.  

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
- Always operate with **dead-time** and isolated gate drivers.  
- Use **current-limited** power supplies during development.  
- Verify **MOSFET temperature and load** ratings.  
- For **educational/laboratory use only**.

---

## License
Released for **educational and research purposes**.  
Use responsibly.
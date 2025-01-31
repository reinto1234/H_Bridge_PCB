# H-Bridge PCB Project

## Projektbeschreibung
Dieses Projekt beinhaltet den Entwurf und die Umsetzung einer H-Brücke unter Verwendung eines Espressif ESP32 Controllers. Die H-Brücke kann Lasten mit den folgenden Spezifikationen steuern:

- **Eingangsspannung:** 24V
- **Eingangsstrom:** 10A
- **Leistung:** 240W

Das Projekt besteht aus zwei Hauptbestandteilen:
1. **H_Bridge_PCB_Design:** Enthält das vollständige PCB-Design in KiCad.
2. **H_Bridge_PCB_Code:** Enthält den Code für den ESP32 zur Steuerung der H-Brücke.

---

## Verzeichnisstruktur

```
H_Bridge_PCB_Project/
│-- H_Bridge_PCB_Design/   # PCB-Design (KiCad-Dateien)
│-- H_Bridge_PCB_Code/     # Quellcode für den ESP32
│   │-- src/               # Hauptcode-Verzeichnis
│   │-- lib/               # Zusätzliche Bibliotheken
│   │-- platformio.ini     # Konfigurationsdatei für PlatformIO
│-- README.md              # Diese Datei
```

---

## Voraussetzungen
Bevor das Projekt gestartet werden kann, sollten folgende Software- und Hardwarekomponenten bereitgestellt werden:

### Hardware:
- ESP32 Entwicklungsboard
- Leistungs-MOSFETs (passend für 24V/10A)
- Schutzdioden
- Treiber-ICs für MOSFETs
- PCB gemäß KiCad-Design

### Software:
- [KiCad](https://www.kicad.org/) (für PCB-Design)
- [PlatformIO](https://platformio.org/) (für ESP32-Entwicklung)
- [VS Code](https://code.visualstudio.com/) (empfohlen für die Code-Entwicklung)

---

## Installation und Nutzung
### 1. PCB-Design
Öffne den **H_Bridge_PCB_Design**-Ordner in KiCad und überprüfe oder modifiziere das Platinenlayout.

### 2. ESP32 Code-Entwicklung
1. **PlatformIO installieren** (falls nicht bereits vorhanden):
   ```sh
   pip install platformio
   ```
2. Projekt in VS Code mit PlatformIO öffnen.
3. Den Code in den ESP32 hochladen:
   ```sh
   pio run --target upload
   ```
4. Serielle Ausgabe zur Fehlersuche anzeigen:
   ```sh
   pio device monitor
   ```

---

## Funktionen des ESP32-Codes
- PWM-Steuerung zur Regelung der Motorgeschwindigkeit
- Richtungssteuerung über GPIO-Pins
- Schutzmechanismen zur Vermeidung von Kurzschlüssen
- Serielle Schnittstelle zur Steuerung und Überwachung

---

## To-Do / Verbesserungen
- Implementierung einer PID-Regelung für präzisere Motorsteuerung
- Integration von Überstromschutz
- Entwicklung einer Web-Oberfläche zur drahtlosen Steuerung

---

## Lizenz
Dieses Projekt steht unter der **MIT-Lizenz**. Es kann frei genutzt, modifiziert und verteilt werden.



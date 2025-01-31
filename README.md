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
│   │-- include/           # Headfiles Verzeichnis
│   │-- lib/               # Zusätzliche Bibliotheken
│   │-- platformio.ini     # Konfigurationsdatei für PlatformIO
│-- README.md              # Diese Datei
```

---

## Voraussetzungen
Bevor das Projekt gestartet werden kann, sollten folgende Software- und Hardwarekomponenten bereitgestellt werden:

### Hardware:
- PCB gemäß KiCad-Design

### Software:
- [KiCad](https://www.kicad.org/) (für PCB-Design)
- [PlatformIO](https://platformio.org/) (für ESP32-Entwicklung)
- [VS Code](https://code.visualstudio.com/) (empfohlen für die Code-Entwicklung)




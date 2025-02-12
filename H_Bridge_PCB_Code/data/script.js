document.addEventListener("DOMContentLoaded", () => {
    const freqInput = document.getElementById("freq");
    const startStopBtn = document.getElementById("startStopBtn");
    const modulationRadios = document.querySelectorAll('input[name="modulation"]');

    let isRunning = false;
    let previousFreq = localStorage.getItem("freq") || "1000";
    freqInput.value = previousFreq;

    if (localStorage.getItem("modulation")) {
        document.querySelector(`input[name="modulation"][value="${localStorage.getItem("modulation")}"]`).checked = true;
    }
    if (localStorage.getItem("isRunning") === "true") {
        isRunning = true;
        disableControls(true);
        updateStartStopButton(true);
    }

    startStopBtn.addEventListener("click", () => {
        let value = parseInt(freqInput.value);
        if (value < 1000 || value > 45000) {
            alert("Frequency must be between 1000Hz and 45000Hz");
            freqInput.value = previousFreq;
            return;
        }

        isRunning = !isRunning;
        localStorage.setItem("isRunning", isRunning);
        disableControls(isRunning);
        updateStartStopButton(isRunning);

        if (isRunning) {
            fetch(`/start?freq=${value}&modulation=${document.querySelector('input[name="modulation"]:checked').value}`);
        } else {
            fetch("/stop");
        }
    });

    function updateStartStopButton(isRunning) {
        if (isRunning) {
            startStopBtn.textContent = "Stop";
            startStopBtn.classList.remove("start");
            startStopBtn.classList.add("stop");
        } else {
            startStopBtn.textContent = "Start";
            startStopBtn.classList.remove("stop");
            startStopBtn.classList.add("start");
        }
    }

    function disableControls(disable) {
        freqInput.disabled = disable;
        modulationRadios.forEach(radio => radio.disabled = disable);
    }

    // WebSocket initialisieren und immer aktuell halten
    function initWebSocket() {
        const socket = new WebSocket(`ws://${window.location.hostname}:81`);

        socket.addEventListener("message", (event) => {
            try {
                const data = JSON.parse(event.data);

                // Sofortige Aktualisierung der Messwerte
                document.getElementById("inputVoltage").textContent = `${data.voltage.toFixed(2)}V`;
                document.getElementById("inputCurrent").textContent = `${data.current.toFixed(2)}A`;
                document.getElementById("inputPower").textContent = `${data.power.toFixed(2)}W`;

                document.getElementById("outputVoltage").textContent = `${data.voltage_out.toFixed(2)}V`;
                document.getElementById("outputCurrent").textContent = `${data.current_out.toFixed(2)}A`;
                document.getElementById("outputPower").textContent = `${data.power_out.toFixed(2)}W`;
                document.getElementById("outputFrequency").textContent = `${data.frequency.toFixed(2)}Hz`;

                // Effizienzberechnung mit Vermeidung von NaN
                let efficiency = (data.power_out > 0 && data.power > 0) ? (data.power_out / data.power) * 100 : 0;
                document.getElementById("efficiency").textContent = `${efficiency.toFixed(2)}%`;
            } catch (error) {
                console.error("Error parsing WebSocket data:", error);
            }
        });

        socket.addEventListener("error", (error) => {
            console.error("WebSocket Error:", error);
        });

        socket.addEventListener("close", () => {
            console.warn("WebSocket disconnected. Reconnecting...");
            setTimeout(initWebSocket, 1000); // Schnellerer Reconnect (1 Sekunde)
        });
    }

    initWebSocket(); // WebSocket starten
});

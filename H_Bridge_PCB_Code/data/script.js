document.addEventListener("DOMContentLoaded", () => {
    const freqInput = document.getElementById("freq");
    const startStopBtn = document.getElementById("startStopBtn");
    const modulationRadios = document.querySelectorAll('input[name="modulation"]');

    // This single 'isRunning' variable will be controlled by the server
    let isRunning = false;

    // --- ROBUST STATE RESTORATION ---
    // Load saved frequency from previous sessions
    if (localStorage.getItem("freq")) {
        freqInput.value = localStorage.getItem("freq");
    }
    // Load saved modulation from previous sessions
    const savedModulation = localStorage.getItem("modulation");
    if (savedModulation) {
        // --- FIX: Check if the element exists before setting its property ---
        const radioToSelect = document.querySelector(`input[name="modulation"][value="${savedModulation}"]`);
        if (radioToSelect) {
            radioToSelect.checked = true;
        }
    }

    // --- EVENT LISTENERS ---
    startStopBtn.addEventListener("click", () => {
        const value = parseInt(freqInput.value);
        if (value < 1000 || value > 45000) {
            alert("Frequency must be between 1000Hz and 45000Hz");
            return;
        }

        // Save settings for next time
        localStorage.setItem("freq", value);
        localStorage.setItem("modulation", document.querySelector('input[name="modulation"]:checked').value);
        
        // Determine which command to send based on the server-controlled state
        const command = isRunning ? "stop" : "start";
        const modulationValue = document.querySelector('input[name="modulation"]:checked').value;
        const url = (command === 'start') 
            ? `/start?freq=${value}&modulation=${modulationValue}`
            : '/stop';
        
        // Send command to server
        fetch(url).catch(err => console.error("Fetch Error:", err));
    });

    // --- UI UPDATE FUNCTIONS (Controlled by server messages) ---
    function updateStartStopButton(running) {
        isRunning = running; // Update global state
        if (running) {
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

    // --- WEBSOCKET HANDLING ---
    function initWebSocket() {
        const socket = new WebSocket(`ws://${window.location.hostname}:81`);

        socket.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                
                // Differentiate between status messages and measurement data
                if (data.type === 'status') {
                    // --- FIX: UI state is now driven ONLY by the server ---
                    updateStartStopButton(data.isRunning);
                    disableControls(data.isRunning);
                } else {
                    updateMeasurementReadings(data);
                }
            } catch (error) {
                console.error("Error processing WebSocket data:", error);
            }
        };

        socket.onerror = (error) => {
            console.error("WebSocket Error:", error);
        };

        socket.onclose = () => {
            console.warn("WebSocket disconnected. Attempting to reconnect...");
            // When connection is lost, revert to a safe "stopped" state
            updateStartStopButton(false);
            disableControls(false);
            setTimeout(initWebSocket, 2000);
        };
    }

    function updateMeasurementReadings(data) {
        document.getElementById("inputVoltage").textContent = `${(data.voltage ?? 0).toFixed(2)}V`;
        document.getElementById("inputCurrent").textContent = `${(data.current ?? 0).toFixed(2)}A`;
        document.getElementById("inputPower").textContent = `${(data.power ?? 0).toFixed(2)}W`;
        document.getElementById("outputVoltage").textContent = `${(data.voltage_out ?? 0).toFixed(2)}V`;
        document.getElementById("outputCurrent").textContent = `${(data.current_out ?? 0).toFixed(2)}A`;
        document.getElementById("outputPower").textContent = `${(data.power_out ?? 0).toFixed(2)}W`;
        document.getElementById("outputFrequency").textContent = `${(data.frequency ?? 0).toFixed(2)}Hz`;

        // Schritt 1: Rohen Wirkungsgrad berechnen
        let rawEfficiency = (data.power_out > 0 && data.power > 0)
                    ? (data.power_out / data.power) * 100
                    : 0;

        // Schritt 2: Prüfen, ob der Wert über 100 liegt
        let efficiency = (rawEfficiency > 100) ? 0 : rawEfficiency;

        document.getElementById("efficiency").textContent = `${efficiency.toFixed(2)}%`;
        }

        // Start the connection process
        initWebSocket();
});
// script.js
document.addEventListener("DOMContentLoaded", () => {
  const vrmsInput = document.getElementById("vrms");
  const startStopBtn = document.getElementById("startStopBtn");

  // Server-controlled running state
  let isRunning = false;

  // ---- Restore saved settings ----
  const savedVrms = localStorage.getItem("vrms");
  if (savedVrms !== null && vrmsInput) {
    vrmsInput.value = savedVrms;
  }

  // ---- Event handlers ----
  startStopBtn.addEventListener("click", () => {
    if (!vrmsInput) return;

    const min = parseFloat(vrmsInput.min || "0");
    const max = parseFloat(vrmsInput.max || "50");
    const step = parseFloat(vrmsInput.step || "0.1");
    const value = parseFloat(vrmsInput.value);

    if (Number.isNaN(value) || value < min || value > max) {
      alert(`VRMS must be between ${min}V and ${max}V`);
      return;
    }
    // Snap to step if needed
    const snapped = Math.round(value / step) * step;

    // Save settings
    localStorage.setItem("vrms", snapped);

    // Build URL based on current server-driven state
    const command = isRunning ? "stop" : "start";
    const url =
      command === "start"
        ? `/start?vrms=${encodeURIComponent(snapped)}`
        : "/stop";

    // Fire and let the server drive UI updates via WebSocket
    fetch(url).catch((err) => console.error("Fetch Error:", err));
  });

  function updateStartStopButton(running) {
    isRunning = running;
    if (running) {
      startStopBtn.textContent = "Stop";
      startStopBtn.classList.add("stopped");
    } else {
      startStopBtn.textContent = "Start";
      startStopBtn.classList.remove("stopped");
    }
  }

  function disableControls(disable) {
    if (vrmsInput) vrmsInput.disabled = disable;
  }

  // ---- WebSocket handling ----
  function initWebSocket() {
    const socket = new WebSocket(`ws://${window.location.hostname}:81`);

    socket.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);

        // 'status' messages control UI state
        if (data.type === "status") {
          updateStartStopButton(Boolean(data.isRunning));
          disableControls(Boolean(data.isRunning));
          return;
        }

        // Otherwise assume measurement payload
        updateMeasurementReadings(data);
      } catch (error) {
        console.error("Error processing WebSocket data:", error);
      }
    };

    socket.onerror = (error) => {
      console.error("WebSocket Error:", error);
    };

    socket.onclose = () => {
      console.warn("WebSocket disconnected. Attempting to reconnect...");
      // Revert to a safe "stopped" state until we reconnect
      updateStartStopButton(false);
      disableControls(false);
      setTimeout(initWebSocket, 2000);
    };
  }

  // ---- Measurement UI updates ----
  function updateMeasurementReadings(data) {
    // Input
    const inV = document.getElementById("inputVoltage");
    const inA = document.getElementById("inputCurrent");
    const inP = document.getElementById("inputPower");

    // Output
    const outV = document.getElementById("outputVoltage");
    const outA = document.getElementById("outputCurrent");
    const outP = document.getElementById("outputPower");
    const outF = document.getElementById("outputFrequency");

    if (inV) inV.textContent = `${Number(data.voltage ?? 0).toFixed(2)}V`;
    if (inA) inA.textContent = `${Number(data.current ?? 0).toFixed(2)}A`;
    if (inP) inP.textContent = `${Number(data.power ?? 0).toFixed(2)}W`;

    if (outV) outV.textContent = `${Number(data.voltage_out ?? 0).toFixed(2)}V`;
    if (outA) outA.textContent = `${Number(data.current_out ?? 0).toFixed(2)}A`;
    if (outP) outP.textContent = `${Number(data.power_out ?? 0).toFixed(2)}W`;
    if (outF) outF.textContent = `${Number(data.frequency ?? 0).toFixed(2)}Hz`;

    // Efficiency calculation
    const effElem = document.getElementById("efficiency");
    if (effElem) {
      const pin = Number(data.power ?? 0);
      const pout = Number(data.power_out ?? 0);
      const rawEfficiency = pin > 0 && pout > 0 ? (pout / pin) * 100 : 0;
      const efficiency = rawEfficiency > 100 ? 0 : rawEfficiency;
      effElem.textContent = `${efficiency.toFixed(2)}%`;
    }
  }

  // Start WebSocket connection
  initWebSocket();
});

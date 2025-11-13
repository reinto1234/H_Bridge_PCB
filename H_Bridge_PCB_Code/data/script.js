document.addEventListener("DOMContentLoaded", () => {
  const vrmsInput     = document.getElementById("vrms");
  const startStopBtn  = document.getElementById("startStopBtn");
  const tripModal     = document.getElementById("trip-modal");
  const tripReasonEl  = document.getElementById("trip-reason");
  const tripOk        = document.getElementById("trip-ok");

  let isRunning = false;
  let ws;

  // Restore saved VRMS
  const savedVrms = localStorage.getItem("vrms");
  if (savedVrms !== null && vrmsInput) vrmsInput.value = savedVrms;

  // -------- UI helpers --------
  function updateStartStopButton(running) {
    isRunning = running;
    // Button text
    startStopBtn.textContent = running ? "Stop" : "Start";
    // Button color: default green; red when running
    startStopBtn.classList.toggle("running", running);
    // Disable VRMS input while running
    if (vrmsInput) vrmsInput.disabled = running;
  }

  function showTripModal(reason) {
    if (tripReasonEl) tripReasonEl.textContent = reason || "Overcurrent detected";
    if (tripModal) tripModal.classList.add("show");
    // Reflect stopped state
    updateStartStopButton(false);
  }

  function closeTripModal() {
    if (tripModal) tripModal.classList.remove("show");
  }

  // -------- Start/Stop click --------
  startStopBtn.addEventListener("click", () => {
    if (!vrmsInput) return;

    const min  = parseFloat(vrmsInput.min || "0");
    const max  = parseFloat(vrmsInput.max || "50");
    const step = parseFloat(vrmsInput.step || "0.1");
    const val  = parseFloat(vrmsInput.value);

    if (Number.isNaN(val) || val < min || val > max) {
      alert(`VRMS must be between ${min} V and ${max} V`);
      return;
    }

    const snapped = Math.round(val / step) * step;
    localStorage.setItem("vrms", snapped);

    const command = isRunning ? "stop" : "start";
    const url = command === "start" ? `/start?vrms=${encodeURIComponent(snapped)}` : "/stop";

    fetch(url).catch((err) => console.error("Fetch Error:", err));
  });

  // -------- Trip modal: close / OK handler --------
  if (tripModal) {
    tripModal.addEventListener("click", (e) => {
      if (e.target === tripModal || e.target.closest("[data-close]")) {
        closeTripModal();
      }
    });
  }

  if (tripOk) {
    tripOk.addEventListener("click", () => {
      fetch("/ack-trip", { method: "POST" })
        .then(() => closeTripModal())
        .catch(err => console.error("Ack trip failed:", err));
    });
  }

  // -------- WebSocket --------
  function initWebSocket() {
    ws = new WebSocket(`ws://${window.location.hostname}:81/`);

    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);

        if (data.type === "status") {
          updateStartStopButton(Boolean(data.isRunning));
          return;
        }

        if (data.type === "trip") {
          // Show popup with server-supplied reason or our default
          showTripModal(data.reason || "Overcurrent detected");
          return;
        }

        updateMeasurementReadings(data);
      } catch (e) {
        console.error("WS parse error:", e);
      }
    };

    ws.onclose = () => {
      console.warn("WS disconnected — retrying in 2s");
      updateStartStopButton(false);
      setTimeout(initWebSocket, 2000);
    };
  }

  // -------- Measurement UI updates --------
  function updateMeasurementReadings(data) {
    const inV  = document.getElementById("inputVoltage");
    const inA  = document.getElementById("inputCurrent");
    const inP  = document.getElementById("inputPower");
    const outV = document.getElementById("outputVoltage");
    const outA = document.getElementById("outputCurrent");
    const outP = document.getElementById("outputPower");
    const outF = document.getElementById("outputFrequency");

    if (inV)  inV.textContent  = `${Number(data.voltage      ?? 0).toFixed(2)} V`;
    if (inA)  inA.textContent  = `${Number(data.current      ?? 0).toFixed(2)} A`;
    if (inP)  inP.textContent  = `${Number(data.power        ?? 0).toFixed(2)} W`;

    if (outV) outV.textContent = `${Number(data.voltage_out ?? 0).toFixed(2)} V`;
    if (outA) outA.textContent = `${Number(data.current_out ?? 0).toFixed(2)} A`;
    if (outP) outP.textContent = `${Number(data.power_out   ?? 0).toFixed(2)} W`;
    if (outF) outF.textContent = `${Number(data.frequency   ?? 0).toFixed(2)} Hz`;

    const effElem = document.getElementById("efficiency");
    if (effElem) {
      const pin  = Number(data.power ?? 0);
      const pout = Number(data.power_out ?? 0);
      const raw  = pin > 0 && pout > 0 ? (pout / pin) * 100 : 0;
      effElem.textContent = `${(raw > 100 ? 0 : raw).toFixed(2)}%`;
    }
  }

  // Go!
  initWebSocket();
});

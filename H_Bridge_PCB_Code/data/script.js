document.addEventListener("DOMContentLoaded", () => {
    const freqInput = document.getElementById("freq");
    const applyFreqBtn = document.getElementById("applyFreq");
    const startStopBtn = document.getElementById("startStopBtn");
    const modulationRadios = document.querySelectorAll('input[name="modulation"]');

    let isRunning = false;
    let previousFreq = localStorage.getItem("freq") || "1000";
    freqInput.value = previousFreq;

    // Load saved settings
    if (localStorage.getItem("modulation")) {
        document.querySelector(`input[name="modulation"][value="${localStorage.getItem("modulation")}"]`).checked = true;
    }
    if (localStorage.getItem("isRunning") === "true") {
        isRunning = true;
        disableControls(true);
        startStopBtn.textContent = "Stop";
        startStopBtn.classList.remove("start");
        startStopBtn.classList.add("stop");
    }

    applyFreqBtn.addEventListener("click", () => {
        let value = parseInt(freqInput.value);
        if (value < 1000 || value > 45000) {
            alert("Frequency must be between 1000Hz and 45000Hz");
            freqInput.value = previousFreq;
            return;
        }
        previousFreq = value;
        localStorage.setItem("freq", value);
        fetch(`/update?freq=${value}`);
    });

    startStopBtn.addEventListener("click", () => {
        isRunning = !isRunning;
        localStorage.setItem("isRunning", isRunning);
        disableControls(isRunning);

        if (isRunning) {
            startStopBtn.textContent = "Stop";
            startStopBtn.classList.remove("start");
            startStopBtn.classList.add("stop");
            fetch("/start");
        } else {
            startStopBtn.textContent = "Start";
            startStopBtn.classList.remove("stop");
            startStopBtn.classList.add("start");
            fetch("/stop");
        }
    });

    modulationRadios.forEach(radio => {
        radio.addEventListener("change", () => {
            localStorage.setItem("modulation", radio.value);
            fetch(`/update?modulation=${radio.value}`);
        });
    });

    function disableControls(disable) {
        freqInput.disabled = disable;
        applyFreqBtn.disabled = disable;
        modulationRadios.forEach(radio => radio.disabled = disable);
    }
});

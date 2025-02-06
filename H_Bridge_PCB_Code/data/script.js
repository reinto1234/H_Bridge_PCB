document.addEventListener("DOMContentLoaded", () => {
    const freqInput = document.getElementById("freq");
    const startStopBtn = document.getElementById("startStopBtn");
    const modulationRadios = document.querySelectorAll('input[name="modulation"]');
    const outputFrequency = document.getElementById("outputFrequency");

    let isRunning = false;
    let previousFreq = localStorage.getItem("freq") || "1000";
    freqInput.value = previousFreq;

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

        if (isRunning) {
            startStopBtn.textContent = "Stop";
            startStopBtn.classList.remove("start");
            startStopBtn.classList.add("stop");
            fetch(`/start?freq=${value}&modulation=${document.querySelector('input[name="modulation"]:checked').value}`);
        } else {
            startStopBtn.textContent = "Start";
            startStopBtn.classList.remove("stop");
            startStopBtn.classList.add("start");
            fetch("/stop");
        }
    });

    function disableControls(disable) {
        freqInput.disabled = disable;
        modulationRadios.forEach(radio => radio.disabled = disable);
    }
});

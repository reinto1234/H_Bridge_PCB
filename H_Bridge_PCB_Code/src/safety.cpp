#include "safety.h"


volatile bool g_emergency_stop = false;

void IRAM_ATTR onAlertISR() {
    g_emergency_stop = true;
}

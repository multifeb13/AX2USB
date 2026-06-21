#include <Arduino.h>
#include "ax2usb.h"
#include "util.h"

#include "debug.h"

namespace {

constexpr uint8_t data_pin = D9;
constexpr uint8_t clock_pin = D10;

}  // namespace

ax2usb::AX2USB a2u;
bool running;

void
setup() {
	Serial1.begin(115200);
	delay(100);
	if (!a2u.begin(data_pin, clock_pin)) {
		Serial1.println("Failed to init ax2usb");
		return;
	}
#ifdef ARDUINO_SEEED_XIAO_RP2040
	// turn off LEDs
	pinMode(16, OUTPUT);
	pinMode(17, OUTPUT);
	pinMode(25, OUTPUT);
	digitalWrite(16, HIGH);
	digitalWrite(17, HIGH);
	digitalWrite(25, HIGH);
#endif

#ifdef ARDUINO_ARCH_RP2040
	rp2040.wdt_begin(2000);
#endif
	running = true;
}

void
loop() {
	if (!running) {
		delay(1000);
		return;
	}
	a2u.loop();
#ifdef ARDUINO_ARCH_RP2040
	rp2040.wdt_reset();
#endif
}

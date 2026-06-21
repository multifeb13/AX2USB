#include "hid_util.h"
#include <Arduino.h>
#include <iterator>
#include "util.h"

#include "debug.h"

namespace hid_util {

namespace {

constexpr int USB_SEND_RETRY_COUNT = 3;
constexpr uint16_t DO_NOTHING = 0x00;

}  // namespace

static uint8_t
usb_key_to_mod_mask(uint8_t usb_key) {
	if (usb_key < HID_KEY_CONTROL_LEFT || usb_key > HID_KEY_GUI_RIGHT) {
		return 0;
	}
	return (1 << (usb_key - HID_KEY_CONTROL_LEFT));
}

#if AX2USB_DEBUG
static std::string
usb_key_str(uint8_t key) {
	if (key >= HID_KEY_0 && key <= HID_KEY_9) {
		return std::string{ static_cast<char>('0' + key - HID_KEY_0) };
	} else if (key >= HID_KEY_A && key <= HID_KEY_Z) {
		return std::string{ static_cast<char>('A' + key - HID_KEY_A) };
	} else if (key == HID_KEY_PRINT_SCREEN) {
		return "Prt";
	} else if (key == HID_KEY_PAUSE) {
		return "Pause";
	} else {
		return std::string{ util::hexchar((key & 0xf0) >> 4), util::hexchar(key & 0x0f) };
	}
}

std::string
HidUtil::usb_codes_str() const {
	std::string ret;
	ret.reserve(24);
	for (size_t i = 0; i < std::size(usb_codes); i++) {
		ret += '[';
		ret += usb_key_str(usb_codes[i]);
		ret += ']';
	}

	return ret;
}

static char
usb_mod_char(uint8_t mod_key) {
	switch (mod_key) {
		case HID_KEY_CONTROL_LEFT:
			return 'C';
		case HID_KEY_SHIFT_LEFT:
			return 'S';
		case HID_KEY_ALT_LEFT:
			return 'A';
		case HID_KEY_GUI_LEFT:
			return 'W';
		case HID_KEY_CONTROL_RIGHT:
			return 'c';
		case HID_KEY_SHIFT_RIGHT:
			return 's';
		case HID_KEY_ALT_RIGHT:
			return 'a';
		case HID_KEY_GUI_RIGHT:
			return 'w';
		default:
			return '*';
	}
}

static char
key_mark(bool make_break) {
	return make_break ? '+' : '-';
}

static char
mod_mark(bool make_break) {
	return make_break ? '#' : '~';
}

static std::string
usb_mods_str(const HidUtil::usb_mod_t& mod) {
	std::string ret;

	ret += mod.l_ctrl ? 'C' : 'c';
	ret += mod.l_shift ? 'S' : 's';
	ret += mod.l_alt ? 'A' : 'a';
	ret += mod.l_gui ? 'W' : 'w';
	ret += '|';
	ret += mod.r_ctrl ? 'C' : 'c';
	ret += mod.r_shift ? 'S' : 's';
	ret += mod.r_alt ? 'A' : 'a';
	ret += mod.r_gui ? 'W' : 'w';

	return ret;
}
#endif

bool
HidUtil::update_usb_codes(uint8_t code, bool make_break) {
	bool modified = false;
	if (make_break) {
		for (size_t i = 0; i < std::size(usb_codes); i++) {
			if (usb_codes[i] == code) {
				break;
			} else if (usb_codes[i] == 0) {
				usb_codes[i] = code;
				modified = true;
				break;
			}
		}
	} else {
		for (size_t i = 0; i < std::size(usb_codes); i++) {
			if (usb_codes[i] == code) {
				if (i == std::size(usb_codes) - 1 || usb_codes[i + 1] == 0) {
					usb_codes[i] = 0;
				} else {
					std::copy(&usb_codes[i + 1], std::end(usb_codes), &usb_codes[i]);
					usb_codes[std::size(usb_codes) - 1] = 0;
				}
				modified = true;
				break;
			}
		}
	}
	return modified;
}

bool
HidUtil::update_usb_modifier(uint8_t mask, bool make_break) {
	auto prev = usb_mod.value;
	if (make_break) {
		usb_mod.value |= mask;
	} else {
		usb_mod.value &= ~mask;
	}
	return usb_mod.value != prev;
}

void
HidUtil::wait_usb_ready() {
	while (!usb_hid.ready()) {
		delay(1);
	}
}

void
HidUtil::send_keyboard_report() {
	for (int i = 0; i < USB_SEND_RETRY_COUNT; i++) {
		if (usb_hid.keyboardReport(report_id_kbd, usb_mod.value, usb_codes)) {
			break;
		}
		wait_usb_ready();
		if (i == USB_SEND_RETRY_COUNT) {
			DEBUG_PRINTLN("Failed to send keyboard to USB");
		}
	}
}

void
HidUtil::send_report16(uint8_t report_id, uint16_t usage) {
	for (int i = 0; i < USB_SEND_RETRY_COUNT; i++) {
		if (usb_hid.sendReport16(report_id, usage)) {
			break;
		}
		wait_usb_ready();
		if (i == USB_SEND_RETRY_COUNT) {
			DEBUG_PRINTLN("Failed to send report to USB");
		}
	}
}

void
HidUtil::send_usb_key(uint8_t usb, bool make_break) {
	if (auto mask = usb_key_to_mod_mask(usb); mask) {
		// modifiers
		if (update_usb_modifier(mask, make_break)) {
			send_keyboard_report();
			DEBUG_PRINTLN(">%c%c %s %s", mod_mark(make_break), usb_mod_char(usb), usb_mods_str(usb_mod).c_str(), usb_codes_str().c_str());
		}
	} else {
		// normal keys
		if (update_usb_codes(usb, make_break)) {
			send_keyboard_report();
			DEBUG_PRINTLN(">%c%s %s %s", key_mark(make_break), usb_key_str(usb).c_str(), usb_mods_str(usb_mod).c_str(),
			              usb_codes_str().c_str());
		}
	}
}

void
HidUtil::send_usb_key_mod(uint8_t usb, uint8_t usb_mod_key, bool make_break) {
	bool cod = update_usb_codes(usb, make_break);
	bool mod = update_usb_modifier(usb_key_to_mod_mask(usb_mod_key), make_break);
	if (cod || mod) {
		send_keyboard_report();
		DEBUG_PRINTLN(">%c%c%c%s %s %s", mod_mark(make_break), usb_mod_char(usb_mod_key), key_mark(make_break),
		              usb_key_str(usb).c_str(), usb_mods_str(usb_mod).c_str(), usb_codes_str().c_str());
	}
}

void
HidUtil::send_usb_key_oneshot(uint8_t usb) {
	if (update_usb_codes(usb, true)) {
		send_keyboard_report();
		wait_usb_ready();
	}
	update_usb_codes(usb, false);
	// send break code even if make code was not sent, to be sure
	send_keyboard_report();
}

void
HidUtil::send_report16_oneshot(uint8_t report_id, uint16_t usage) {
	send_report16(report_id, usage);
	wait_usb_ready();
	send_report16(report_id, DO_NOTHING);
}

}  // namespace hid_util

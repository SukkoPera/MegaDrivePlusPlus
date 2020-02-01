/*******************************************************************************
 * This file is part of MegaDrive++.                                           *
 *                                                                             *
 * Copyright (C) 2015-2019 by SukkoPera <software@sukkology.net>               *
 *                                                                             *
 * MegaDrive++ is free software: you can redistribute it and/or modify         *
 * it under the terms of the GNU General Public License as published by        *
 * the Free Software Foundation, either version 3 of the License, or           *
 * (at your option) any later version.                                         *
 *                                                                             *
 * MegaDrive++ is distributed in the hope that it will be useful,              *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of              *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the               *
 * GNU General Public License for more details.                                *
 *                                                                             *
 * You should have received a copy of the GNU General Public License           *
 * along with MegaDrive++. If not, see <http://www.gnu.org/licenses/>.         *
 *******************************************************************************
 *
 * MegaDrive++ - Universal Region mod, 50/60 Hz switch and In-Game-Reset (IGR)
 * for Sega Mega Drive (AKA Genesis)
 *
 * Please refer to the GitHub page and wiki for any information:
 * https://github.com/SukkoPera/MegaDrivePlusPlus
 */

/* Arduino Nano:
 *
 *                                 +-----+
 *                    +------------| USB |------------+
 *                    |            +-----+            |
 *     (Built-in LED) | [ ]D13/SCK        MISO/D12[ ] |
 *                    | [ ]3.3V           MOSI/D11[X]~| [LED Blue]
 *                    | [ ]V.ref     ___    SS/D10[X]~| [LED Green]
 *          Reset Out | [X]A0       / N \       D9[X]~| [LED Red]
 *           Reset In | [X]A1      /  A  \      D8[X] | [Single Pin Led]
 * JP3/4 (Video Mode) | [X]A2      \  N  /      D7[X] | Pad Port Pin 1
 *   JP1/2 (Language) | [X]A3       \_0_/       D6[X]~| Pad Port Pin 2
 *          [LCD SDA] | [X]A4/SDA               D5[X]~| Pad Port Pin 3
 *          [LCD SCL] | [X]A5/SCL               D4[X] | Pad Port Pin 4
 *                    | [ ]A6              INT1/D3[X]~| Pad Port Pin 6
 *                    | [ ]A7              INT0/D2[X] | Pad Port Pin 7
 *                +5V | [X]5V                  GND[X] | GND
 *                    | [ ]RST                 RST[ ] |
 *                    | [ ]GND   5V MOSI GND   TX1[ ] |
 *                    | [ ]Vin   [ ] [ ] [ ]   RX0[X] | Pad Port Pin 9
 *                    |          [ ] [ ] [ ]          |
 *                    |          MISO SCK RST         |
 *                    | NANO-V3                       |
 *                    +-------------------------------+
 *
 * Connections of pins in square brackets are optional. All the others are
 * mandatory.
 *
 * Wiring considerations:
 * - We read the pad port status through an ISR triggered by a level change on
 *   the SELECT line. Said line is triggered very quickly for 6-button pads (~4
 *   us), so the ISR has very little time to complete. Thus we need to keep all
 *   button lines on the same port and only PORTD has enough pins on an
 *   ATmega328.
 * - Outputting debugging messages is useful, but unfortunately the hardware
 *   UART is right on PORTD on the 328. Actually we only need the TX pin
 *   (ATmega328 pin 3, mapped to pin 1 on an Arduino), so we keep that one free.
 *   We cannot use the hardware UART support though, since that would also
 *   prevent us from using pin 0 at will, thus we resort to using a send-only
 *   software serial implementation. Note that while we are no longer tied to
 *   any specific pin at this point, we really want to use pin 1, since that is
 *   connected to the onboard Serial <-> USB converter on Arduino boards.
 */

#define RESET_IN_PIN A1
#define RESET_OUT_PIN A0
#define VIDEOMODE_PIN A2
#define LANGUAGE_PIN A3
#define MODE_LED_R_PIN 9          // PWM
#define MODE_LED_G_PIN 10         // PWM
#define MODE_LED_B_PIN 11         // PWM
#define PAD_LED_PIN LED_BUILTIN


/*******************************************************************************
 * BUTTON COMBO SETTINGS
 ******************************************************************************/

// DON'T TOUCH THIS! Just look at it for the button names you can use below!
enum __attribute__ ((__packed__)) PadButton {
	MD_PAD_6BTN =  1 << 15,	// Not a button, set to 1 if pad is 6-button

	MD_BTN_Z =     1 << 11,
	MD_BTN_Y =     1 << 10,
	MD_BTN_X =     1 << 9,
	MD_BTN_MODE =  1 << 8,
	MD_BTN_UP =    1 << 7,
	MD_BTN_DOWN =  1 << 6,
	MD_BTN_LEFT =  1 << 5,
	MD_BTN_RIGHT = 1 << 4,
	MD_BTN_B =     1 << 3,
	MD_BTN_C =     1 << 2,
	MD_BTN_A =     1 << 1,
	MD_BTN_START = 1 << 0
};

/* Button combo that enables the other combos.
 *
 * Note: That vertical bar ("pipe") means that the buttons must be pressed
 *       together.
 */
#define TRIGGER_COMBO (MD_BTN_START | MD_BTN_B)

/* Reset combo. This (and the following) are to be considered in addition to
 * TRIGGER_COMBO.
 */
#define RESET_COMBO (MD_BTN_A | MD_BTN_C)

// Region/video mode combos
#define EUR_COMBO MD_BTN_DOWN
#define USA_COMBO MD_BTN_RIGHT
#define JAP_COMBO MD_BTN_LEFT


/*******************************************************************************
 * ADVANCED SETTINGS
 ******************************************************************************/

/* Enable fast control of I/O pins
 *
 * This enables very fast (2 clock cycles) control of I/O pins. The DigitalIO
 * library by greiman is needed for this: https://github.com/greiman/DigitalIO.
 *
 * This should make startup faster and hopefully will solve problems with Virtua
 * Racing or Ecco The Dolphin (See issue #5 on GitHub).
 */
//~ #define ENABLE_FAST_IO

// Offset in the EEPROM at which the current mode should be saved
#define MODE_ROM_OFFSET 42

// Time to wait after mode change before saving the new mode (milliseconds)
const unsigned long MODE_SAVE_DELAY = 3000L;

// Force the reset line level when active. Undefine to enable auto-detection.
//#define FORCE_RESET_ACTIVE_LEVEL LOW

/* Colors to use to indicate the video mode, in 8-bit RGB componentes. Unless
 * you really want weird colors, use only 0x00 (off) and 0xFF (on).
 *
 * Oh, and good luck trying to fit a 5mm RGB led in the MegaDrive ;).
 */
#define MODE_LED_EUR_COLOR {0x00, 0xFF, 0x00}  // Green
#define MODE_LED_USA_COLOR {0x00, 0x00, 0xFF}  // Blue
#define MODE_LED_JAP_COLOR {0xFF, 0x00, 0x00}  // Red

// Define this if your led is common-anode, comment out for common-cathode
//#define MODE_LED_COMMON_ANODE

/* Also indicate the video mode with a single led. It is blinked 1-3 times
 * according to which mode is set (1 is EUR, see enum VideoMode below).
 */
#define MODE_LED_SINGLE_PIN 8

/* Presses of the reset button longer than this amount of milliseconds will
 * switch to the next mode, shorter presses will reset the console.
 */
const unsigned long LONGPRESS_LEN = 700U;

// Debounce duration for the reset button
const unsigned long DEBOUNCE_RESET_MS = 20U;

// Duration of the reset pulse (milliseconds)
const unsigned long RESET_LEN = 350U;

/* Button presses will be considered valid only after it has been stable for
 * this amount of milliseconds
 */
const unsigned long DEBOUNCE_BUTTONS_MS = 55U;


/*******************************************************************************
 * DEBUGGING SUPPORT
 ******************************************************************************/

/* Show some information on a 16x2 LCD screen: how the reset line is detected,
 * what buttons are pressed, etc. The screen must be connected via i2c and will
 * be driven with F. Malpartida's New LiquidCrystal library because I like it
 * and it works fine with my display. Get it at:
 * https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
 */
//#define ENABLE_LCD

/* Send debug messages to serial port. This requires Nick Gammon's
 * SendOnlySoftwareSerial library, get it at:
 * https://github.com/nickgammon/SendOnlySoftwareSerial
 */
//#define ENABLE_SERIAL_DEBUG

// Print the controller status on serial. Useful for debugging.
//#define DEBUG_PAD

/*******************************************************************************
 * END OF SETTINGS
 ******************************************************************************/


#ifdef ENABLE_FAST_IO
#include <DigitalIO.h>		// https://github.com/greiman/DigitalIO
#else
#define fastDigitalRead(x) digitalRead(x)
#define fastDigitalWrite(x, y) digitalWrite(x, y)
#define fastPinMode(x, y) pinMode(x, y)
#endif

#ifdef ENABLE_LCD
	#include <LiquidCrystal_I2C.h>

	/* Init LCD - This can vary depending on your display, adapter, etc...
	 * Experiment with the LCD library examples until you find a working
	 * configuration for your display and then port it here.
	 *
	 * This page helped me with many displays:
	 * https://arduinoinfo.mywikis.net/wiki/LCD-Blue-I2C
	 */
	LiquidCrystal_I2C lcd (0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

	#define lcd_start() do {lcd.begin (16, 4); lcd.clear (); lcd.home ();} while (0)
	#define lcd_print(...) lcd.print (__VA_ARGS__)
	#define lcd_print_at(row, col, ...) do {lcd.setCursor (col, row); lcd.print (__VA_ARGS__);} while (0)
	#define lcd_clear() do {lcd.clear (); lcd.home ();} while (0)
#else
	#define lcd_start() do {} while (0)
	#define lcd_print(...) do {} while (0)
	#define lcd_print_at(row, col, ...) do {} while (0)
	#define lcd_clear() do {} while (0)
#endif

#ifdef ENABLE_SERIAL_DEBUG
	#include <SendOnlySoftwareSerial.h>

	SendOnlySoftwareSerial swSerial (1);

	#define dstart(spd) swSerial.begin (spd)
	#define debug(...) swSerial.print (__VA_ARGS__)
	#define debugln(...) swSerial.println (__VA_ARGS__)
#else
	#define dstart(...) do {} while (0)
	#define debug(...) do {} while (0)
	#define debugln(...) do {} while (0)
#endif

#include <EEPROM.h>

enum __attribute__ ((__packed__)) VideoMode {
	EUR,
	USA,
	JAP,
	MODES_NO // Leave at end
};

// This will be handy
#if (defined MODE_LED_R_PIN || defined MODE_LED_G_PIN || defined MODE_LED_B_PIN)
	#define ENABLE_MODE_LED_RGB

const byte mode_led_colors[][MODES_NO] = {
	MODE_LED_EUR_COLOR,
	MODE_LED_USA_COLOR,
	MODE_LED_JAP_COLOR
};
#endif

// Combo detection enable flag
boolean enabled = true;

// Video mode
VideoMode current_mode;
unsigned long mode_last_changed_time = 0;

// Reset level when NOT ACTIVE
byte reset_inactive_level;

/* Raw pad port statuses as read by the ISR:
 * - g_buttons_1 contains the port status when SELECT is HIGH
 * - g_buttons_2 contains the port status when SELECT is LOW
 * - g_buttons_3 contains the port status when SELECT is PULSE-3
 *
 * These are raw readings, since we don't really have any time to spare in the
 * ISR, so HIGH means "Not pressed" here, that's why we start with 0xFFs.
 */
volatile byte g_buttons_1 = 0xFF;
volatile byte g_buttons_2 = 0xFF;
volatile byte g_buttons_3 = 0xFF;

void rgb_led_off () {
#ifdef ENABLE_MODE_LED_RGB
	byte c = 0;

#ifdef RGB_LED_COMMON_ANODE
	c = 255;
#endif

#ifdef MODE_LED_R_PIN
	analogWrite (MODE_LED_R_PIN, c);
#endif

#ifdef MODE_LED_G_PIN
	analogWrite (MODE_LED_G_PIN, c);
#endif

#ifdef MODE_LED_B_PIN
	analogWrite (MODE_LED_B_PIN, c);
#endif

#endif  // ENABLE_MODE_LED_RGB
}

inline void save_mode () {
	if (mode_last_changed_time > 0 && millis () - mode_last_changed_time >= MODE_SAVE_DELAY) {
		debug (F("Saving video mode to EEPROM: "));
		debugln (current_mode);

		byte saved_mode = EEPROM.read (MODE_ROM_OFFSET);
		if (current_mode != saved_mode) {
			EEPROM.write (MODE_ROM_OFFSET, static_cast<byte> (current_mode));
		} else {
			debugln (F("Mode unchanged, not saving"));
		}
		mode_last_changed_time = 0;    // Don't save again

		// Blink led to tell the user that mode was saved
		rgb_led_off ();

		// Keep off for a bit
		delay (200);

		// Turn leds back on
		rgb_led_update ();

#ifdef MODE_LED_SINGLE_PIN
		// Make one long flash
		fastDigitalWrite (MODE_LED_SINGLE_PIN, LOW);
		delay (500);
		fastDigitalWrite (MODE_LED_SINGLE_PIN, HIGH);
#endif
	}
}

inline void change_mode (int increment) {
	// This also loops in [0, MODES_NO) backwards
	VideoMode new_mode = static_cast<VideoMode> ((current_mode + increment + MODES_NO) % MODES_NO);
	set_mode (new_mode, true);
}

inline void next_mode () {
	change_mode (+1);
}

void rgb_led_update () {
#ifdef ENABLE_MODE_LED_RGB
	const byte *colors = mode_led_colors[current_mode];
	byte c;

#ifdef MODE_LED_R_PIN
	c = colors[0];
#ifdef MODE_LED_COMMON_ANODE
	c = 255 - c;
#endif
	analogWrite (MODE_LED_R_PIN, c);
#endif

#ifdef MODE_LED_G_PIN
	c = colors[1];
#ifdef MODE_LED_COMMON_ANODE
	c = 255 - c;
#endif
	analogWrite (MODE_LED_G_PIN, c);
#endif

#ifdef MODE_LED_B_PIN
	c = colors[2];
#ifdef MODE_LED_COMMON_ANODE
	c = 255 - c;
#endif
	analogWrite (MODE_LED_B_PIN, c);
#endif

#endif  // ENABLE_MODE_LED_RGB
}

void flash_single_led () {
#ifdef MODE_LED_SINGLE_PIN
	/* WARNING: This loop must be reasonably shorter than LONGPRESS_LEN in
	 * the worst case!
	 */
	for (byte i = 0; i < current_mode + 1; ++i) {
		fastDigitalWrite (MODE_LED_SINGLE_PIN, LOW);
		delay (40);
		fastDigitalWrite (MODE_LED_SINGLE_PIN, HIGH);
		delay (80);
	}
#endif
}

void set_mode (VideoMode m, boolean save) {
	switch (m) {
		default:
			// Invalid value
			debug (F("ERROR: Tried to set invalid mode: "));
			debugln (m);

			// Get back to something meaningful
			m = EUR;
			// Fall through
		case EUR:
			fastDigitalWrite (VIDEOMODE_PIN, LOW);    // PAL 50Hz
			fastDigitalWrite (LANGUAGE_PIN, HIGH);    // ENG
			lcd_print_at (0, 13, F("EUR"));
			break;
		case USA:
			fastDigitalWrite (VIDEOMODE_PIN, HIGH);   // NTSC 60Hz
			fastDigitalWrite (LANGUAGE_PIN, HIGH);    // ENG
			lcd_print_at (0, 13, F("USA"));
			break;
		case JAP:
			fastDigitalWrite (VIDEOMODE_PIN, HIGH);   // NTSC 60Hz
			fastDigitalWrite (LANGUAGE_PIN, LOW);     // JAP
			lcd_print_at (0, 13, F("JAP"));
			break;
	}

	// Update LCD only now, so that at startup mode is set ASAP
	lcd_print_at (0, 11, F("M:"));

	current_mode = m;
	rgb_led_update ();
	flash_single_led ();

	if (save) {
		mode_last_changed_time = millis ();
	}
}

inline void handle_reset_button () {
	static byte debounce_level = LOW;
	static bool reset_pressed_before = false;
	static unsigned long last_int = 0, reset_press_start = 0;
	static unsigned int hold_cycles = 0;

	byte reset_level = fastDigitalRead (RESET_IN_PIN);
	if (reset_level != debounce_level) {
		// Reset debouncing timer
		last_int = millis ();
		debounce_level = reset_level;
	} else if (millis () - last_int > DEBOUNCE_RESET_MS) {
		// OK, button is stable, see if it has changed
		if (reset_level != reset_inactive_level && !reset_pressed_before) {
			// Button just pressed
			reset_press_start = millis ();
			hold_cycles = 0;
		} else if (reset_level == reset_inactive_level && reset_pressed_before) {
			// Button released
			if (hold_cycles == 0) {
				debugln (F("Reset button pushed for a short time"));
				reset_console ();
			}
		} else {
			// Button has not just been pressed/released
			if (reset_level != reset_inactive_level && millis () - reset_press_start >= LONGPRESS_LEN * (hold_cycles + 1)) {
				// Reset has been held for a while
				debugln (F("Reset button held"));
				++hold_cycles;
				next_mode ();
			}
		}

		reset_pressed_before = (reset_level != reset_inactive_level);
	}
}

void reset_console () {
	lcd_print_at (1, 0, F("  Resetting...  "));

	debugln (F("Resetting console"));

	fastDigitalWrite (RESET_OUT_PIN, !reset_inactive_level);
	delay (RESET_LEN);
	fastDigitalWrite (RESET_OUT_PIN, reset_inactive_level);

	lcd_print_at (1, 0, F("                "));
}

void setup () {
	/* Init video mode: We do this as soon as possible since the MegaDrive's
	 * reset line seems to be edge-triggered, so we cannot hold the console
	 * in the reset state while we are setting up stuff. We'll take care of
	 * the rest later.
	 */
	noInterrupts ();
	fastPinMode (VIDEOMODE_PIN, OUTPUT);
	fastPinMode (LANGUAGE_PIN, OUTPUT);
	current_mode = static_cast<VideoMode> (EEPROM.read (MODE_ROM_OFFSET));
	debug (F("Loaded video mode from EEPROM: "));
	debugln (current_mode);
	set_mode (current_mode, false);		// Don't overwrite EEPROM
	interrupts ();

	// Pheeew, that was quick! Let's go on with the rest!
	dstart (57600);
	debugln (F("Starting up..."));

#ifdef ENABLE_LCD
	// Note that this will hang if interrupts are disabled!
	lcd_start ();
	lcd_print_at (0, 0, F("-= Welcome to =-"));
	lcd_print_at (1, 0, F("-= MegaDrive++=-"));
	delay (2000);
	lcd.clear ();
#endif

/* Rant: As per D4s's installation schematics out there (which we use too), it
 * seems that on consoles with an active low reset signal, the Reset In input
 * is taken before the pull-up resistor, while on consoles with active-high
 * reset it is taken AFTER the pull-down resistor. This means that detecting
 * the reset level by sampling the same line on both consoles is tricky, as in
 * both cases one of the Reset In/Out signals is left floating :(. The
 * following should work reliably, but we allow for a way to force the reset
 * line level.
 */
#ifndef FORCE_RESET_ACTIVE_LEVEL
	// Let things settle down and then sample the reset line
	delay (100);
	fastPinMode (RESET_IN_PIN, INPUT_PULLUP);
	reset_inactive_level = fastDigitalRead (RESET_IN_PIN);
	debug (F("Reset line is "));
	debug (reset_inactive_level ? F("HIGH") : F("LOW"));
	debugln (F(" at startup"));

	lcd_print_at (0, 0, F("R:"));
	lcd_print (reset_inactive_level ? F("HI") : F("LO"));
#else
	reset_inactive_level = !FORCE_RESET_ACTIVE_LEVEL;
	debug (F("Reset line is forced to active-"));
	debugln (FORCE_RESET_ACTIVE_LEVEL ? F("HIGH") : F("LOW"));

	lcd_print_at (0, 0, F("R:"));
	lcd_print (FORCE_RESET_ACTIVE_LEVEL ? F("HIF") : F("LOF"));
#endif

	if (reset_inactive_level == LOW) {
		// No need for pull-up
		fastPinMode (RESET_IN_PIN, INPUT);
#ifdef FORCE_RESET_ACTIVE_LEVEL   // If this is not defined pull-up was already enabled above
	} else {
		fastPinMode (RESET_IN_PIN, INPUT_PULLUP);
#endif
	}

	// Reset console so that it picks up the new mode/lang
	fastPinMode (RESET_OUT_PIN, OUTPUT);
	reset_console ();

	// Setup leds
#ifdef MODE_LED_R_PIN
	fastPinMode (MODE_LED_R_PIN, OUTPUT);
#endif

#ifdef MODE_LED_G_PIN
	fastPinMode (MODE_LED_G_PIN, OUTPUT);
#endif

#ifdef MODE_LED_B_PIN
	fastPinMode (MODE_LED_B_PIN, OUTPUT);
#endif

#ifdef MODE_LED_SINGLE_PIN
	fastPinMode (MODE_LED_SINGLE_PIN, OUTPUT);
#endif

#ifdef PAD_LED_PIN
	fastPinMode (PAD_LED_PIN, OUTPUT);
#endif

	/* Do this again so that leds and LCD get set properly: when we did it
	 * above the led pins had not been set in output mode and the LCD had
	 * not been initialized yet.
	 */
	set_mode (current_mode, false);

	// Prepare to read pad
	setup_pad ();

	// FIXME: Show this on LCD somehow
	if (fastDigitalRead (6) == LOW) {
		// Disable all triggers from controller
		debugln (F("Combo detection disabled"));
		enabled = false;

		// Blink to tell the user
		for (byte i = 0; i < 3; ++i) {
#ifdef ENABLE_MODE_LED_RGB
			rgb_led_off ();
#endif
#ifdef MODE_LED_SINGLE_PIN
			fastDigitalWrite (MODE_LED_SINGLE_PIN, LOW);
#endif
			delay (350);
#ifdef ENABLE_MODE_LED_RGB
			rgb_led_update ();
#endif
#ifdef MODE_LED_SINGLE_PIN
			fastDigitalWrite (MODE_LED_SINGLE_PIN, HIGH);
#endif
			delay (250);
		}
	}

	// We are ready to roll!
	lcd_print_at (1, 0, F("     Ready!     "));
	delay (1000);
	lcd_print_at (1, 0, F("                "));

	debugln (F("Ready!"));
}

void setup_pad () {
	/* Set port directions: All button lines are INPUTs
	 * (Commented out since INPUT is the default state of pins at reset)
	 */
	//~ fastPinMode (0, INPUT);
	//~ fastPinMode (2, INPUT);
	//~ fastPinMode (3, INPUT);
	//~ fastPinMode (4, INPUT);
	//~ fastPinMode (5, INPUT);
	//~ fastPinMode (6, INPUT);
	//~ fastPinMode (7, INPUT);

	/* Enable interrupts: we can't use attachInterrupt() here, since our ISR is
	 * going to be bare
	 */
	EICRA |= (1 << ISC00);    // Trigger interrupt on CHANGE
	EIMSK |= (1 << INT0);     // Enable interrupt 0 (i.e.: on pin 2)
	interrupts ();            // Enable all interrupts, probably redundant
}

/* Clear the pad button registers. This is useful because during resets the
 * SELECT line is not triggered and this will keep triggering resets.
 */
void clear_pad () {
	g_buttons_1 = 0xFF;
	g_buttons_2 = 0xFF;
	g_buttons_3 = 0xFF;

	/* This also looks like a good place to initialize the controller snooping
	 * state machine
	 */
	volatile byte *state = &GPIOR2;
	*state = 0;			// i.e.: PS_INIT (defined in readpad.S)
}

/******************************************************************************/


/* Makes sure that the same button/combo has been pressed steadily for some
 * time.
 */
word debounce_buttons (word buttons) {
	static word currentButtons = 0;
	static word oldButtons = 0;
	static unsigned long pressedOn = 0;

	word ret = currentButtons;

	if (buttons == oldButtons) {
		if (millis () - pressedOn > DEBOUNCE_BUTTONS_MS) {
			// Same combo held long enough
			ret = currentButtons = buttons;
		} else {
			// Combo held not long enough (yet)
		}
	} else {
		// Buttons bouncing
		oldButtons = buttons;
		pressedOn = millis ();
	}

	return ret;
}

/* The basic idea here is to make up a word (i.e.: 2 bytes) where each bit
 * represents the state of a button, with 1 meaning pressed, for commodity's
 * sake. The bit-button mapping is defined in the PadButton enum above.
 */
word read_pad () {
	static unsigned long last_bit_reset = 0;

	/* Invert all bits, since we want to use 1 for pressed
	 *
	 * Note that bit 2 in all of these is the SELECT line.
	 */
	byte b1 = ~g_buttons_1;     // Select HIGH......: UDLRBxxC
	byte b2 = ~g_buttons_2;     // Select LOW.......: UDxxAxxS
	byte b3 = ~g_buttons_3;     // Select PULSE-3...: ZYXMxxxx

	/* Compose all bytes into a single word, respecting the order in PadButton.
	 * Note that we take UP and DOWN from b2 because sometimes b1 will contain
	 * spurious data from b3, i.e.: Keeping X pressed reports LEFT, Y reports
	 * DOWN, etc... This way we restrict the problem to X and MODE.
	 *
	 * It would be great to eliminate the problem completely, but we still
	 * haven't found a way :(.
	 */
	word buttons = (b1 & 0x38) | ((b1 & 0x01) << 2)
	             | (b2 & 0xC0) | ((b2 & 0x08) >> 2) | (b2 & 0x01)
	             | ((b3 & 0xF0) << 4)
	             ;

	if ((g_buttons_3 & 0x04) || (last_bit_reset != 0 && millis () - last_bit_reset < 500))	{
		/* g_buttons_3 is only set by 6-button pads. Since bit 2 will always
		 * read as HIGH (it's the SELECT line), we can use it as a 6-button pad
		 * indicator
		 */
		buttons |= MD_PAD_6BTN;

		if (millis () - last_bit_reset >= 500) {
			/* Set bit 2 of g_buttons_3 to 0. Since g_buttons_3 is only
			 * set by 6-button pads, and since that bit will always read as
			 * HIGH (it's the SELECT line), we can use it as a 6-button pad
			 * indicator
			 */
			g_buttons_3 &= ~(1 << 2);

			last_bit_reset = millis ();
		}
	}

	return buttons;
}


#define IGNORE_COMBO_MS LONGPRESS_LEN

inline void handle_pad () {
	static unsigned long last_combo_time = 0;

	word pad_status = read_pad ();
	pad_status = debounce_buttons (pad_status);

#ifdef PAD_LED_PIN
	fastDigitalWrite (PAD_LED_PIN, (pad_status & ~MD_PAD_6BTN) != 0);
#endif

#ifdef DEBUG_PAD
	static word last_pad_status = 0;

	if (pad_status != last_pad_status) {
		debug (F("Pressed: "));
		if (pad_status & MD_BTN_UP)
			debug (F("Up "));
		if (pad_status & MD_BTN_DOWN)
			debug (F("Down "));
		if (pad_status & MD_BTN_LEFT)
			debug (F("Left "));
		if (pad_status & MD_BTN_RIGHT)
			debug (F("Right "));
		if (pad_status & MD_BTN_A)
			debug (F("A "));
		if (pad_status & MD_BTN_B)
			debug (F("B "));
		if (pad_status & MD_BTN_C)
			debug (F("C "));
		if (pad_status & MD_BTN_X)
			debug (F("X "));
		if (pad_status & MD_BTN_Y)
			debug (F("Y "));
		if (pad_status & MD_BTN_Z)
			debug (F("Z "));
		if (pad_status & MD_BTN_START)
			debug (F("Start "));
		if (pad_status & MD_BTN_MODE)
			debug (F("Mode "));
		debugln ();

		last_pad_status = pad_status;
	}
#endif

	if (pad_status & MD_BTN_UP)
		lcd_print_at (1, 0, 'U');
	else
		lcd_print_at (1, 0, ' ');
	if (pad_status & MD_BTN_DOWN)
		lcd_print_at (1, 1, 'D');
	else
		lcd_print_at (1, 1, ' ');
	if (pad_status & MD_BTN_LEFT)
		lcd_print_at (1, 2, 'L');
	else
		lcd_print_at (1, 2, ' ');
	if (pad_status & MD_BTN_RIGHT)
		lcd_print_at (1, 3, 'R');
	else
		lcd_print_at (1, 3, ' ');
	if (pad_status & MD_BTN_A)
		lcd_print_at (1, 5, 'A');
	else
		lcd_print_at (1, 5, ' ');
	if (pad_status & MD_BTN_B)
		lcd_print_at (1, 6, 'B');
	else
		lcd_print_at (1, 6, ' ');
	if (pad_status & MD_BTN_C)
		lcd_print_at (1, 7, 'C');
	else
		lcd_print_at (1, 7, ' ');
	if (pad_status & MD_BTN_X)
		lcd_print_at (1, 9, 'X');
	else
		lcd_print_at (1, 9, ' ');
	if (pad_status & MD_BTN_Y)
		lcd_print_at (1, 10, 'Y');
	else
		lcd_print_at (1, 10, ' ');
	if (pad_status & MD_BTN_Z)
		lcd_print_at (1, 11, 'Z');
	else
		lcd_print_at (1, 11, ' ');
	if (pad_status & MD_BTN_START)
		lcd_print_at (1, 13, 'S');
	else
		lcd_print_at (1, 13, ' ');
	if (pad_status & MD_BTN_MODE)
		lcd_print_at (1, 15, ' ');
	else
		lcd_print_at (1, 15, ' ');

	if (pad_status & MD_PAD_6BTN)
		lcd_print_at (0, 6, "P:6B");
	else
		lcd_print_at (0, 6, "P:3B");

	if (enabled && (pad_status & TRIGGER_COMBO) == TRIGGER_COMBO && millis () - last_combo_time > IGNORE_COMBO_MS) {
		if ((pad_status & RESET_COMBO) == RESET_COMBO) {
			debugln (F("Reset combo detected"));
			reset_console ();
			clear_pad ();     // Avoid continuous resets
			last_combo_time = millis ();
#ifdef EUR_COMBO
		} else if ((pad_status & EUR_COMBO) == EUR_COMBO) {
			debugln (F("EUR mode combo detected"));
			set_mode (EUR, true);
			last_combo_time = millis ();
#endif
#ifdef USA_COMBO
		} else if ((pad_status & USA_COMBO) == USA_COMBO) {
			debugln (F("USA mode combo detected"));
			set_mode (USA, true);
			last_combo_time = millis ();
#endif
#ifdef JAP_COMBO
		} else if ((pad_status & JAP_COMBO) == JAP_COMBO) {
			debugln (F("JAP mode combo detected"));
			set_mode (JAP, true);
			last_combo_time = millis ();
#endif
		}
	}
}

void loop () {
	handle_reset_button ();
	handle_pad ();
	save_mode ();
}

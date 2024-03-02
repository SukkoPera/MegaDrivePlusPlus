/*******************************************************************************
 * This file is part of MegaDrive++.                                           *
 *                                                                             *
 * Copyright (C) 2015-2024 by SukkoPera <software@sukkology.net>               *
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
 *     JP3/4 (Region) | [X]A2      \  N  /      D7[X] | Pad Port Pin 1
 *   JP1/2 (Language) | [X]A3       \_0_/       D6[X]~| Pad Port Pin 2
 *          [LCD SDA] | [X]A4/SDA               D5[X]~| Pad Port Pin 3
 *          [LCD SCL] | [X]A5/SCL               D4[X] | Pad Port Pin 4
 *                    | [ ]A6              INT1/D3[X]~| Pad Port Pin 6
 *                    | [ ]A7              INT0/D2[X] | Pad Port Pin 7
 *                +5V | [X]5V                  GND[X] | GND
 *                    | [ ]RST                 RST[ ] |
 *                    | [ ]GND   5V MOSI GND   RX0[X] | Pad Port Pin 9
 *                    | [ ]Vin   [ ] [ ] [ ]   TX1[ ] |
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

const byte PIN_UP = 7;
const byte PIN_DOWN = 6;
const byte PIN_LEFT = 5;
const byte PIN_RIGHT = 4;
const byte PIN_B_A = 3;
const byte PIN_C_START = 0;
const byte PIN_SELECT = 2;
const byte PIN_RESET_IN = A1;
const byte PIN_RESET_OUT = A0;
const byte PIN_REGION = A2;
const byte PIN_LANGUAGE = A3;
const byte PIN_PAD_LED = LED_BUILTIN;

// These are kept as #defines so that any can be disabled if unused
#define PIN_REGION_LED_R 9          // PWM
#define PIN_REGION_LED_G 10         // PWM
#define PIN_REGION_LED_B 11         // PWM


/*******************************************************************************
 * BUTTON COMBO SETTINGS
 ******************************************************************************/

// DON'T TOUCH THIS! Just look at it for the button names you can use below!
const word MD_PAD_6BTN  =  1 << 15;	// Not a button, set to 1 if pad is 6-button

const word MD_BTN_Z     = 1 << 11;
const word MD_BTN_Y     = 1 << 10;
const word MD_BTN_X     = 1 << 9;
const word MD_BTN_MODE  = 1 << 8;
const word MD_BTN_UP    = 1 << 7;
const word MD_BTN_DOWN  = 1 << 6;
const word MD_BTN_LEFT  = 1 << 5;
const word MD_BTN_RIGHT = 1 << 4;
const word MD_BTN_B     = 1 << 3;
const word MD_BTN_C     = 1 << 2;
const word MD_BTN_A     = 1 << 1;
const word MD_BTN_START = 1 << 0;

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
//#define ENABLE_FAST_IO

// Force the reset line level when active. Undefine to enable auto-detection.
//#define FORCE_RESET_ACTIVE_LEVEL LOW

// Offset in the EEPROM at which the current region should be saved
#define REGION_ROM_OFFSET 42

// Time to wait after region change before saving the new region (milliseconds)
const unsigned long REGION_SAVE_DELAY = 3000L;

/* Colors to use to indicate the video mode, in 8-bit RGB componentes. Unless
 * you really want weird colors, use only 0x00 (off) and 0xFF (on).
 *
 * Oh, and good luck trying to fit a 5mm RGB led in the MegaDrive ;).
 */
#define REGION_LED_EUR_COLOR {0x00, 0xFF, 0x00}  // Green
#define REGION_LED_USA_COLOR {0x00, 0x00, 0xFF}  // Blue
#define REGION_LED_JAP_COLOR {0xFF, 0x00, 0x00}  // Red

// Define this if your led is common-anode, comment out for common-cathode
//#define REGION_LED_COMMON_ANODE

/* Also indicate the video mode with a single led. It is blinked 1-3 times
 * according to which mode is set (1 is EUR, see enum Region below).
 */
#define PIN_REGION_LED_SINGLE 8

// Undefine to remove support for RGB led
#define ENABLE_REGION_LED_RGB  (defined PIN_REGION_LED_R || defined PIN_REGION_LED_G || defined PIN_REGION_LED_B)

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
#define fastDigitalRead(pin) digitalRead(pin)
#define fastDigitalWrite(pin, level) digitalWrite(pin, level)
#define fastPinMode(pin, mode) pinMode(pin, mode)
#define fastPinConfig(pin, mode, level) {pinMode(pin, mode); digitalWrite(pin, level);}
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

enum class Region: uint8_t {
	EUR,
	USA,
	JAP,
};

const byte REGIONS_NO = 3;

#ifdef ENABLE_REGION_LED_RGB
const byte region_led_colors[][REGIONS_NO] = {
	REGION_LED_EUR_COLOR,
	REGION_LED_USA_COLOR,
	REGION_LED_JAP_COLOR
};
#endif

// Combo detection enable flag
boolean enabled = true;

// Region
Region current_region;
unsigned long region_last_changed_time = 0;

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
#ifdef ENABLE_REGION_LED_RGB
	byte c = 0;

#ifdef RGB_LED_COMMON_ANODE
	c = 255;
#endif

#ifdef PIN_REGION_LED_R
	analogWrite (PIN_REGION_LED_R, c);
#endif

#ifdef PIN_REGION_LED_G
	analogWrite (PIN_REGION_LED_G, c);
#endif

#ifdef PIN_REGION_LED_B
	analogWrite (PIN_REGION_LED_B, c);
#endif

#endif  // ENABLE_REGION_LED_RGB
}

inline void save_region () {
	if (region_last_changed_time > 0 && millis () - region_last_changed_time >= REGION_SAVE_DELAY) {
		debug (F("Saving region to EEPROM: "));
		debugln (current_region);

		Region saved_region = static_cast<Region> (EEPROM.read (REGION_ROM_OFFSET));
		if (current_region != saved_region) {
			EEPROM.write (REGION_ROM_OFFSET, static_cast<byte> (current_region));
		} else {
			debugln (F("Mode unchanged, not saving"));
		}
		region_last_changed_time = 0;    // Don't save again

		// Blink led to tell the user that mode was saved
		rgb_led_off ();

		// Keep off for a bit
		delay (200);

		// Turn leds back on
		rgb_led_update ();

#ifdef PIN_REGION_LED_SINGLE
		// Make one long flash
		fastDigitalWrite (PIN_REGION_LED_SINGLE, LOW);
		delay (500);
		fastDigitalWrite (PIN_REGION_LED_SINGLE, HIGH);
#endif
	}
}

inline void change_region (int increment) {
	// This also loops in [0, REGIONS_NO) backwards
	Region new_region = static_cast<Region> ((static_cast<byte> (current_region) + increment + REGIONS_NO) % REGIONS_NO);
	set_region (new_region, true);
}

inline void next_region () {
	change_region (+1);
}

void rgb_led_update () {
#ifdef ENABLE_REGION_LED_RGB
	const byte *colors = region_led_colors[static_cast<byte> (current_region)];
	byte c;

#ifdef PIN_REGION_LED_R
	c = colors[0];
#ifdef REGION_LED_COMMON_ANODE
	c = 255 - c;
#endif
	analogWrite (PIN_REGION_LED_R, c);
#endif

#ifdef PIN_REGION_LED_G
	c = colors[1];
#ifdef REGION_LED_COMMON_ANODE
	c = 255 - c;
#endif
	analogWrite (PIN_REGION_LED_G, c);
#endif

#ifdef PIN_REGION_LED_B
	c = colors[2];
#ifdef REGION_LED_COMMON_ANODE
	c = 255 - c;
#endif
	analogWrite (PIN_REGION_LED_B, c);
#endif

#endif  // ENABLE_REGION_LED_RGB
}

void flash_single_led () {
#ifdef PIN_REGION_LED_SINGLE
	/* WARNING: This loop must be reasonably shorter than LONGPRESS_LEN in
	 * the worst case!
	 */
	for (byte i = 0; i < static_cast<byte> (current_region) + 1; ++i) {
		fastDigitalWrite (PIN_REGION_LED_SINGLE, LOW);
		delay (40);
		fastDigitalWrite (PIN_REGION_LED_SINGLE, HIGH);
		delay (80);
	}
#endif
}

void set_region (Region m, boolean save) {
	switch (m) {
		default:
			// Invalid value
			debug (F("ERROR: Tried to set invalid region: "));
			debugln (m);

			// Get back to something meaningful
			m = Region::EUR;
			// Fall through
		case Region::EUR:
			fastDigitalWrite (PIN_REGION, LOW);    // PAL 50Hz
			fastDigitalWrite (PIN_LANGUAGE, HIGH);    // ENG
			lcd_print_at (0, 13, F("EUR"));
			break;
		case Region::USA:
			fastDigitalWrite (PIN_REGION, HIGH);   // NTSC 60Hz
			fastDigitalWrite (PIN_LANGUAGE, HIGH);    // ENG
			lcd_print_at (0, 13, F("USA"));
			break;
		case Region::JAP:
			fastDigitalWrite (PIN_REGION, HIGH);   // NTSC 60Hz
			fastDigitalWrite (PIN_LANGUAGE, LOW);     // JAP
			lcd_print_at (0, 13, F("JAP"));
			break;
	}

	// Update LCD only now, so that at startup mode is set ASAP
	lcd_print_at (0, 11, F("M:"));

	current_region = m;
	rgb_led_update ();
	flash_single_led ();

	if (save) {
		region_last_changed_time = millis ();
	}
}

inline void handle_reset_button () {
	static byte debounce_level = LOW;
	static bool reset_pressed_before = false;
	static unsigned long last_int = 0, reset_press_start = 0;
	static unsigned int hold_cycles = 0;

	byte reset_level = fastDigitalRead (PIN_RESET_IN);
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
				next_region ();
			}
		}

		reset_pressed_before = (reset_level != reset_inactive_level);
	}
}

void reset_console () {
	lcd_print_at (1, 0, F("  Resetting...  "));

	debugln (F("Resetting console"));

	fastDigitalWrite (PIN_RESET_OUT, !reset_inactive_level);
	delay (RESET_LEN);
	fastDigitalWrite (PIN_RESET_OUT, reset_inactive_level);

	lcd_print_at (1, 0, F("                "));
}

void setup () {
	/* Init region: We do this as soon as possible since the MegaDrive's
	 * reset line seems to be edge-triggered, so we cannot hold the console
	 * in the reset state while we are setting up stuff. We'll take care of
	 * the rest later.
	 */
	noInterrupts ();
	fastPinMode (PIN_REGION, OUTPUT);
	fastPinMode (PIN_LANGUAGE, OUTPUT);
	current_region = static_cast<Region> (EEPROM.read (REGION_ROM_OFFSET));
	debug (F("Loaded video mode from EEPROM: "));
	debugln (current_region);
	set_region (current_region, false);		// Don't overwrite EEPROM
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
	fastPinMode (PIN_RESET_IN, INPUT_PULLUP);
	reset_inactive_level = fastDigitalRead (PIN_RESET_IN);
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
		fastPinMode (PIN_RESET_IN, INPUT);
#ifdef FORCE_RESET_ACTIVE_LEVEL   // If this is not defined pull-up was already enabled above
	} else {
		fastPinMode (PIN_RESET_IN, INPUT_PULLUP);
#endif
	}

	// Reset console so that it picks up the new mode/lang
	fastPinMode (PIN_RESET_OUT, OUTPUT);
	reset_console ();

	// Setup leds
#ifdef PIN_REGION_LED_R
	fastPinMode (PIN_REGION_LED_R, OUTPUT);
#endif

#ifdef PIN_REGION_LED_G
	fastPinMode (PIN_REGION_LED_G, OUTPUT);
#endif

#ifdef PIN_REGION_LED_B
	fastPinMode (PIN_REGION_LED_B, OUTPUT);
#endif

#ifdef PIN_REGION_LED_SINGLE
	fastPinMode (PIN_REGION_LED_SINGLE, OUTPUT);
#endif

#ifdef PIN_PAD_LED
	fastPinMode (PIN_PAD_LED, OUTPUT);
#endif

	/* Do this again so that leds and LCD get set properly: when we did it
	 * above the led pins had not been set in output mode and the LCD had
	 * not been initialized yet.
	 */
	set_region (current_region, false);

	// Prepare to read pad
	setup_pad ();

	// If DOWN is pressed at startup, disable all combos
	// FIXME: Show this on LCD somehow
	if (fastDigitalRead (PIN_DOWN) == LOW) {
		debugln (F("Combo detection disabled"));
		enabled = false;

		// Blink to tell the user
		for (byte i = 0; i < 3; ++i) {
#ifdef ENABLE_REGION_LED_RGB
			rgb_led_off ();
#endif
#ifdef PIN_REGION_LED_SINGLE
			fastDigitalWrite (PIN_REGION_LED_SINGLE, LOW);
#endif
			delay (350);
#ifdef ENABLE_REGION_LED_RGB
			rgb_led_update ();
#endif
#ifdef PIN_REGION_LED_SINGLE
			fastDigitalWrite (PIN_REGION_LED_SINGLE, HIGH);
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
	// Set port directions: All button lines are INPUTs with pull-ups
	fastPinConfig (PIN_UP, INPUT, 1);
	fastPinConfig (PIN_DOWN, INPUT, 1);
	fastPinConfig (PIN_LEFT, INPUT, 1);
	fastPinConfig (PIN_RIGHT, INPUT, 1);
	fastPinConfig (PIN_B_A, INPUT, 1);
	fastPinConfig (PIN_C_START, INPUT, 1);
	fastPinConfig (PIN_SELECT, INPUT, 1);

	/* Enable interrupts: we can't use attachInterrupt() here, since our ISR is
	 * going to be bare
	 */
	noInterrupts ();
	EICRA |= (1 << ISC00);    // Trigger interrupt on CHANGE
	EIFR |= (1 << INTF0);	  // Clear any pending interrupts
	EIMSK |= (1 << INT0);     // Enable interrupt 0 (i.e.: on pin 2)
	interrupts ();
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

#ifdef PIN_PAD_LED
	fastDigitalWrite (PIN_PAD_LED, (pad_status & ~MD_PAD_6BTN) != 0);
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
			set_region (Region::EUR, true);
			last_combo_time = millis ();
#endif
#ifdef USA_COMBO
		} else if ((pad_status & USA_COMBO) == USA_COMBO) {
			debugln (F("USA mode combo detected"));
			set_region (Region::USA, true);
			last_combo_time = millis ();
#endif
#ifdef JAP_COMBO
		} else if ((pad_status & JAP_COMBO) == JAP_COMBO) {
			debugln (F("JAP mode combo detected"));
			set_region (Region::JAP, true);
			last_combo_time = millis ();
#endif
		}
	}
}

void loop () {
	handle_reset_button ();
	handle_pad ();
	save_region ();
}

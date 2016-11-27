/*******************************************************************************
 * This file is part of MegaDrive++.                                           *
 *                                                                             *
 * Copyright (C) 2015-2016 by SukkoPera <software@sukkology.net>               *
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

/* MegaDrive - Bare ATmega328 Wiring diagram:
 *
 *                    ,-----_-----.
 *                    |1     A5 28| JP1/2 (Language)
 *     Pad Port Pin 9 |2   0 A4 27| JP3/4 (Video Mode)
 *                    |3   1 A3 26| Reset In
 *     Pad Port Pin 7 |4   2 A2 25| Reset Out
 *     Pad Port Pin 6 |5   3 A1 24|
 *     Pad Port Pin 4 |6   4 A0 23|
 *                +5V |7        22| GND
 *                GND |8        21| +5V
 *                    |9        20| +5V
 *                    |10    13 19| (Built-in LED)
 *     Pad Port Pin 3 |11  5 12 18|
 *     Pad Port Pin 2 |12  6 11 17| LED Blue
 *     Pad Port Pin 1 |13  7 10 16| LED Green
 *                    |14  8  9 15| LED Red
 *                    `-----------'
 */

#define RESET_IN_PIN A3
#define RESET_OUT_PIN A2
#define VIDEOMODE_PIN A4
#define LANGUAGE_PIN A5
#define MODE_LED_R_PIN 9
#define MODE_LED_G_PIN 10
#define MODE_LED_B_PIN 11
#define PAD_LED_PIN LED_BUILTIN
#define ENABLE_SERIAL_DEBUG


/*******************************************************************************
 * BUTTON COMBO SETTINGS
 ******************************************************************************/

// DON'T TOUCH THIS! Just look at it for the button names you can use below!
enum __attribute__ ((__packed__)) PadButton {
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

/* Button combos to perform other actions. These are to be considered in
 * addition to TRIGGER_COMBO.
 *
 * Note that we cannot detect certain buttons on some platforms
 */
#define RESET_COMBO (MD_BTN_A | MD_BTN_C)

/* On ATtinyX61's, ATtinyX313's and Arduinos we can detect all buttons, so we
 * can make up a specific combo for every mode that switches straight to it,
 * no need to cycle among modes.
 */
#define EUR_COMBO MD_BTN_DOWN
#define USA_COMBO MD_BTN_RIGHT
#define JAP_COMBO MD_BTN_LEFT


/*******************************************************************************
 * ADVANCED SETTINGS
 ******************************************************************************/

/* Offset in the EEPROM at which the current mode should be saved. Undefine to
 * disable mode saving.
 */
#define MODE_ROM_OFFSET 42

// Time to wait after mode change before saving the new mode (milliseconds)
#define MODE_SAVE_DELAY 5000L

// Force the reset line level when active. Undefine to enable auto-detection.
//#define FORCE_RESET_ACTIVE_LEVEL LOW

/* Colors to use to indicate the video mode, in 8-bit RGB componentes. You can
 * use any value here if your led is connected to PWM-capable pins, otherwise
 * values specified here will be interpreted as either fully off (if 0) or fully
 * on (if anything else).
 *
 * Note that using PWM-values here sometimes causes unpredictable problems. This
 * happened to me on an ATtiny861, and it's probably due to how pins and timers
 * interact. It seems to work fine on a full Arduino, but unless you really want
 * weird colors, use only 0x00 and 0xFF.
 *
 * Oh, and good luck trying to fit a 5mm RGB led in the MegaDrive ;).
 */
#define MODE_LED_EUR_COLOR {0x00, 0xFF, 0x00}  // Green
#define MODE_LED_USA_COLOR {0x00, 0x00, 0xFF}  // Blue
#define MODE_LED_JAP_COLOR {0xFF, 0x00, 0x00}  // Red

// Define this if your led is common-anode, comment out for common-cathode
//#define MODE_LED_COMMON_ANODE

/* Use a single led to indicate the video mode. This is enabled automatically
 * in place of the RGB led when low flash space is detected, but since this
 * does NOT disable the RGB led, it can be used together with it, provided that
 * you have a free pin.
 *
 * Basically, the single led is blinked 1-3 times according to which mode is set
 * (1 is EUR, see enum VideoMode below).
 */
//#define MODE_LED_SINGLE_PIN 3

/* Presses of the reset button longer than this amount of milliseconds will
 * switch to the next mode, shorter presses will reset the console.
 */
#define LONGPRESS_LEN 700

// Debounce duration for the reset button
#define DEBOUNCE_MS 20

// Duration of the reset pulse (milliseconds)
#define RESET_LEN 350

// Print the controller status on serial. Useful for debugging.
#ifdef ENABLE_SERIAL_DEBUG
#define DEBUG_PAD
#endif

/*******************************************************************************
 * END OF SETTINGS
 ******************************************************************************/


#ifdef ENABLE_SERIAL_DEBUG
  #include <SendOnlySoftwareSerial.h>

  SendOnlySoftwareSerial swSerial(1);

  #define dstart(spd) swSerial.begin (spd)
  #define debug(...) swSerial.print (__VA_ARGS__)
  #define debugln(...) swSerial.println (__VA_ARGS__)
#else
  #define dstart(...)
  #define debug(...)
  #define debugln(...)
#endif

#ifdef MODE_ROM_OFFSET
  #include <EEPROM.h>
#endif

extern "C" {
  void readpad ();
}


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

enum __attribute__ ((__packed__)) PadState {
  PS_INIT,            // Initialization
  PS_HI,              // Select is high (U/D/L/R/B/C)
  PS_LO,              // Select is low (U/D/A/Start)

  // The following states are only triggered by 6-button pads
  //~ PS_6BTN_ALL_LO,     // Pins 1/2/3/4 are all LOW
  PS_6BTN_XYZ,        // X/Y/Z/Mode
  PS_6BTN_ALL_HI      // Pins 1/2/3/4 are all HIGH
};

#ifdef LOW_FLASH
  // A bit of hack, but seems to work fine and saves quite a bit of flash memory
  #define analogWrite digitalWrite
#endif


// Video mode
VideoMode current_mode;
unsigned long mode_last_changed_time;

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


inline void save_mode () {
#ifdef MODE_ROM_OFFSET
  if (mode_last_changed_time > 0 && millis () - mode_last_changed_time >= MODE_SAVE_DELAY) {
    debug ("Saving video mode to EEPROM: ");
    debugln (current_mode);

    byte saved_mode = EEPROM.read (MODE_ROM_OFFSET);
    if (current_mode != saved_mode) {
      EEPROM.write (MODE_ROM_OFFSET, static_cast<byte> (current_mode));
    } else {
      debugln ("Mode unchanged, not saving");
    }
    mode_last_changed_time = 0;    // Don't save again

    // Blink led to tell the user that mode was saved
#ifdef ENABLE_MODE_LED_RGB
    byte c = 0;

#ifdef RGB_LED_COMMON_ANODE
    c = 255 - c;
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

    // Keep off for a bit
    delay (200);

    // Turn leds back on
    update_mode_leds ();
#endif  // ENABLE_MODE_LED_RGB

#ifdef MODE_LED_SINGLE_PIN
    // Make one long flash
    digitalWrite (MODE_LED_SINGLE_PIN, LOW);
    delay (500);
    digitalWrite (MODE_LED_SINGLE_PIN, HIGH);
#endif
  }
#endif  // MODE_ROM_OFFSET
}

inline void change_mode (int increment) {
  // This also loops in [0, MODES_NO) backwards
  VideoMode new_mode = static_cast<VideoMode> ((current_mode + increment + MODES_NO) % MODES_NO);
  set_mode (new_mode);
}

inline void next_mode () {
  change_mode (+1);
}

inline void prev_mode () {
  change_mode (-1);
}

void update_mode_leds () {
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

#ifdef MODE_LED_SINGLE_PIN
  // WARNING: This loop must be reasonably shorter than LONGPRESS_LEN in the worst case!
  for (byte i = 0; i < current_mode + 1; ++i) {
    digitalWrite (MODE_LED_SINGLE_PIN, LOW);
    delay (40);
    digitalWrite (MODE_LED_SINGLE_PIN, HIGH);
    delay (80);
  }
#endif
}

void set_mode (VideoMode m) {
  switch (m) {
    default:
    case EUR:
      digitalWrite (VIDEOMODE_PIN, LOW);    // PAL 50Hz
      digitalWrite (LANGUAGE_PIN, HIGH);    // ENG
      break;
    case USA:
      digitalWrite (VIDEOMODE_PIN, HIGH);   // NTSC 60Hz
      digitalWrite (LANGUAGE_PIN, HIGH);    // ENG
      break;
    case JAP:
      digitalWrite (VIDEOMODE_PIN, HIGH);   // NTSC 60Hz
      digitalWrite (LANGUAGE_PIN, LOW);     // JAP
      break;
  }

  current_mode = m;
  update_mode_leds ();

  mode_last_changed_time = millis ();
}

inline void handle_reset_button () {
  static byte debounce_level = LOW;
  static bool reset_pressed_before = false;
  static long last_int = 0, reset_press_start = 0;
  static unsigned int hold_cycles = 0;

  byte reset_level = digitalRead (RESET_IN_PIN);
  if (reset_level != debounce_level) {
    // Reset debouncing timer
    last_int = millis ();
    debounce_level = reset_level;
  } else if (millis () - last_int > DEBOUNCE_MS) {
    // OK, button is stable, see if it has changed
    if (reset_level != reset_inactive_level && !reset_pressed_before) {
      // Button just pressed
      reset_press_start = millis ();
      hold_cycles = 0;
    } else if (reset_level == reset_inactive_level && reset_pressed_before) {
      // Button released
      if (hold_cycles == 0) {
        debugln ("Reset button pushed for a short time");
        reset_console ();
      }
    } else {
      // Button has not just been pressed/released
      if (reset_level != reset_inactive_level && millis () % reset_press_start >= LONGPRESS_LEN * (hold_cycles + 1)) {
        // Reset has been held for a while
        debugln ("Reset button hold");
        ++hold_cycles;
        next_mode ();
      }
    }

    reset_pressed_before = (reset_level != reset_inactive_level);
  }
}

void reset_console () {
  debugln ("Resetting console");

  digitalWrite (RESET_OUT_PIN, !reset_inactive_level);
  delay (RESET_LEN);
  digitalWrite (RESET_OUT_PIN, reset_inactive_level);
}

void setup () {
  dstart (57600);
  debugln ("Starting up...");

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
  pinMode (RESET_IN_PIN, INPUT_PULLUP);
  reset_inactive_level = digitalRead (RESET_IN_PIN);
  debug ("Reset line is ");
  debug (reset_inactive_level ? "HIGH" : "LOW");
  debugln (" at startup");
#else
  reset_inactive_level = !FORCE_RESET_ACTIVE_LEVEL;
  debug ("Reset line is forced to active-");
  debugln (FORCE_RESET_ACTIVE_LEVEL ? "HIGH" : "LOW");
#endif

  if (reset_inactive_level == LOW) {
    // No need for pull-up
    pinMode (RESET_IN_PIN, INPUT);
#ifdef FORCE_RESET_ACTIVE_LEVEL   // If this is not defined pull-up was already enabled above
  } else {
    pinMode (RESET_IN_PIN, INPUT_PULLUP);
#endif
  }

  // Enable reset
  pinMode (RESET_OUT_PIN, OUTPUT);
  digitalWrite (RESET_OUT_PIN, !reset_inactive_level);

  // Setup leds
#ifdef MODE_LED_R_PIN
  pinMode (MODE_LED_R_PIN, OUTPUT);
#endif

#ifdef MODE_LED_G_PIN
  pinMode (MODE_LED_G_PIN, OUTPUT);
#endif

#ifdef MODE_LED_B_PIN
  pinMode (MODE_LED_B_PIN, OUTPUT);
#endif

#ifdef MODE_LED_SINGLE_PIN
  pinMode (MODE_LED_SINGLE_PIN, OUTPUT);
#endif

#ifdef PAD_LED_PIN
  pinMode (PAD_LED_PIN, OUTPUT);
#endif

  // Init video mode
  pinMode (VIDEOMODE_PIN, OUTPUT);
  pinMode (LANGUAGE_PIN, OUTPUT);
  current_mode = EUR;
#ifdef MODE_ROM_OFFSET
  byte tmp = EEPROM.read (MODE_ROM_OFFSET);
  debug ("Loaded video mode from EEPROM: ");
  debugln (tmp);
  if (tmp < MODES_NO) {
    // Palette EEPROM value is good
    current_mode = static_cast<VideoMode> (tmp);
  }
#endif
  set_mode (current_mode);
  mode_last_changed_time = 0;   // No need to save what we just loaded

  // Prepare to read pad
  setup_pad ();

  // Finally release the reset line
  digitalWrite (RESET_OUT_PIN, reset_inactive_level);
}

inline void setup_pad () {
  // Set port directions: All button lines are INPUTs
  DDRD &= ~((1 << DDD7) | (1 << DDD6) | (1 << DDD5) | (1 << DDD4) | (1 << DDD3) | (1 << DDD2) | (1 << DDD0));

  // The SIGNALLING line is an output
  DDRB |= (1 << DDB4);
  PORTB |= (1 << DDB4);

  /* Enable interrupts: we can't use attachInterrupt() here, since our ISR is
   * going to be bare
   */
  EICRA |= (1 << ISC00);    // Trigger interrupt on CHANGE
  EIMSK |= (1 << INT0);     // Enable interrupt 0 (i.e.: on pin 2)
  interrupts ();            // Enable all interrupts, probably redundant
}

/******************************************************************************/


/* The basic idea here is to make up a word (i.e.: 2 bytes) where each bit
 * represents the state of a button, with 1 meaning pressed, for commodity's
 * sake. The bit-button mapping is defined in the PadButton enum above.
 */
word read_pad () {
  // Invert all bits, since we want to use 1 for pressed
  byte b1 = ~g_buttons_1;
  byte b2 = ~g_buttons_2;
  byte b3 = ~g_buttons_3;

  // Compose all bytes into a single word, respecting the order in PadButton
  word buttons = (b1 & 0xF8) | ((b1 & 0x01) << 2)
               | ((b2 & 0x08) >> 2) | (b2 & 0x01)
               | (((word) (b3 & 0xF0)) << 4)
               ;

  return buttons;
}


#define IGNORE_COMBO_MS LONGPRESS_LEN

inline void handle_pad () {
  static long last_combo_time = 0;

  word pad_status = read_pad ();

#ifdef PAD_LED_PIN
  digitalWrite (PAD_LED_PIN, pad_status != 0);
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
    if (pad_status & MD_BTN_MODE)
      debug (F("Mode "));
    if (pad_status & MD_BTN_START)
      debug (F("Start "));
    debugln ();

    last_pad_status = pad_status;
  }
#endif

  if ((pad_status & TRIGGER_COMBO) == TRIGGER_COMBO && millis () - last_combo_time > IGNORE_COMBO_MS) {
    if ((pad_status & RESET_COMBO) == RESET_COMBO) {
      debugln ("Reset combo detected");
      reset_console ();
      pad_status = 0;     // Avoid continuous reset (pad_status might keep the last value during reset!)
      last_combo_time = millis ();
#ifdef EUR_COMBO
    } else if ((pad_status & EUR_COMBO) == EUR_COMBO) {
      debugln ("EUR mode combo detected");
      set_mode (EUR);
      last_combo_time = millis ();
#endif
#ifdef USA_COMBO
    } else if ((pad_status & USA_COMBO) == USA_COMBO) {
      debugln ("USA mode combo detected");
      set_mode (USA);
      last_combo_time = millis ();
#endif
#ifdef JAP_COMBO
    } else if ((pad_status & JAP_COMBO) == JAP_COMBO) {
      debugln ("JAP mode combo detected");
      set_mode (JAP);
      last_combo_time = millis ();
#endif
#ifdef NEXT_MODE_COMBO
    } else if ((pad_status & NEXT_MODE_COMBO) == NEXT_MODE_COMBO) {
      debugln ("Next mode combo detected");
      next_mode ();
      last_combo_time = millis ();
#endif
#ifdef PREV_MODE_COMBO
    } else if ((pad_status & PREV_MODE_COMBO) == PREV_MODE_COMBO) {
      debugln ("Previous mode combo detected");
      prev_mode ();
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

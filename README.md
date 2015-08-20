# MegaDrivePlusPlus
**MegaDrivePlusPlus** is a modchip for the **Sega Mega Drive** (AKA **Sega Genesis**) that enables IGR (*In-Game-Reset*), and 50/60 Hz and Language switching in a... switchless fashion.

## Features
- **Reset-From-Pad** (AKA **In-Game-Reset** AKA **IGR**): Press **Start + A + B + C**.
  - Supports consoles with both active-high and active-low reset signals by
autosensing. Only the former was tested, though :).
- **EUR/USA/JAP mode switching**:
  - Through Reset button: Keep pushed to cycle through modes.
  - From pad: Press **Start + B + Left/Right** to cycle through modes or **Start + Down/Left/Right** to set a certain mode.
  - The last used mode is saved after 10 seconds and reused at power up.
  - Supports common-anode or common-cathode, dual or RGB LEDs to indicate the current mode (Colors can be set to any value when PWM pins are available).
  - Can also indicate the current mode by flashing a single LED, if you don't
feel like replacing the original LED.
- Uses **cheap *Atmel AVR* microcontrollers**.
  - Can be **flashed on different chips** (ATtiny's, ATmega's, or **even a full
Arduino** board), but please note that **not all features are supported on all chips**, depending on
the number of available I/O pins, please read on for details.
  - If flashed on an ATtiny84, it is **pin-to-pin compatibile with the *D4s/Seb
mod***.
- Last but not least, Even though default settings are recommended, **everything can be customized** to taste.

## Choosing a microcontroller
First of all, if you already have an Atmel AVR microcontroller handy, check out below to see if that particular chip is supported and if the features you are interested in are available on it.

If you have to buy a new chip from scratch I'd strongly recommend getting an
**Arduino Nano clone**, because:
- It is cheap (2-3 EUR straight from China).
- It has plenty of pins, which means it supports all the features.
- It has an embedded USB port and serial converter, allowing for easy
upgrades (and debugging, in case).

An **ATtiny861** is also a valid choice, if you prefer sticking to a single chip,
as it has plenty of I/O pins and it supports all features..

## Supported microcontrollers
All diagrams below for ATtiny chips are based on ATTinyCore:
https://github.com/SpenceKonde/ATTinyCore. Probably other cores will work as well, but you might have to adjust pin numbers and *#defines*, so please stick to ATTinyCore..

NOTE: In all diagrams, outer pin number are referred to the physical chips,
while inner pin numbers are Arduino pin numbers.

[To Be Continued..]

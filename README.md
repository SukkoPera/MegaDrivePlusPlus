## MegaDrive++
MegaDrive++ is a modchip for the Sega Mega Drive (AKA Genesis) with Region switching, 50/60 Hz toggling and In-Game-Reset (IGR) functionalities.

### Features
MegaDrive++ has the following features:

- **EUR/USA/JAP mode switching**: this effectively **makes your console universal**, allowing it to **bypass region checks** and to **run all games** without resorting to an adapter.
  - If you come from a PAL region, you will also be able to **run most games at 60 Hz**, which means **full-speed** and **full-screen**! Get rid of those **black bars**! See the difference [here](https://youtu.be/X1CW8Da8i1o)!
  - The mod is **switchless**, so you don't need to modify the aesthetics of your console installing ugly switches, but rather you will be able to change the region:
    - Through the <kbd>Reset</kbd> button: **Keep pushed** to cycle through modes.
    - From the Player 1 controller pad: Press <kbd>Start</kbd> + <kbd>B</kbd> + <kbd>Down</kbd>/<kbd>Left</kbd>/<kbd>Right</kbd> to set your desired mode.
  - The last used mode is saved automatically after 5 seconds and reused at power up.
  - Supports a single, dual or RGB LED to indicate the current mode.
- **Reset-From-Pad** (AKA **In-Game-Reset** AKA **IGR**): Press <kbd>Start</kbd> + <kbd>A</kbd> + <kbd>B</kbd> + <kbd>C</kbd>.
  - Supports consoles with both active-high and active-low reset signals by autosensing (i.e.: **all console revisions**!).
- Uses a **cheap Arduino Nano board**.
- Uses the popular **Arduino environment**, allowing for easy development, testing and modifications.
- Even though default settings are recommended, **everything can be customized** to taste.
- Last but not least, it is **Open Source and Free Software**!

### Branches
If you were a MegaDrive++ user before May 2019, please note that a few things have changed:
- The old *6button*/*lcd_support* branch has become the *master* branch: **this is now the recommended version** to use and the only supported hardware is the Arduino Nano.
- The old *master* branch has become the *legacy* branch: this is only kept for reference, it is no longer supported and won't get any updates in the future.

### Installation
If you are interested in modding your console with MegaDrive++, please [head to the wiki](https://github.com/SukkoPera/MegaDrivePlusPlus/wiki). There you will find full instructions about what chip to buy, how to put MegaDrive++ on it and how to install it, with a full wiring guide for a few different Mega Drive/Genesis models that were sold.

### License
MegaDrive++ is Copyright &copy; 2015-2019 by SukkoPera.

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program. If not, see <http://www.gnu.org/licenses/>.

### Support the Project
If you want to support the development of this project, buy me a coffee:

<a href='https://ko-fi.com/L3L0U18L' target='_blank'><img height='36' style='border:0px;height:36px;' src='https://az743702.vo.msecnd.net/cdn/kofi2.png?v=2' border='0' alt='Buy Me a Coffee at ko-fi.com' /></a>

### Get Help
If you need help or have questions, you can join [the official Telegram group](https://t.me/joinchat/HUHdWBC9J9JnYIrvTYfZmg).

### Thanks
- TBD

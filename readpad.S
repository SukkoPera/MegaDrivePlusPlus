; ******************************************************************************
; * This file is part of MegaDrive++.                                          *
; *                                                                            *
; * Copyright (C) 2015-2019 by SukkoPera <software@sukkology.net>              *
; *                                                                            *
; * MegaDrive++ is free software: you can redistribute it and/or modify        *
; * it under the terms of the GNU General Public License as published by       *
; * the Free Software Foundation, either version 3 of the License, or          *
; * (at your option) any later version.                                        *
; *                                                                            *
; * MegaDrive++ is distributed in the hope that it will be useful,             *
; * but WITHOUT ANY WARRANTY; without even the implied warranty of             *
; * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
; * GNU General Public License for more details.                               *
; *                                                                            *
; * You should have received a copy of the GNU General Public License          *
; * along with MegaDrive++. If not, see <http://www.gnu.org/licenses/>.        *
; ******************************************************************************
;
; MegaDrive++ - Universal Region mod, 50/60 Hz switch and In-Game-Reset (IGR)
; for Sega Mega Drive (AKA Genesis)
;
; Please refer to the GitHub page and wiki for any information:
; https://github.com/SukkoPera/MegaDrivePlusPlus
;
; General ISR writing tips: http://nerdralph.blogspot.it/2014/07/writing-avr-interrupt-service-routines.html
; Instruction reference: http://www.atmel.com/webdoc/avrassembler/avrassembler.wb_RJMP.html
; Register usage: https://gcc.gnu.org/wiki/avr-gcc#Register_Layout

#include <avr/io.h>

.extern g_buttons_1
.extern g_buttons_2
.extern g_buttons_3

.section .text




#define PAD_IOPORT _SFR_IO_ADDR (PIND)
#define SEL_BIT PIND2

; The following are bit positions
#define PS_INIT 		0	; Initialization
#define PS_HI			1	; Select is high (U/D/L/R/B/C)
#define PS_LO			2	; Select is low (U/D/A/Start)
;  The following states are only triggered by 6-button pads
#define PS_6BTN_XYZ		3	; X/Y/Z/Mode
#define PS_6BTN_ALL_HI	4	; Pins 1/2/3/4 are all HIGH

#define REGWRK r26

; Note that GPIOR0 is one of the lowest 64 registers on ATmega88/168/328's, thus
; it can be used with SBI/CBI/SBIC/SBIS
#define IO_GPIOR0 _SFR_IO_ADDR (GPIOR0)
#define IO_TREG1 _SFR_IO_ADDR (GPIOR1)
#define IO_TREG2 _SFR_IO_ADDR (GPIOR2)

.global INT0_vect
INT0_vect:
						; Save context
						out IO_TREG1, REGWRK				; GPIOR1 = r26
						in REGWRK, _SFR_IO_ADDR (SREG)		; r26 = SREG
						out IO_TREG2, REGWRK				; GPIOR2 = r26 = SREG

						; Read controller, all at once!
						in REGWRK, PAD_IOPORT

						; State machine start
						sbic IO_GPIOR0, PS_LO
						rjmp ps_lo
						sbic IO_GPIOR0, PS_HI
						rjmp ps_hi
						sbic IO_GPIOR0, PS_6BTN_XYZ
						rjmp ps_xyz
						sbic IO_GPIOR0, PS_6BTN_ALL_HI
						rjmp ps_all_hi
						; If state is INIT or any unknown state, we'll fall through

						; GPIOR0 is no longer needed after this point, so it can be used as a scratch register

ps_init:
						out IO_GPIOR0, REGWRK			; GPIOR0 = Buttons
						ldi REGWRK, 1 << PS_LO			; Assume select is HIGH, it will be low next time we're called
						sbis IO_GPIOR0, SEL_BIT
						ldi REGWRK, 1 << PS_HI			; Whoops, it was LOW, so it will be high next time
						rjmp sm_end

ps_hi:
						sbrs REGWRK, SEL_BIT        	; If SELECT is not HIGH...
						rjmp sm_reset               	; ... Reset SM
						sts g_buttons_1, REGWRK     	; Save port status to g_buttons_1
						ldi REGWRK, 1 << PS_LO         	; SELECT will be low next time we're called
						rjmp sm_end

ps_lo:
						sbrc REGWRK, SEL_BIT        	; SELECT must be LOW
						rjmp sm_reset
						out IO_GPIOR0, REGWRK			; GPIOR0 = Buttons
						andi REGWRK, 0xF0				; Check if U/D/L/R are all low at the same time (This sets SREG flags)
						breq have6btn               	; If == 0, U/D/L/R are all low, this means we have a 6-button pad
						in REGWRK, IO_GPIOR0			; Else, we have Start & A (and UP & DOWN, in case)
						sts g_buttons_2, REGWRK
						ldi REGWRK, 1 << PS_HI
						rjmp sm_end
have6btn:				ldi REGWRK, 1 << PS_6BTN_XYZ	; No useful data at this point with 6-button pads, just go to next state
						rjmp sm_end

ps_xyz:
						sbrs REGWRK, SEL_BIT        	; SELECT must be HIGH
						rjmp sm_reset
						sts g_buttons_3, REGWRK
						ldi REGWRK, 1 << PS_6BTN_ALL_HI
						rjmp sm_end

ps_all_hi:
						sbrc REGWRK, SEL_BIT        	; SELECT must be LOW
						rjmp sm_reset
						ldi REGWRK, 1 << PS_HI
						rjmp sm_end

sm_reset:
						; SELECT did not have the expected level, reset SM
						ldi REGWRK, 1 << PS_INIT
						; Fall through to sm_end

sm_end:					; End of state machine: We shall always get here with the next state in REGWRK
						out IO_GPIOR0, REGWRK			; Update next state

						; Restore context
						in REGWRK, IO_TREG2				; r26 = GPIO2 = Saved SREG
						out _SFR_IO_ADDR (SREG), REGWRK
						in REGWRK, IO_TREG1				; R26 = GPIOR1 = Saved R26

						reti							; All done!

; ******************************************************************************
; * This file is part of MegaDrive++.                                          *
; *                                                                            *
; * Copyright (C) 2015-2016 by SukkoPera <software@sukkology.net>              *
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
; State machine implemented as per: http://www.avrfreaks.net/forum/how-do-you-make-jump-table-avr-assembly

#include <avr/io.h>


.section .data
state:      .byte 0

.extern g_buttons_1
.extern g_buttons_2
.extern g_buttons_3

.section .text

.global INT0_vect


#define PAD_IOPORT _SFR_IO_ADDR (PIND)
#define SEL_BIT PIND2
#define SIG_IOPORT _SFR_IO_ADDR (PORTB)
#define SIG_BIT DDB4

;~ #define ENABLE_SIGNALLING

#define PS_INIT 0          ; Initialization
#define PS_HI 1            ; Select is high (U/D/L/R/B/C)
#define PS_LO 2            ; Select is low (U/D/A/Start)
;  The following states are only triggered by 6-button pads
#define PS_6BTN_XYZ 3      ; X/Y/Z/Mode
#define PS_6BTN_ALL_HI 4   ; Pins 1/2/3/4 are all HIGH

#define R_PORT r24
#define R_STATE r26


INT0_vect:  ; Save context
						push R_PORT
						in R_PORT, _SFR_IO_ADDR (SREG)
						push R_PORT
						push R_STATE

#ifdef ENABLE_SIGNALLING
						sbi SIG_IOPORT, SIG_BIT
#endif

						in R_PORT, PAD_IOPORT

#ifdef ENABLE_SIGNALLING
						cbi SIG_IOPORT, SIG_BIT
						sbi SIG_IOPORT, SIG_BIT
#endif

						; State machine start
						; Note that we do NOT implement a jump table with ijmp here, as it
						; would actually be slower!
						lds R_STATE, state
						cpi R_STATE, 0
						breq ps_init
						dec R_STATE
						breq ps_hi
						dec R_STATE
						breq ps_lo
						dec R_STATE
						breq ps_xyz
						dec R_STATE
						breq ps_all_hi

ps_init:
						sbrs R_PORT, SEL_BIT
						rjmp init_hi
						ldi R_STATE, PS_LO          ; Select is high, it will be low next time we're called
						rjmp sm_end
init_hi:    ldi R_STATE, PS_HI
						rjmp sm_end

ps_hi:
						sbrs R_PORT, SEL_BIT        ; If SELECT is not HIGH...
						rjmp sm_reset               ; ... Reset SM
						sts g_buttons_1, R_PORT     ; Save port status to g_buttons_1
						ldi R_STATE, PS_LO          ; SELECT will be low next time we're called
						rjmp sm_end

ps_lo:
						sbrc R_PORT, SEL_BIT        ; SELECT must be LOW
						rjmp sm_reset
						mov R_STATE, R_PORT         ; Abuse R_STATE, we don't need it at this time
						andi R_STATE, 0xF0          ; Check if U/D/L/R are all low at the same time (This sets SREG flags)
						breq have6btn               ; If == 0, U/D/L/R are all low, this means we have a 6-button pad
						sts g_buttons_2, R_PORT     ; Else, we have Start & A (and UP & DOWN, in case)
						ldi R_STATE, PS_HI
						rjmp sm_end
have6btn:   ldi R_STATE, PS_6BTN_XYZ    ; No useful data at this point with 6-button pads, just go to next state
						rjmp sm_end

ps_xyz:
						sbrs R_PORT, SEL_BIT        ; SELECT must be HIGH
						rjmp sm_reset
						sts g_buttons_3, R_PORT
						ldi R_STATE, PS_6BTN_ALL_HI
						rjmp sm_end

ps_all_hi:
						sbrc R_PORT, SEL_BIT        ; SELECT must be LOW
						rjmp sm_reset
						ldi R_STATE, PS_HI
						rjmp sm_end

sm_reset:   ; SELECT did not have the expected level, reset SM
#ifdef ENABLE_SIGNALLING
						cbi SIG_IOPORT, SIG_BIT
						sbi SIG_IOPORT, SIG_BIT
						cbi SIG_IOPORT, SIG_BIT
						sbi SIG_IOPORT, SIG_BIT
						cbi SIG_IOPORT, SIG_BIT
						sbi SIG_IOPORT, SIG_BIT
#endif
						ldi R_STATE, PS_INIT
						; rjmp sm_end

						; End of state machine, save state to RAM
sm_end:     sts state, R_STATE

#ifdef ENABLE_SIGNALLING
						cbi SIG_IOPORT, SIG_BIT
#endif

						; Restore context
						pop R_STATE
						pop R_PORT
						out _SFR_IO_ADDR (SREG), R_PORT
						pop R_PORT

						reti

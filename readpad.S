; General tips: http://nerdralph.blogspot.it/2014/07/writing-avr-interrupt-service-routines.html
; Instruction reference: http://www.atmel.com/webdoc/avrassembler/avrassembler.wb_RJMP.html
; Register usage: https://gcc.gnu.org/wiki/avr-gcc#Register_Layout
; State machine implemented as per: http://www.avrfreaks.net/forum/how-do-you-make-jump-table-avr-assembly

; Call-used registers (r18-r27, r30-r31): May be allocated by gcc for local data. You may use them freely in assembler subroutines. Calling C subroutines can clobber any of them - the caller is responsible for saving and restoring.
; Call-saved registers (r2-r17, r28-r29): May be allocated by gcc for local data. Calling C subroutines leaves them unchanged. Assembler subroutines are responsible for saving and restoring these registers, if changed. r29:r28 (Y pointer) is used as a frame pointer (points to local data on stack) if necessary. The requirement for the callee to save/preserve the contents of these registers even applies in situations where the compiler assigns them for argument passing.
; Fixed registers (r0, r1): Never allocated by gcc for local data, but often used for fixed purposes:
; r0 - temporary register, can be clobbered by any C code (except interrupt handlers which save it), may be used to remember something for a while within one piece of assembler code
; r1 - assumed to be always zero in any C code, may be used to remember something for a while within one piece of assembler code, but must then be cleared after use (clr r1). This includes any use of the [f]mul[s[u]] instructions, which return their result in r1:r0. Interrupt handlers save and clear r1 on entry, and restore r1 on exit (in case it was non-zero).


#include <avr/io.h>

; correct for avr/io.h 0x20 port offset for I/O instructions
;~ #define ioPort(x) (x - 0x20)

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

#define ENABLE_SIGNALLING

#define PS_INIT 0          ; Initialization
#define PS_HI 1            ; Select is high (U/D/L/R/B/C)
#define PS_LO 2            ; Select is low (U/D/A/Start)
;  The following states are only triggered by 6-button pads
#define PS_6BTN_XYZ 3      ; X/Y/Z/Mode
#define PS_6BTN_ALL_HI 4   ; Pins 1/2/3/4 are all HIGH

#define R_PORT r24
#define R_STATE r26


INT0_vect:	; Save context
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
init_hi:		ldi R_STATE, PS_HI
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
						push R_PORT									; Save R_PORT temporarily
            andi R_PORT, 0xF0						; Check if U/D/L/R are all low at the same time (This sets SREG flags)
            pop R_PORT									; Restore old R_PORT (Does not change SREG)
            breq have6btn								; If == 0, U/D/L/R are all low, this means we have a 6-button pad
            sts g_buttons_2, R_PORT			; Else, we have Start & A (and UP & DOWN, in case)
            ldi R_STATE, PS_HI
            rjmp sm_end
have6btn:		ldi R_STATE, PS_6BTN_XYZ		; No useful data at this point with 6-button pads, just go to next state
            rjmp sm_end

ps_xyz:
						sbrs R_PORT, SEL_BIT				; SELECT must be HIGH
						rjmp sm_reset
            sts g_buttons_3, R_PORT
            ldi R_STATE, PS_6BTN_ALL_HI
            rjmp sm_end

ps_all_hi:
						sbrc R_PORT, SEL_BIT				; SELECT must be LOW
						rjmp sm_reset
            ldi R_STATE, PS_HI
            rjmp sm_end

sm_reset:		; SELECT did not have the expected level, reset SM
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

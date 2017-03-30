; -----------------------------------------------------------------------------
; Recoil Gun Base Station Power control chip
; (c) 2017 HotGen Ltd.
; Written for the TR4P151AF chip
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Note about options on the TR4P151AF
;    Security: is unlock (can be locked i think)
;    WDT: is disabled (can be enabled i think)
;    RTCEN: is disabled (unused)
;    PA3: is PA3 (unused pin, probably best with this schematic)
;    XT32ENB: disabled (must be disabled with this schematic)
;    EXT_OSC: disabled (must be disabled with this schematic)
;    ADJ: IO (PB0 must be I/O - non-default option)
;    IADJ: is 0
;    MCK: is 8 MHz (if this changes then the timing constants e.g. Tim2_Speed needs to change)
;    OSC: is 14.33 kHz (green mode unused)
;    IR: disabled (must be disabled since PA1 is an input)
; -----------------------------------------------------------------------------

; *****************************************************************************
; Notes about assembly language
;   jc = jump if less than
;   jnc = jump if greater or equal
; *****************************************************************************

; -----------------------------------------------------------------------------
; PINOUT
; 1. VCC
; 2. PB1 = USR        (active high input)
; 3. PB0 = PWR        (active high output)
; 4. PA3 = unused
; 
; 5. PA2 = LED        (active low output)
; 6. PA1 = PWR_CTRL_1 (active low input) (was active high)
; 7. PA0 = PWR_CTRL_2 (active high output)
; 8. GND
; 
; GPIO
; PA0 = PWR_CTRL_2/LED3 = output tx
; PA1 = PWR_CTRL_1/LED2 = input rx (low when button pressed on board)
; PA2 = LED             = output high (on or flashing)
; PA3 = unused          = input pull high
; PB0 = PWR             = output high
; PB1 = USR             = input pull low (high when button pressed on board)
; 
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Behaviour of inputs
; 
; PWR_CTRL_1 signal = (input from cpu) - CPU should raise this to turn the power off, then lower it when it is done.
; USR = press this button for a small amount of time in order to trigger power off
; 
; -----------------------------------------------------------------------------
; Behaviour of outputs
; 
; PWR = On power on, turn this on.
;       On power off turn this off (after the LED sequence).
; LED = on power on, turn on after a short delay
;       on power off, flash a few times and turn off
; PWR_CTRL_2 signal = (output to cpu) - we raise this to warn the CPU of imminent power off
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Defines
; -----------------------------------------------------------------------------


; -----------------------------------------------------------------------------
; Include Block
; -----------------------------------------------------------------------------

#include     "power.inc"

; -----------------------------------------------------------------------------
; Hardware register values
; -----------------------------------------------------------------------------
WDT_KICK	equ	5	; Value to kick the watchdog with

; -----------------------------------------------------------------------------
; User defined "registers" (SRAM variables)
; -----------------------------------------------------------------------------
#define Optimal_MAH_PCH		; System MAH + PCH Optimal (Optimal MAH and PCH)
VARRM = {
; -------------------------------------
; System variables
A_SRT           ; Save the A register during an interrupt
g_timer0	; Counts up every   0.1ms
g_timer1	; Counts up every   1.6ms
g_timer2	; Counts up every  25.6ms
g_timer3	; Counts up every 409.6ms
ButtonCnt0	; How many samples 1.6ms of the button have been off?
ButtonCnt1	; How many samples 1.6ms of the button have been on?
ButtonOn	; Is the button on (1) or off (0)?
ButtonPress	; Has the button been pressed?
ButtonInit      ; Are we in the initial phase (where the button might be on)?
PowerOff	; Power off the device
g_pwm		; How bright the LED should be (15=full)

off_timer0	; Counts up every   1.6ms
off_timer1	; Counts up every  25.6ms
off_phase0	; Counts up every 307.2ms (phases: LED 0=off 1=on 2=off 3=on 4=off 5=on 6=poweroff)
off_phase1	; 
cpu_rx		; The value (0 or BIT_RX) of the cpu at the time the decision was made to power off
rx_timer0	; Counts up every   1.6ms
rx_timer1	; Counts up every  25.6ms

cpu_last_rx	; The last value (0 or BIT_RX) of the input pin
rxb_timer0	; Counts up every   0.1ms
rxb_timer1	; Counts up every   1.6ms
rxb_timer2	; Counts up every  25.6ms
rxb_timer3	; Counts up every 409.6ms
rxb_bits	; The input payload
rxb_count	; How many payload bits have been received
rxb_cmd		; A received command (or 0=none)
rxb_err0	; How many errors have I received?
rxb_err1	; How many errors have I received?
}

; -----------------------------------------------------------------------------
; General Parameters for the program
VINT_MAH        equ	00    ; The SRAM bank we want to use inside the interrupt [Mind you, Optima overrides this]
Tim2_Speed	equ	256-100	; 100uS = 10kHz

PORT_PWR	equ	data_pb
PIN_PWR		equ	0
BIT_PWR		equ	1 << PIN_PWR

PORT_USR	equ	data_pb
PIN_USR		equ	1
BIT_USR		equ	1 << PIN_USR

PORT_LED	equ	data_pa
PIN_LED		equ	2
BIT_LED		equ	1 << PIN_LED

PORT_RX		equ	data_pa
PIN_RX		equ	1
BIT_RX		equ	1 << PIN_RX
XOR_RX		equ	BIT_RX ; Either BIT_RX for inverted or 0 for normal

PORT_TX		equ	data_pa
PIN_TX		equ	0
BIT_TX		equ	1 << PIN_TX

; Timing parameters
TIMEOUT_FAST	equ	6	; Timeout if CPU acknowledges me
TIMEOUT_SLOW	equ	30	; Timeout if CPU does not acknowledge
TIMEOUT_PHASE	equ	192	; Timeout in 1.6ms units between phases (~300ms)
TIMEOUT_RX	equ	64	; Timeout in 1.6ms units for CPU RX to cause power down
SAT_HIGH	equ	15	; Timeout in 1.6ms units for button press to transition to high
SAT_LOW		equ	15	; Timeout in 1.6ms units for button press to transition to low

; Timing for the rx protocol
TIME_STEP	equ	500	; Time in 0.1ms units (50ms base tick)
TIME_SLOP	equ	200	; Allow 20ms slop each way (10% slop on 4 unit periods)
TIME_ONE_MIN	equ	TIME_STEP*1-TIME_SLOP	; 
TIME_ONE_MAX	equ	TIME_STEP*1+TIME_SLOP	; 
TIME_TWO_MIN	equ	TIME_STEP*2-TIME_SLOP	; 
TIME_TWO_MAX	equ	TIME_STEP*2+TIME_SLOP	; 
TIME_FOUR_MIN	equ	TIME_STEP*4-TIME_SLOP	; 
TIME_FOUR_MAX	equ	TIME_STEP*4+TIME_SLOP	; 
TIME_IDLE_MIN	equ	TIME_STEP*6		; Not used; assumed to be 16*16*16 = 409.6ms

; Commands for the other side to send
CMD_NULL	equ	0	; Bits 0x001EA on the wire
CMD_REBOOTING	equ	1	; Bits 0x003D6 on the wire
CMD_BOOTED	equ	2	; Bits 0x003D2 on the wire
CMD_ACK_POWER	equ	3	; Bits 0x007A6 on the wire
CMD_ERROR_1	equ	8	; Bits 0x003CA on the wire
CMD_ERROR_2	equ	10	; Bits 0x00792 on the wire
CMD_ERROR_3	equ	11	; Bits 0x00F26 on the wire
CMD_POWER_OFF	equ	13	; Bits 0x00F36 on the wire



; -----------------------------------------------------------------------------
; PROGRAM Entry points
; -----------------------------------------------------------------------------

	; (0) Entry point for program start
	org	0
	ldpch	PGMSRT
	jmp	PGMSRT
	nop
	nop
	
	; (4) Entry point for wakeup
	ldpch	WakeUp
	jmp	WakeUp
	nop
	nop

	; (8) Entry point for interrupt start (Timer1, Timer2, RTC)
	ldpch	INT_Start
	jmp	INT_Start

; -----------------------------------------------------------------------------
; Main Interrupt routine for Timer1, Timer2, RTC 
; Hardware will store the following registers and restore them on RETI
;	MAH = Memory address high
;       PCH = Program counter high
;       PCL = Program counter low
;       CZ  = Carry and Zero flags of the status register
;       ENINT = ENINT flag of the SYS0 register
; Cannot call subroutines; must preserve other registers (e.g. A)
; -----------------------------------------------------------------------------
INT_Start:
	ldmah	#VINT_MAH ; (Optima will do this anyway)
	ld	(A_SRT),A ; Preserve the A register over the interrupt

	; Despatch the interrupt
	; Is this interrupt from the real time clock? (once per second)
	ld	a,(RTC)
	and	A,#1000B;RTCFG/F38K/RTCS1/RTCS0
	jz	RTC_Acked
	; This interrupt is from the real time clock (not important)
	clr	#3,(RTC) ; Clear the real time clock overflow flag
RTC_Acked:

	; Is this interrupt from Timer1? (should not happen)
	ld	A,(STATUS);TM2IFG/TM1IFG/CF/ZF
	and	A,#0100B
	jz	Timer1_Acked
	clr	#2,(STATUS) ; Clear the timer1 interrupt flag
Timer1_Acked:
	
	; Is this interrupt from Timer2?
	ld	a,(STATUS)
	and	A,#1000B
	jz	Timer2_Acked

	; Timer 2 interrupt
	clr	#3,(STATUS) ; Clear the timer2 interrupt flag
	inc	(g_timer0)
	adr	(g_timer1)
	adr	(g_timer2)
	adr	(g_timer3)
	
	; Support fake PWM on the LED
	ld	A,(g_timer0)
	cmp	A,(g_pwm)
	jc	PwmLedOn ; jump on less
	set	#PIN_LED,(PORT_LED) ; Turn off the LED
	jmp	PwmLedDone
PwmLedOn:
	clr	#PIN_LED,(PORT_LED)
PwmLedDone:

	; Look at the rx bits
	inc	(rxb_timer0)
	adr	(rxb_timer1)
	adr	(rxb_timer2)
	jc	RxOverflow ; We have overflowed (409.6ms) so this is idle

	; Look at the bits	
	ld	a,(PORT_RX)
	and	a,#BIT_RX
	cmp	A,(cpu_last_rx)
	jz	RxNotChanged
	; A transition has occurred
	ld	(cpu_last_rx),A

	; Compare with IDLE time
	ld	A,(rxb_timer3)
	jnz	RxReset

	; Compare with TIME_ONE_MIN
	ld	A,(rxb_timer0)
	cmp	A,#TIME_ONE_MIN.n0
	ld	A,(rxb_timer1)
	sbc	A,#TIME_ONE_MIN.n1
	ld	A,(rxb_timer2)
	sbc	A,#TIME_ONE_MIN.n2
	jc	RxError

	; Compare with TIME_ONE_MAX
	ld	A,(rxb_timer0)
	cmp	A,#TIME_ONE_MAX.n0
	ld	A,(rxb_timer1)
	sbc	A,#TIME_ONE_MAX.n1
	ld	A,(rxb_timer2)
	sbc	A,#TIME_ONE_MAX.n2
	jc	RxOne

	; Compare with TIME_TWO_MIN
	ld	A,(rxb_timer0)
	cmp	A,#TIME_TWO_MIN.n0
	ld	A,(rxb_timer1)
	sbc	A,#TIME_TWO_MIN.n1
	ld	A,(rxb_timer2)
	sbc	A,#TIME_TWO_MIN.n2
	jc	RxError

	; Compare with TIME_TWO_MAX
	ld	A,(rxb_timer0)
	cmp	A,#TIME_TWO_MAX.n0
	ld	A,(rxb_timer1)
	sbc	A,#TIME_TWO_MAX.n1
	ld	A,(rxb_timer2)
	sbc	A,#TIME_TWO_MAX.n2
	jc	RxTwo

	; Compare with TIME_FOUR_MIN
	ld	A,(rxb_timer0)
	cmp	A,#TIME_FOUR_MIN.n0
	ld	A,(rxb_timer1)
	sbc	A,#TIME_FOUR_MIN.n1
	ld	A,(rxb_timer2)
	sbc	A,#TIME_FOUR_MIN.n2
	jc	RxError

	; Compare with TIME_FOUR_MAX
	ld	A,(rxb_timer0)
	cmp	A,#TIME_FOUR_MAX.n0
	ld	A,(rxb_timer1)
	sbc	A,#TIME_FOUR_MAX.n1
	ld	A,(rxb_timer2)
	sbc	A,#TIME_FOUR_MAX.n2
	jc	RxFour
	jmp	RxOverflow

RxChanged:
	ld	a,#0
	ld	(rxb_timer0),a
	ld	(rxb_timer1),a
	ld	(rxb_timer2),a
	ld	(rxb_timer3),a

RxNotChanged:

Timer2_Acked:
	ld	a,(A_SRT) ; Restore the A register
	reti

; Flag as a long transition, no packet
RxOverflow:
	ld	a,#1
	ld	(rxb_timer3),a
	ld	a,#0
	ld	(rxb_bits),A
	ld	(rxb_count),A
	ld	a,(PORT_RX)
	and	a,#BIT_RX
	ld	(cpu_last_rx),A
	jmp	RxNotChanged

RxError:
	; For debugging - count errors
	inc	(rxb_err0)
	adr	(rxb_err1)
	
RxReset:
	ld	a,#0
	ld	(rxb_bits),A
	ld	(rxb_count),A
	jmp	RxChanged

; Receive a zero payload bit (one time unit pulse)
RxOne:
	inc	(rxb_count)
	clr	c
	rrc	(rxb_bits)
	jmp	RxChanged
		
; Receive a one payload bit (two time unit pulse)
RxTwo:
	inc	(rxb_count)
	set	c
	rrc	(rxb_bits)
	jmp	RxChanged
	
; Receive an end of packet pulse (four time unit pulse)
RxFour:
	ld	A,(rxb_count)
	cmp	A,#4
	jnz	RxError
	; We have received a command!
	ld	A,(rxb_bits)
	ld	(rxb_cmd),A
	jmp	RxReset


; -----------------------------------------------------------------------------
; MAIN PROGRAM entry point
PGMSRT:
	ld	A,#0
	ld	(SYS0),A	; Disable interrupts
	ld	(USER1),A	; This nybble could be used by user code (and is)
	ld	(USER2),A	; This nybble could be used by user code (but isn't)

	; Initialise input/output (power on but no LED)
	ldpch	PODY_IO_Init
	call	PODY_IO_Init

	; Delay loop (uses SRAM before it is cleared)
	; Should be 11*0xD0000/2 = about 0.585 seconds
	ldmah	#0
	ld	A,#0
	ld	(20H),A
	ld	(21H),A
	ld	(22H),A
	ld	(23H),A
	ld	A,#0DH ; Short delay
	ld	(24H),A ; 20 bit timer 0xD0000
DELAY1:
	ld	A,#WDT_KICK
	ld	(WDT),A ; Kick the watchdog

	ld	A,(20H)
	clr	C
	adc	A,#2
	ld	(20H),A
	adr	(21H)
	adr	(22H)
	adr	(23H)
	adr	(24H)
	jnc	DELAY1

	; Initialise input/output again
	ldpch	PODY_IO_Init
	call	PODY_IO_Init

	; Clear Banks 0..3 of SRAM
	ldmah	#3
	ldpch	Clear_SRAM_INIT
	call	Clear_SRAM_INIT

	ldmah	#2
	ldpch	Clear_SRAM_INIT
	call	Clear_SRAM_INIT

	ldmah	#1
	ldpch	Clear_SRAM_INIT
	call	Clear_SRAM_INIT

	ldmah	#0
	ldpch	Clear_SRAM_INIT
	call	Clear_SRAM_INIT
	
	; Setup timer2 interrupt every 100uS
	ldmah	#0
	ldpch	Timer2_Init
	call	Timer2_Init

	; Turn on the LED
	ld	A,#15 ; Full on
	ld	(g_pwm),A
	clr	#PIN_LED,(PORT_LED)
	
	; Initialise the button variables
	ld	a,#SAT_HIGH
	ld	(ButtonCnt1),A
	ld	a,#0
	ld	(ButtonCnt0),A
	ld	(ButtonPress),A
	ld	(PowerOff),A
	ld	a,#1
	ld	(ButtonOn),A
	ld	(ButtonInit),A

; -----------------------------------------------------------------------------
; Where wakeup code would return to
WakeUp:
	nop
	nop
	; We want the RTC interrupt to be every 1s instead of 125ms to avoid disruption
	set	#1,(RTC)
	set	#0,(RTC)

; -----------------------------------------------------------------------------
; Main loop
MAIN_LOOP:
	; Kick the watchdog
	nop
	nop
	ld	a,#WDT_KICK
	ld	(WDT),a
	set	#1,(SYS0) ; Enable interrupts
	nop	; paranoid

	; Check the output direction
	ld	a,#0101b	; 
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#0001b
	ld	(IOC_PB),a	; Port B dir (0=input/1=output)

	; Throttle the main loop to 1.6ms
	ld	a,(g_timer1)
Wait_irq:
	cmp	a,(g_timer1)
	jz	Wait_irq

	; Check for input from CPU and debounce it (100ms required)
	ld	A,(PowerOff)
	jnz	Main_RxOK
	ld	a,(PORT_RX)
	xor	a,#XOR_RX
	and	a,#BIT_RX
	jz	Main_RxLow
	; Debounce
	inc	(rx_timer0)
	adr	(rx_timer1)
	ld	a,(rx_timer1)
	cmp	a,#TIMEOUT_RX.n1
;;	jz	StartPowerOff	; DEBUG BODGE - DONT LET CPU TURN ME OFF
	jmp	Main_RxOK

Main_RxLow:
	ld	a,#0
	ld	(rx_timer0),a
	ld	(rx_timer1),a
Main_RxOK:
	
	; Check for input from user and debounce it (25ms required)
	ld	a,(PORT_USR)
	and	a,#BIT_USR
	jz	Main_UsrLow

	; Usr pin is high, hence pressed. Debounce this.
	ld	a,#0
	ld	(ButtonCnt0),a
	ld	a,(ButtonCnt1)
	cmp	a,#SAT_HIGH
	jz	Saturated1
	inc	(ButtonCnt1)
	jmp	NotSaturated
	
Main_UsrLow:
	; Usr pin is low, hence released. Debounce this.
	ld	a,#0
	ld	(ButtonCnt1),a
	ld	a,(ButtonCnt0)
	cmp	a,#SAT_LOW
	jz	Saturated0
	inc	(ButtonCnt0)
	jmp	NotSaturated
Saturated0:
	ld	a,#0
	ld	(ButtonOn),a
	ld	(ButtonInit),A
	jmp	NotSaturated
Saturated1:
	ld	a,#1
	ld	(ButtonOn),a
NotSaturated:

	; Have we pressed the button?
	ld	a,(ButtonInit)
	jnz	StillInit
	ld	A,(ButtonOn)
	jz	StillOff
	ld	a,#1
	ld	(ButtonPress),a
StillOff:
StillInit:

	; Have we released the button after a press?
	ld	A,(ButtonOn)
	jnz	StillOn
	ld	A,(ButtonPress)
	jz	StillOn
	; We have pressed and released the button: trigger the off sequence
	ld	A,(PowerOff)
	jnz	StillOn
	; Start the sequence
StartPowerOff:
	ld	a,#1
	ld	(PowerOff),A
	ld	a,#0
	ld	(off_timer0),a
	ld	(off_timer1),a
	ld	(off_phase0),a
	ld	(off_phase1),a
	ld	a,(PORT_RX)
	xor	a,#XOR_RX
	and	a,#BIT_RX
	ld	(cpu_rx),a
	set	#PIN_TX,(PORT_TX) ; Tell the CPU we are switching off
StillOn:

	; Perform power off sequence
	ld	A,(PowerOff)
	jz	PowerStillOn
	inc	(off_timer0)
	adr	(off_timer1)
	ld	a,(off_timer1)
	cmp	a,#TIMEOUT_PHASE.n1
	jnz	PowerSamePhase
	; Increase the phase every 300 ms
	ld	a,#0
	ld	(off_timer1),a
	ld	(off_timer0),a ; Unneeded
	inc	(off_phase0)
	adr	(off_phase1)

	ld	a,(PORT_RX)
	xor	a,#XOR_RX
	and	a,#BIT_RX
	xor	a,(cpu_rx)
	jz	PowerWaitCpu
	; Fast timeout
	ld	a,(off_phase0)
	cmp	a,#TIMEOUT_FAST.n0
	ld	a,(off_phase1)
	sbc	a,#TIMEOUT_FAST.n1
	jnc	PowerNowOff ; We are in the turn off phase
	jmp	PowerSamePhase

	; We are in shut down phase and CPU has not acknowledged yet
PowerWaitCpu:
	ld	a,(off_phase0)
	cmp	a,#TIMEOUT_SLOW.n0
	ld	a,(off_phase1)
	sbc	a,#TIMEOUT_SLOW.n1
	jnc	PowerNowOff ; We are in the turn off phase
	
PowerSamePhase:

	; Set the visible LED on or off
	ld	a,(off_phase0)
	and	a,#1
	jz	PowerOffLed
	; Turn on the LED
	ld	A,#8
	ld	(g_pwm),A
;	clr	#PIN_LED,(PORT_LED)
	jmp	PowerStillOn
PowerOffLed:
	ld	A,#0
	ld	(g_pwm),A
;	set	#PIN_LED,(PORT_LED)
	
PowerStillOn:

	LDPCH	MAIN_LOOP
	JMP	MAIN_LOOP
	
; -----------------------------------------------------------------------------
; Turn off the power to the CPU
PowerNowOff:
;	set	#PIN_TX,(PORT_TX)
;	clr	#PIN_RX,(PORT_RX)
;	set	#PIN_LED,(PORT_LED)
	ld	a,#1111b	; 
	ld	(data_pa),a	; Port A data (0=low/1=high)
;	clr	#PIN_PWR,(PORT_PWR)
;	clr	#PIN_USR,(PORT_USR)
	ld	a,#0000b
	ld	(data_pb),a	; Port B data (0=low/1=high)

	; Wait 200ms - on real hardware during this time I would lose my own power
	ld	a,(g_timer2)
	clr	c
	adc	a,#8
Wait2_irq:
	cmp	a,(g_timer2)
	jnz	Wait2_irq

	clr	#1,(SYS0) ; Clear ENINT and disable interrupts
	nop
	nop
	halt 
HaltEnd:
	ldpch	HaltEnd
	jmp	HaltEnd

; -----------------------------------------------------------------------------
PODY_IO_Init:
	; PIN A0  LED3    (active high)     = output low
	; PIN A1  LED2    (active high)     = input pull low
	; PIN A2  LED     (active low)      = output high
	; PIN A3          (unused)          = input pull high
	ld	a,#0101b	; 
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
;	clr	#PIN_TX,(PORT_TX)
;	clr	#PIN_RX,(PORT_RX)
;	set	#PIN_LED,(PORT_LED)
	ld	a,#1110b
	ld	(data_pa),a	; Port A data (0=low/1=high)
	ld	a,#0000b
	ld	exio(pawk),a	; Port A wakeup - none
	ld	a,#1010b
	ld	exio(papu),a	; Port A pull up 100kOhm resistor
	ld	a,#0000b
	ld	exio(papl),a	; Port A pull down 100kOhm resistor

	; Pin B0 PWR      (active high) = output high
	; Pin B1 USR      (active high) = input pull low (wake)
	; Pin B2          (unused)      = input pull low
	; Pin B3          (unused)      = input pull low
	ld	a,#0001b
	ld	(IOC_PB),a	; Port B dir (0=input/1=output)
;	set	#PIN_PWR,(PORT_PWR)
;	clr	#PIN_USR,(PORT_USR)
	ld	a,#0001b
	ld	(data_pb),a	; Port B data (0=low/1=high)
	ld	a,#0010b
	ld	exio(pbwk),a	; Port B wakeup
	ld	a,#0000b
	ld	exio(pbpu),a	; Port B pull up 100kOhm resistor 
	ld	a,#1110b
	ld	exio(pbpl),a	; Port B pull down 100kOhm resistor
	rets
	
	
; -----------------------------------------------------------------------------
; Clear a single bank (32 nybbles) of SRAM
Clear_SRAM_INIT:
	ld	A,#00H
	ld	(20H),A
	ld	(21H),A
	ld	(22H),A
	ld	(23H),A
	ld	(24H),A
	ld	(25H),A
	ld	(26H),A
	ld	(27H),A
	ld	(28H),A
	ld	(29H),A
	ld	(2AH),A
	ld	(2BH),A
	ld	(2CH),A
	ld	(2DH),A
	ld	(2EH),A
	ld	(2FH),A
	ld	(30H),A
	ld	(31H),A
	ld	(32H),A
	ld	(33H),A
	ld	(34H),A
	ld	(35H),A
	ld	(36H),A
	ld	(37H),A
	ld	(38H),A
	ld	(39H),A
	ld	(3AH),A
	ld	(3BH),A
	ld	(3CH),A
	ld	(3DH),A
	ld	(3EH),A
	ld	(3FH),A	 
	rets

; -----------------------------------------------------------------------------
; Set up timer 2 for 100uS interrupt (10kHz) +/- 2%
; High speed oscillator HRCOSC = 32Mhz
; CPU clock FMCK = /4 = 8Mhz
; Scaler2 = Div8 = 1MHz
; Timer2 = 0x9C = 10kHz
Timer2_Init:
	ld	a,#0
	ld	(TMCTL),A;3:TM2EN,2:TM1EN,1:TM1SCK,0:TM1ALD
	nop 
	set	#3,(TMCTL)
	set	#3,(SYS0) ;3:TM2SK,2:TM1SK,1:ENINI,0:PWM0

	clr	#1,(SYS0) ; Clear ENINI (disable interrupts)
	nop	; Safety measure from clearing global interrupts
	clr	#0,(SYS0) ; Clear PWM0 mode
	nop

	; Set Timer2 Autoload enabled and scalar to div8 (i.e. 1Mhz)
	ld	a,#1101b
	ld	(SCALER2),A ;div2-div1-div0. 8M/8=1. auto download

	; timing = (256 - time val/(time clock)) so 0xCE is 50uS
	ld	A,#Tim2_Speed.N0 ; Low nybble
	ld	(TIM2),A
	ld	A,#Tim2_Speed.N1 ; High nybble
	ld	(TIM2),A

;	set	#1,(SYS0) ; Set ENINI
	nop
Timr2_Init_End:
	rets

; -----------------------------------------------------------------------------

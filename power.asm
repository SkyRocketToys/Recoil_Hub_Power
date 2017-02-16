; -----------------------------------------------------------------------------
; Recoil Gun Base Station Power control chip
; (c) 2017 HotGen Ltd.
; Written for the TR4P151AT chip
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; PINOUT
; 1. VCC
; 2. PB1 = USR        (active high input)
; 3. PB0 = PWR        (active high output)
; 4. PA3 = unused
; 
; 5. PA2 = LED        (active low output)
; 6. PA1 = PWR_CTRL_1 (input)
; 7. PA0 = PWR_CTRL_2 (input)
; 8. GND
; 
; GPIO
; PA0 = PWR_CTRL_2/LED3 = input
; PA1 = PWR_CTRL_1/LED2 = input
; PA2 = LED             = output high
; PA3 = unused          = input pull high
; PB0 = PWR             = output high
; PB1 = USR             = input pull low
; 
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Behaviour of inputs
; 
; PWR_CTRL_1 signal = ignored until we know what it is for - probably for the CPU to tell us to turn off
; PWR_CTRL_2 signal = ignored until we know what it is for - probably for us to warn the CPU of imminant power off
; USR = press this button for a small amount of time in order to trigger power off
; 
; -----------------------------------------------------------------------------
; Behaviour of outputs
; 
; PWR = On power on, turn this on.
;       On power off turn this off (after the LED sequence).
; LED = on power on, turn on after a short delay
;       on power off, flash thrice and turn off
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Defines
; -----------------------------------------------------------------------------

; -----------------------------------------------------------------------------
; Include Block
; -----------------------------------------------------------------------------

#include     "power.inc"

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
off_timer0	; Counts up every   1.6ms
off_timer1	; Counts up every  25.6ms
off_phase	; Counts up every 307.2ms (phases: LED 0=off 1=on 2=off 3=on 4=off 5=on 6=poweroff)
}


; -----------------------------------------------------------------------------
; General Parameters for the program
VINT_MAH        equ	00    ; The SRAM bank we want to use inside the interrupt [Mind you, Optima overrides this]

Tim2_Speed	equ	256-100	; 100uS = 10kHz

PORT_USR	equ	data_pb
PIN_USR		equ	1
BIT_USR		equ	1 << PIN_USR

PORT_LED	equ	data_pa
PIN_LED		equ	2
BIT_LED		equ	1 << PIN_LED

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
	
Timer2_Acked:
	
	ld	a,(A_SRT) ; Restore the A register
	reti


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
	ld	A,#05H
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
	clr	#PIN_LED,(PORT_LED)
	
	; Initialise the button variables
	ld	a,#15
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
	ld	a,#05h
	ld	(WDT),a
	set	#1,(SYS0) ; Enable interrupts
	nop	; paranoid

	; Check the output direction
	ld	a,#0100b
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#0001b
	ld	(IOC_PB),a	; Port B dir (0=input/1=output)

	; Throttle the main loop to 1.6ms
	ld	a,(g_timer1)
Wait_irq:
	cmp	a,(g_timer1)
	jz	Wait_irq
	
	; Check for input from user and debounce it (25ms required)
	ld	a,(PORT_USR)
	and	a,#BIT_USR
	jz	Main_UsrLow

	; Usr pin is high, hence pressed. Debounce this.
	ld	a,#0
	ld	(ButtonCnt0),a
	ld	a,(ButtonCnt1)
	cmp	a,#15
	jz	Saturated1
	inc	(ButtonCnt1)
	jmp	NotSaturated
	
Main_UsrLow:
	; Usr pin is low, hence released. Debounce this.
	ld	a,#0
	ld	(ButtonCnt1),a
	ld	a,(ButtonCnt0)
	cmp	a,#15
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
	jz	StillInit
	ld	A,(ButtonOn)
	jz	StillOff
	ld	a,#1
	ld	(ButtonPress),a
StillOff:
StillInit:

	; Have we released the button after a press?
	ld	(ButtonOn),A
	jnz	StillOn
	ld	(ButtonPress),A
	jz	StillOn
	; We have pressed and released the button: trigger the off sequence
	ld	A,(PowerOff)
	jz	StillOn
	; Start the sequence
	ld	a,#1
	ld	(PowerOff),A
	ld	a,#0
	ld	(off_timer0),a
	ld	(off_timer1),a
	ld	(off_phase),a
StillOn:

	; Perform power off sequence
	ld	A,(PowerOff)
	jz	PowerStillOn
	inc	(off_timer0)
	adr	(off_timer1)
	ld	a,(off_timer1)
	cmp	a,#12
	jnz	PowerSamePhase
	; Increase the phase every 300 ms
	ld	a,#0
	ld	(off_timer1),a
	ld	(off_timer0),a ; Unneeded
	inc	(off_phase)
	ld	a,(off_phase)
	cmp	a,#6
	jz	PowerNowOff ; We are in the turn off phase
PowerSamePhase:

	; Set the visible LED on or off
	ld	a,(off_phase)
	and	a,#1
	jz	PowerOffLed
	; Turn on the LED
	clr	#PIN_LED,(PORT_LED)
	jmp	PowerStillOn
PowerOffLed:
	set	#PIN_LED,(PORT_LED)
	
PowerStillOn:

	LDPCH	MAIN_LOOP
	JMP	MAIN_LOOP
	
; -----------------------------------------------------------------------------
; Turn off the power to the CPU
PowerNowOff:
	ld	a,#1000b
	ld	(data_pa),a	; Port A data (0=low/1=high)
	ld	a,#0000b
	ld	(data_pb),a	; Port B data (0=low/1=high)

	clr	#1,(SYS0) ; Clear ENINT and disable interrupts
	nop
	nop
	halt 
HaltEnd:
	ldpch	HaltEnd
	jmp	HaltEnd

; -----------------------------------------------------------------------------
PODY_IO_Init:
	; PIN A0  LED3    (active unknown)  = input no pull
	; PIN A1  LED2    (active unknown)  = input no pull
	; PIN A2  LED     (active low)      = output high
	; PIN A3          (unused)          = input pull high
	ld	a,#0100b
	ld	(IOC_PA),a	; Port A direction (0=input/1=output)
	ld	a,#1100b
	ld	(data_pa),a	; Port A data (0=low/1=high)
	ld	a,#0000b
	ld	exio(pawk),a	; Port A wakeup - none
	ld	a,#1000b
	ld	exio(papu),a	; Port A pull up 100kOhm resistor
	ld	a,#0000b
	ld	exio(papl),a	; Port A pull down 100kOhm resistor

	; Pin B0 PWR      (active high) = output high
	; Pin B1 USR      (active high) = input pull low (wake)
	; Pin B2          (unused)      = input pull low
	; Pin B3          (unused)      = input pull low
	ld	a,#0001b
	ld	(IOC_PB),a	; Port B dir (0=input/1=output)
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

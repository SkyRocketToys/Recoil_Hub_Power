;****************************************************************************
;                    151 ASM MCU Code
;****************************************************************************


;----------------------
;COMMON I/O            
;----------------------
 
STATUS	EQU	00H	; R/W, BIT0: Zero.
			; R/W, BIT1: Carry.
			; R/W, BIT2: Timer1 interrupt flag.
			; R/W, BIT3: Timer2 interrupt flag.

RTC	EQU	01H	; R/W,
			; b1, b0: Select RTC interrupt period.
			;  0, 0 : 0.125SEC.
			;  0, 1 : 0.25SEC.
			;  1, 0 : 0.5SEC.
			;  1, 1 :   1SEC.
			; b2: 38k clock control, 0: PA1 output keep low or high for option. 
			;                        1: PA1 output 38k clock.
			; b3: RTC overflow flag.

IOC_PA	EQU	02H	; R/W, PORT A IO CONTROL, '0'=INPUT, '1'=OUTPUT, PA3 only input.
DATA_PA	EQU	03H	; R/W, PORT A DATA READ/WRITE
ADCCT	EQU	04H	; R/W, Simple ADC control register, only support TR4P153CT
IOC_PB	EQU	05H	; R/W, PORT B IO CONTROL, '0'=INPUT, '1'=OUTPUT
DATA_PB	EQU	06H	; R/W, PORT B DATA READ/WRITE

USER1	EQU	07H	; R/W, General purpose user register1

TMCTL	EQU	08H	; b0: Timer1 auto load control. 0:Timer1 auto load off, 1:Timer1 auto load on.
			; b1: Timer1 clock source select. External PA2 input CLK
			; b2: Timer 1 enable control.
			; b3: Timer 2 enable control.

SYS0	EQU	09H	; b0: PWM signal output to PA2. 0: PA2 is I/O pin, 1: PA2 is PWM output pin.
			; b1: Control global interrupt.(ENINT control the interrupt enable of PWMINT /TIMINT)
			;     0:  disable global interrupt, 1:  enable global interrupt
			; b2: Timer 1 interrupt mask control. 0:  disable interrupt, 1:  enable interrupt.
			; b3: Timer 2 interrupt mask control. 0:  disable interrupt, 1:  enable interrupt.

TIM1	EQU	0AH	; R/W, Timer1 8bit timer control.
TIM2	EQU	0BH	; R/W, Timer2 8bit timer control.

SCALER1	EQU	0EH	; R/W, SCALER is the pre-scaler of TIMER1
			; b2~b0
			; 0,0,0 : SYSTEM CLK/256
			; 0,0,1 : SYSTEM CLK/128
			; 0,1,0 : SYSTEM CLK/64
			; 0,1,1 : SYSTEM CLK/32
			; 1,0,0 : SYSTEM CLK/16
			; 1,0,1 : SYSTEM CLK/8
			; 1,1,0 : SYSTEM CLK/4
			; 1,1,1 : SYSTEM CLK/2
			; b3: TM1 time out toggle signal out put control. ( PA2 pad define )
			;     0: Disabled, PA2 port is I/O function.
			;     1: Enabled, TM1 toggle output.

USER2	EQU	0FH	; R/W, General purpose user register2

MDCTL	EQU	10H	; W, IC mode control.
			; b3, b2
			; MD1,MD0=00  Stop mode
			; MD1,MD0=01  Green to Normal mode
			; MD1,MD0=10  Green mode
			; MD1,MD0=11  NC

DMA0	EQU	18H	; R/W,DATA ROM ADDRESS LOWEST POINTER
DMA1	EQU	19H	; R/W,2ND ADDRESS
DMA2	EQU	1AH	; R/W,3RD

RES_FLAG EQU 	1BH	; R, b2: WDT overflow flag,   0: WDTFG is cleared by executing clear
 			;			         WDT (writing 05h to DMDM register)
 			;			         or after power up. 
 			;			      1: WDTFG is set by WDT overflow occurred.
 			;    b3: External reset flag, 0: ERESFG is cleared by executing clear
 			;			         WDT (writing 05h to DMDM register)
 			;			         or after power up. 
 			;			      1: WDTFG is set by WDT overflow occurred.	

DMDL	EQU	1CH	; R, Read rom data,LOW NIBBLE			
DMDM	EQU	1DH	; R, Read rom data,Middle NIBBLE 
WDT	EQU	1DH	; W, WATCH DOG TIMER, WRITE 05H TO THIS REGISTER----CLEAR WDT
DMDH	EQU	1EH	; R, Read rom data,HIGH NIBBLE

WATCH	EQU	1DH

SCALER2	EQU	1FH	; R/W, SCALER is the pre-scaler of TIMER2
			; b2~b0
			; 0,0,0 : SYSTEM CLK/256
			; 0,0,1 : SYSTEM CLK/128
			; 0,1,0 : SYSTEM CLK/64
			; 0,1,1 : SYSTEM CLK/32
			; 1,0,0 : SYSTEM CLK/16
			; 1,0,1 : SYSTEM CLK/8
			; 1,1,0 : SYSTEM CLK/4
			; 1,1,1 : SYSTEM CLK/2
			; Timer2 auto load control. 0:Timer2 auto load off, 1:Timer2 auto load on.

;------------------------
;EXTENSION I/O 		
;------------------------
EXIOSTART.
PAPU		EQU	00H	; W, PORT A pull up control
PAPL		EQU	01H	; W, PORT A pull down control
PBPU		EQU	02H	; W, PORT B pull up control
PBPL		EQU	03H	; W, PORT B pull down control
PAWK		EQU	06H	; W, PORT A wake up control 
PBWK		EQU	07H	; W, PORT B wake up control 
EXIOEND.